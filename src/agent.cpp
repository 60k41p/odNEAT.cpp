#include "agent.h"

#include "mutation.h"

namespace odneat {

    OdneatAgent::OdneatAgent(std::uint32_t robot_identifier,
                             int input_count,
                             int output_count,
                             double minimum_energy,
                             double maximum_energy,
                             double default_energy,
                             double minimum_threshold,
                             const OdneatParams &parameters,
                             std::uint64_t random_seed)
        : robot_identifier_(robot_identifier),
          input_count_(input_count),
          output_count_(output_count),
          parameters_(parameters),
          random_generator_(random_seed),
          innovation_clock_(robot_identifier),
          active_genome_(Genome::createMinimalGenome(input_count, output_count, innovation_clock_)),
          active_controller_(RecurrentNetwork::decodeFromGenome(active_genome_, input_count, output_count)),
          energy_tracker_(minimum_energy, maximum_energy, default_energy, minimum_threshold),
          fitness_averager_(),
          population_(parameters.internal_population_size,
                      parameters.disjoint_coefficient,
                      parameters.excess_coefficient,
                      parameters.weight_difference_coefficient,
                      parameters.compatibility_threshold),
          tabu_list_(parameters.disjoint_coefficient,
                     parameters.excess_coefficient,
                     parameters.weight_difference_coefficient,
                     parameters.compatibility_threshold,
                     parameters.tabu_expiry_history_size),
          broadcast_policy_(),
          maturation_cycles_remaining_(parameters.maturation_period_cycles),
          evaluation_count_(1),
          exchange_enabled_(true),
          tabu_enabled_(true),
          maturation_enabled_(true),
          speciation_enabled_(true) {
        active_genome_.setFitness(energy_tracker_.getEnergy());
        active_genome_.setEvaluationCount(1);
        population_.addGenome(active_genome_);
    }

    std::uint32_t OdneatAgent::getRobotIdentifier() const { return robot_identifier_; }

    const Genome &OdneatAgent::getActiveGenome() const { return active_genome_; }

    double OdneatAgent::getEnergy() const { return energy_tracker_.getEnergy(); }

    double OdneatAgent::getFitness() const { return fitness_averager_.getFitness(); }

    const InternalPopulation &OdneatAgent::getPopulation() const { return population_; }

    const TabuList &OdneatAgent::getTabuList() const { return tabu_list_; }

    int OdneatAgent::getMaturationCyclesRemaining() const { return maturation_cycles_remaining_; }

    int OdneatAgent::getEvaluationCount() const { return evaluation_count_; }

    std::mt19937_64 &OdneatAgent::accessRandomGenerator() { return random_generator_; }

    AgentStepResult OdneatAgent::executeControlCycle(const std::vector<double> &sensor_inputs,
                                                     double energy_delta,
                                                     const std::vector<ReceivedGenome> &received_genomes) {
        AgentStepResult step_result{};
        Genome broadcast_genome{};
        double broadcast_energy = 0.0;
        step_result.did_broadcast = prepareBroadcast(broadcast_genome, broadcast_energy);
        int accepted_count = 0;
        int merged_count = 0;
        int rejected_count = 0;
        incorporateReceivedGenomes(received_genomes, accepted_count, merged_count, rejected_count);
        step_result.accepted_genome_count = accepted_count;
        step_result.merged_duplicate_count = merged_count;
        step_result.tabu_rejected_count = rejected_count;
        const std::vector<double> network_outputs = active_controller_.stepNetwork(sensor_inputs);

        if (network_outputs.size() >= 2) {
            step_result.left_wheel_speed = rescaleOutputToWheelSpeed(network_outputs[0]);
            step_result.right_wheel_speed = rescaleOutputToWheelSpeed(network_outputs[1]);
        }

        energy_tracker_.addEnergyDelta(energy_delta);
        fitness_averager_.addEnergySample(energy_tracker_.getEnergy());
        active_genome_.setFitness(fitness_averager_.getFitness());
        active_genome_.setEvaluationCount(fitness_averager_.getSampleCount());
        step_result.applied_energy_delta = energy_delta;

        if (maturation_cycles_remaining_ > 0) {
            --maturation_cycles_remaining_;
        }

        const bool maturation_protects = maturation_enabled_ && (maturation_cycles_remaining_ > 0);

        if (energy_tracker_.isDepleted() && !maturation_protects) {
            tabu_list_.addTabuGenome(active_genome_);
            replaceControllerWithOffspring();
            step_result.did_replace_controller = true;
        }

        return step_result;
    }

    void OdneatAgent::incorporateExternal(const std::vector<ReceivedGenome> &received_genomes, int &accepted_count, int &merged_count, int &rejected_count) {
        incorporateReceivedGenomes(received_genomes, accepted_count, merged_count, rejected_count);
    }

    std::vector<double> OdneatAgent::stepController(const std::vector<double> &sensor_inputs) { return active_controller_.stepNetwork(sensor_inputs); }

    bool OdneatAgent::updateEnergyAndMaybeReplace(double energy_delta) {
        energy_tracker_.addEnergyDelta(energy_delta);
        fitness_averager_.addEnergySample(energy_tracker_.getEnergy());
        active_genome_.setFitness(fitness_averager_.getFitness());
        active_genome_.setEvaluationCount(fitness_averager_.getSampleCount());
        if (maturation_cycles_remaining_ > 0) {
            --maturation_cycles_remaining_;
        }

        const bool maturation_protects = maturation_enabled_ && (maturation_cycles_remaining_ > 0);
        if (energy_tracker_.isDepleted() && !maturation_protects) {
            tabu_list_.addTabuGenome(active_genome_);
            replaceControllerWithOffspring();
            return true;
        }

        return false;
    }

    bool OdneatAgent::prepareBroadcast(Genome &broadcast_genome, double &broadcast_energy) {
        // Sync the stored copy with the active genome's live fitness before applying Eq. 2:
        // per-cycle energy samples otherwise never reach the population statistics.
        population_.syncStoredFitness(active_genome_, active_genome_.getFitness(), active_genome_.getEvaluationCount());
        const double total_fitness = population_.computeTotalAverageFitness();
        std::size_t active_index = 0;
        bool found_active = false;
        const std::vector<Genome> &stored_genomes = population_.getGenomes();

        for (std::size_t genome_index = 0; genome_index < stored_genomes.size(); ++genome_index) {
            if (stored_genomes[genome_index].isIdenticalTo(active_genome_)) {
                active_index = genome_index;
                found_active = true;
                break;
            }
        }

        double active_species_fitness = 0.0;

        if (found_active) {
            active_species_fitness = population_.findSpeciesFitnessForGenome(active_index);
        } else if (!population_.getSpecies().empty()) {
            // Active copy evicted while full: neutral order-independent estimate (species mean).
            active_species_fitness = total_fitness / static_cast<double>(population_.getSpecies().size());
        }

        const bool should_send = broadcast_policy_.shouldBroadcast(active_species_fitness, total_fitness, random_generator_);

        if (should_send) {
            broadcast_genome = active_genome_;
            broadcast_energy = energy_tracker_.getEnergy();
        }

        return should_send;
    }

    void OdneatAgent::forceControllerReplacement() {
        tabu_list_.addTabuGenome(active_genome_);
        replaceControllerWithOffspring();
    }

    void OdneatAgent::setExchangeEnabled(bool exchange_enabled) { exchange_enabled_ = exchange_enabled; }

    void OdneatAgent::setTabuEnabled(bool tabu_enabled) { tabu_enabled_ = tabu_enabled; }

    void OdneatAgent::setMaturationEnabled(bool maturation_enabled) { maturation_enabled_ = maturation_enabled; }

    void OdneatAgent::setSpeciationEnabled(bool speciation_enabled) {
        speciation_enabled_ = speciation_enabled;
        population_.setNichingEnabled(speciation_enabled);
    }

    void OdneatAgent::incorporateReceivedGenomes(const std::vector<ReceivedGenome> &received_genomes,
                                                 int &accepted_count,
                                                 int &merged_count,
                                                 int &rejected_count) {
        accepted_count = 0;
        merged_count = 0;
        rejected_count = 0;

        for (const ReceivedGenome &received_genome : received_genomes) {
            tabu_list_.observeReceivedGenome(received_genome.genome);

            if (!exchange_enabled_) {
                continue;
            }

            if (population_.containsIdenticalGenome(received_genome.genome)) {
                population_.mergeDuplicateFitness(received_genome.genome, received_genome.sender_energy);
                ++merged_count;
                continue;
            }

            if (tabu_enabled_ && !tabu_list_.approvesCandidate(received_genome.genome)) {
                ++rejected_count;
                continue;
            }

            Genome candidate_genome = received_genome.genome;
            candidate_genome.setEvaluationCount(1);

            if (candidate_genome.getFitness() == 0.0) {
                candidate_genome.setFitness(received_genome.sender_energy);
            }

            Genome evicted_genome{};
            bool has_eviction = false;
            population_.addGenomeWithEviction(candidate_genome, evicted_genome, has_eviction);

            if (has_eviction) {
                tabu_list_.addTabuGenome(evicted_genome);
            }

            ++accepted_count;
        }
    }

    void OdneatAgent::replaceControllerWithOffspring() {
        // Sync the outgoing genome's final fitness so parent selection sees its full evaluation.
        population_.syncStoredFitness(active_genome_, active_genome_.getFitness(), active_genome_.getEvaluationCount());
        Genome offspring{};

        if (population_.isEmpty()) {
            offspring = Genome::createMinimalGenome(input_count_, output_count_, innovation_clock_);
        } else {
            const int parent_species = population_.selectParentSpecies(random_generator_);

            if (parent_species < 0) {
                offspring = population_.getGenomes().front();
                mutateGenome(offspring, innovation_clock_, random_generator_, parameters_, nullptr);
            } else {
                const std::size_t first_parent_index = population_.tournamentSelectInSpecies(parent_species, random_generator_);
                const std::size_t second_parent_index = population_.tournamentSelectInSpecies(parent_species, random_generator_);
                const Genome &first_parent = population_.getGenomes()[first_parent_index];
                const Genome &second_parent = population_.getGenomes()[second_parent_index];

                if (first_parent_index == second_parent_index) {
                    offspring = first_parent;
                    mutateGenome(offspring, innovation_clock_, random_generator_, parameters_, nullptr);
                } else {
                    offspring = generateOffspring(first_parent, &second_parent, innovation_clock_, random_generator_, parameters_);
                }
            }
        }

        population_.addGenome(offspring);
        active_genome_ = offspring;
        reinstallActiveController();
        ++evaluation_count_;
    }

    void OdneatAgent::reinstallActiveController() {
        active_controller_ = RecurrentNetwork::decodeFromGenome(active_genome_, input_count_, output_count_);
        active_controller_.resetActivations();
        energy_tracker_.refillToDefault();
        fitness_averager_.resetAverager();
        fitness_averager_.addEnergySample(energy_tracker_.getEnergy());
        active_genome_.setFitness(fitness_averager_.getFitness());
        active_genome_.setEvaluationCount(fitness_averager_.getSampleCount());

        if (maturation_enabled_) {
            maturation_cycles_remaining_ = parameters_.maturation_period_cycles;
        } else {
            maturation_cycles_remaining_ = 0;
        }
    }

}  // namespace odneat
