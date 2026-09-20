#include "population.h"

#include <limits>

#include "config.h"

namespace odneat {

    InternalPopulation::InternalPopulation(int maximum_population_size,
                                           double disjoint_coefficient,
                                           double excess_coefficient,
                                           double weight_difference_coefficient,
                                           double kCompatibilityThreshold)
        : maximum_population_size_(maximum_population_size),
          disjoint_coefficient_(disjoint_coefficient),
          excess_coefficient_(excess_coefficient),
          weight_difference_coefficient_(weight_difference_coefficient),
          compatibility_threshold_(kCompatibilityThreshold) {}

    const std::vector<Genome> &InternalPopulation::getGenomes() const { return genomes_; }

    const std::vector<Species> &InternalPopulation::getSpecies() const { return species_list_; }

    int InternalPopulation::getMaximumSize() const { return maximum_population_size_; }

    std::size_t InternalPopulation::getCurrentSize() const { return genomes_.size(); }

    bool InternalPopulation::isEmpty() const { return genomes_.empty(); }

    bool InternalPopulation::isFull() const { return genomes_.size() >= static_cast<std::size_t>(maximum_population_size_); }

    bool InternalPopulation::containsIdenticalGenome(const Genome &candidate_genome) const {
        for (const Genome &stored_genome : genomes_) {
            if (stored_genome.isIdenticalTo(candidate_genome)) {
                return true;
            }
        }

        return false;
    }

    bool InternalPopulation::mergeDuplicateFitness(const Genome &received_genome, double remote_energy) {
        for (Genome &stored_genome : genomes_) {
            if (stored_genome.isIdenticalTo(received_genome)) {
                const int previous_count = stored_genome.getEvaluationCount();
                const double previous_fitness = stored_genome.getFitness();
                const int merged_count = previous_count + 1;
                const double merged_fitness = (previous_fitness * static_cast<double>(previous_count) + remote_energy) / static_cast<double>(merged_count);
                stored_genome.setFitness(merged_fitness);
                stored_genome.setEvaluationCount(merged_count);
                refreshSpeciation();

                return true;
            }
        }

        return false;
    }

    PopulationAcceptance InternalPopulation::addGenome(const Genome &candidate_genome) {
        Genome evicted_genome{};
        bool has_eviction = false;
        return addGenomeWithEviction(candidate_genome, evicted_genome, has_eviction);
    }

    PopulationAcceptance InternalPopulation::addGenomeWithEviction(const Genome &candidate_genome, Genome &evicted_genome, bool &has_eviction) {
        has_eviction = false;

        for (Genome &stored_genome : genomes_) {
            if (stored_genome.isIdenticalTo(candidate_genome)) {
                const int previous_count = stored_genome.getEvaluationCount();
                const double merged_fitness = (stored_genome.getFitness() * static_cast<double>(previous_count) + candidate_genome.getFitness()) /
                                              static_cast<double>(previous_count + 1);
                stored_genome.setFitness(merged_fitness);
                stored_genome.setEvaluationCount(previous_count + 1);
                refreshSpeciation();

                return PopulationAcceptance::kDuplicateMerged;
            }
        }

        if (!isFull()) {
            genomes_.push_back(candidate_genome);
            refreshSpeciation();

            return PopulationAcceptance::kAccepted;
        }

        std::size_t worst_index = 0;
        double worst_fitness = std::numeric_limits<double>::max();

        for (std::size_t genome_index = 0; genome_index < genomes_.size(); ++genome_index) {
            if (genomes_[genome_index].getAdjustedFitness() < worst_fitness) {
                worst_fitness = genomes_[genome_index].getAdjustedFitness();
                worst_index = genome_index;
            }
        }

        evicted_genome = genomes_[worst_index];
        has_eviction = true;
        genomes_[worst_index] = candidate_genome;
        refreshSpeciation();

        return PopulationAcceptance::kAccepted;
    }

    int InternalPopulation::selectParentSpecies(std::mt19937_64 &randomGenerator) const {
        if (species_list_.empty()) {
            return -1;
        }

        const double total_fitness = computeTotalAverageFitness();

        if (total_fitness <= 0.0) {
            std::uniform_int_distribution<std::size_t> uniformSpecies(0, species_list_.size() - 1);

            return static_cast<int>(uniformSpecies(randomGenerator));
        }

        std::uniform_real_distribution<double> roulette(0.0, total_fitness);
        double roulette_ball = roulette(randomGenerator);

        for (std::size_t species_index = 0; species_index < species_list_.size(); ++species_index) {
            roulette_ball -= species_list_[species_index].average_adjusted_fitness;
            if (roulette_ball <= 0.0) {
                return static_cast<int>(species_index);
            }
        }

        return static_cast<int>(species_list_.size() - 1);
    }

    std::size_t InternalPopulation::tournamentSelectInSpecies(int species_index, std::mt19937_64 &randomGenerator) const {
        const Species &species = species_list_.at(static_cast<std::size_t>(species_index));
        std::uniform_int_distribution<std::size_t> memberPicker(0, species.member_indices.size() - 1);
        const std::size_t first_contender = species.member_indices[memberPicker(randomGenerator)];
        const std::size_t second_contender = species.member_indices[memberPicker(randomGenerator)];

        if (genomes_[second_contender].getFitness() > genomes_[first_contender].getFitness()) {
            return second_contender;
        }

        return first_contender;
    }

    double InternalPopulation::computeTotalAverageFitness() const {
        double total_fitness = 0.0;

        for (const Species &species : species_list_) {
            total_fitness += species.average_adjusted_fitness;
        }

        return total_fitness;
    }

    double InternalPopulation::findSpeciesFitnessForGenome(std::size_t genome_index) const {
        for (const Species &species : species_list_) {
            for (const std::size_t member_index : species.member_indices) {
                if (member_index == genome_index) {
                    return species.average_adjusted_fitness;
                }
            }
        }

        return 0.0;
    }

    bool InternalPopulation::removeWorstGenome(Genome &removed_genome) {
        if (genomes_.empty()) {
            return false;
        }

        std::size_t worst_index = 0;
        double worst_fitness = genomes_.front().getAdjustedFitness();

        for (std::size_t genome_index = 1; genome_index < genomes_.size(); ++genome_index) {
            if (genomes_[genome_index].getAdjustedFitness() < worst_fitness) {
                worst_fitness = genomes_[genome_index].getAdjustedFitness();
                worst_index = genome_index;
            }
        }

        removed_genome = genomes_[worst_index];
        genomes_.erase(genomes_.begin() + static_cast<std::ptrdiff_t>(worst_index));
        refreshSpeciation();

        return true;
    }

    void InternalPopulation::clearPopulation() {
        genomes_.clear();
        species_list_.clear();
    }

    void InternalPopulation::respecialiseAndShareFitness() { refreshSpeciation(); }

    void InternalPopulation::setNichingEnabled(bool niching_enabled) {
        niching_enabled_ = niching_enabled;
        refreshSpeciation();
    }

    bool InternalPopulation::isNichingEnabled() const { return niching_enabled_; }

    void InternalPopulation::refreshFitnessSharing() {
        applyFitnessSharing(genomes_, species_list_);
        refreshSpeciesFitness(genomes_, species_list_);
    }

    bool InternalPopulation::syncStoredFitness(const Genome &genome, double fitness, int evaluation_count) {
        for (Genome &stored_genome : genomes_) {
            if (stored_genome.isIdenticalTo(genome)) {
                stored_genome.setFitness(fitness);
                stored_genome.setEvaluationCount(evaluation_count);
                refreshFitnessSharing();
                return true;
            }
        }

        return false;
    }

    void InternalPopulation::restoreGenomes(const std::vector<Genome> &genomes) {
        genomes_ = genomes;
        refreshSpeciation();
    }

    void InternalPopulation::refreshSpeciation() {
        if (!niching_enabled_) {
            // "No niching" ablation (paper Sec. 6.2): one shared species, no fitness sharing.
            species_list_.clear();
            if (!genomes_.empty()) {
                Species single{};
                single.species_id = 0;
                single.representative_index = 0;
                for (std::size_t genome_index = 0; genome_index < genomes_.size(); ++genome_index) {
                    single.member_indices.push_back(genome_index);
                    genomes_[genome_index].setAdjustedFitness(genomes_[genome_index].getFitness());
                }

                species_list_.push_back(single);
                refreshSpeciesFitness(genomes_, species_list_);
            }

            return;
        }

        species_list_ = speciatePopulation(genomes_, disjoint_coefficient_, excess_coefficient_, weight_difference_coefficient_, compatibility_threshold_);
        refreshFitnessSharing();
    }

}  // namespace odneat
