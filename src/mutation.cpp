#include "mutation.h"

#include <algorithm>

#include "config.h"
#include "crossover.h"
#include "genome.h"
#include "innovation_clock.h"

namespace odneat {

    bool addRandomConnection(Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator, double weight_minimum, double weight_maximum) {
        const std::vector<NeuronGene> &neuron_genes = genome.getNeuronGenes();

        if (neuron_genes.size() < 2) {
            return false;
        }

        std::uniform_int_distribution<std::size_t> neuron_picker(0, neuron_genes.size() - 1);
        std::uniform_real_distribution<double> weight_picker(weight_minimum, weight_maximum);

        for (int attempt_index = 0; attempt_index < 20; ++attempt_index) {
            const NeuronGene &input_candidate = neuron_genes[neuron_picker(randomGenerator)];
            const NeuronGene &output_candidate = neuron_genes[neuron_picker(randomGenerator)];

            if (input_candidate.innovation_id == output_candidate.innovation_id) {
                continue;
            }

            // Unrestricted recurrent: any source allowed, but never sink into inputs or bias
            // (those activations are clamped to sensor/1.0, so incoming edges would be dead genes).
            if (output_candidate.neuron_type == NeuronType::kInput || output_candidate.neuron_type == NeuronType::kBias) {
                continue;
            }

            bool connection_exists = false;

            for (const ConnectionGene &existing_gene : genome.getConnectionGenes()) {
                if (existing_gene.input_neuron_id == input_candidate.innovation_id && existing_gene.output_neuron_id == output_candidate.innovation_id) {
                    connection_exists = true;
                    break;
                }
            }

            if (connection_exists) {
                continue;
            }

            ConnectionGene fresh_connection{};
            fresh_connection.innovation_id = innovationClock.nextInnovationId();
            fresh_connection.input_neuron_id = input_candidate.innovation_id;
            fresh_connection.output_neuron_id = output_candidate.innovation_id;
            fresh_connection.weight = weight_picker(randomGenerator);
            fresh_connection.enabled = true;
            genome.addConnectionGene(fresh_connection);
            genome.sortGenesChronologically();

            return true;
        }

        return false;
    }

    bool addRandomNeuron(Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator) {
        std::vector<ConnectionGene> &connection_genes = genome.accessConnectionGenes();
        std::vector<std::size_t> enabled_indices{};
        enabled_indices.reserve(connection_genes.size());

        for (std::size_t connection_index = 0; connection_index < connection_genes.size(); ++connection_index) {
            if (connection_genes[connection_index].enabled) {
                enabled_indices.push_back(connection_index);
            }
        }

        if (enabled_indices.empty()) {
            return false;
        }

        std::uniform_int_distribution<std::size_t> enabledPicker(0, enabled_indices.size() - 1);
        const std::size_t chosen_index = enabled_indices[enabledPicker(randomGenerator)];
        ConnectionGene &split_gene = connection_genes[chosen_index];
        split_gene.enabled = false;
        const double original_weight = split_gene.weight;
        const InnovationId input_id = split_gene.input_neuron_id;
        const InnovationId output_id = split_gene.output_neuron_id;
        NeuronGene hidden_gene{};
        hidden_gene.innovation_id = innovationClock.nextInnovationId();
        hidden_gene.neuron_type = NeuronType::kHidden;
        hidden_gene.input_index = -1;
        hidden_gene.output_index = -1;
        genome.addNeuronGene(hidden_gene);
        ConnectionGene input_edge{};
        input_edge.innovation_id = innovationClock.nextInnovationId();
        input_edge.input_neuron_id = input_id;
        input_edge.output_neuron_id = hidden_gene.innovation_id;
        input_edge.weight = 1.0;
        input_edge.enabled = true;
        ConnectionGene output_edge{};
        output_edge.innovation_id = innovationClock.nextInnovationId();
        output_edge.input_neuron_id = hidden_gene.innovation_id;
        output_edge.output_neuron_id = output_id;
        output_edge.weight = original_weight;
        output_edge.enabled = true;
        genome.addConnectionGene(input_edge);
        genome.addConnectionGene(output_edge);
        genome.sortGenesChronologically();

        return true;
    }

    void mutateGenome(
        Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator, const OdneatParams &parameters, MutationOutcome *outcome) {
        MutationOutcome local_outcome{};
        std::uniform_real_distribution<double> unitDistribution(0.0, 1.0);
        std::normal_distribution<double> weightNoise(0.0, parameters.weight_mutation_magnitude);

        for (ConnectionGene &connection_gene : genome.accessConnectionGenes()) {
            if (unitDistribution(randomGenerator) < parameters.weight_perturb_rate) {
                connection_gene.weight += weightNoise(randomGenerator);
                if (connection_gene.weight < parameters.weight_minimum) {
                    connection_gene.weight = parameters.weight_minimum;
                }

                if (connection_gene.weight > parameters.weight_maximum) {
                    connection_gene.weight = parameters.weight_maximum;
                }

                ++local_outcome.perturbed_weights;
            }

            if (unitDistribution(randomGenerator) < parameters.toggle_enabled_rate) {
                connection_gene.enabled = !connection_gene.enabled;
                local_outcome.toggled_enabled = true;
            }
        }

        // Stanley bias-neuron model: no per-neuron bias to perturb; bias evolves via bias->X weights.
        // perturbed_biases retained in MutationOutcome for API compat (always 0).
        (void)genome;

        if (unitDistribution(randomGenerator) < parameters.add_connection_rate) {
            local_outcome.added_connection =
                addRandomConnection(genome, innovationClock, randomGenerator, parameters.weight_minimum, parameters.weight_maximum);
        }

        if (unitDistribution(randomGenerator) < parameters.add_neuron_rate) {
            local_outcome.added_neuron = addRandomNeuron(genome, innovationClock, randomGenerator);
        }

        if (outcome != nullptr) {
            *outcome = local_outcome;
        }
    }

    Genome generateOffspring(const Genome &first_parent,
                             const Genome *second_parent,
                             InnovationClock &innovationClock,
                             std::mt19937_64 &randomGenerator,
                             const OdneatParams &parameters) {
        std::uniform_real_distribution<double> unitDistribution(0.0, 1.0);
        Genome offspring{};

        if (second_parent != nullptr && unitDistribution(randomGenerator) < parameters.crossover_rate) {
            if (second_parent->getFitness() > first_parent.getFitness()) {
                offspring = crossoverGenomes(*second_parent, first_parent, randomGenerator, parameters.disabled_inheritance_rate, nullptr);
            } else {
                offspring = crossoverGenomes(first_parent, *second_parent, randomGenerator, parameters.disabled_inheritance_rate, nullptr);
            }
        } else {
            offspring = first_parent;
            // Cloned-then-mutated offspring is unevaluated: reset fitness rather than
            // inheriting the parent's sampled mean (see crossoverGenomes).
            offspring.setFitness(0.0);
            offspring.setEvaluationCount(0);
            offspring.setAdjustedFitness(0.0);
        }

        mutateGenome(offspring, innovationClock, randomGenerator, parameters, nullptr);

        return offspring;
    }

}  // namespace odneat
