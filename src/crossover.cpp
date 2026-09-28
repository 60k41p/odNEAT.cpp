#include "crossover.h"

#include <algorithm>
#include <random>
#include <unordered_map>

#include "genome.h"
#include "innovation_clock.h"

namespace odneat {

    Genome crossoverGenomes(const Genome &fitter_parent,
                            const Genome &weaker_parent,
                            std::mt19937_64 &randomGenerator,
                            double disabled_inheritance_rate,
                            CrossoverOutcome *outcome) {
        const bool equal_fitness = (fitter_parent.getFitness() == weaker_parent.getFitness());
        std::unordered_map<InnovationId, const ConnectionGene *, InnovationIdHash> weaker_by_id{};
        weaker_by_id.reserve(weaker_parent.getConnectionGenes().size() * 2);

        for (const ConnectionGene &connection_gene : weaker_parent.getConnectionGenes()) {
            weaker_by_id.emplace(connection_gene.innovation_id, &connection_gene);
        }

        Genome offspring{};
        CrossoverOutcome local_outcome{};
        std::uniform_real_distribution<double> unitDistribution(0.0, 1.0);

        // Matching-neuron activation coin-flip: disjoint/excess neurons carry intact (existing behavior), but when both parents share a neuron by
        // historical marking the offspring takes either parent's transfer function with equal probability, so CPPN function diversity recombines.
        std::unordered_map<InnovationId, const NeuronGene *, InnovationIdHash> weaker_neurons_by_id{};
        weaker_neurons_by_id.reserve(weaker_parent.getNeuronGenes().size() * 2);
        for (const NeuronGene &neuron_gene : weaker_parent.getNeuronGenes()) {
            weaker_neurons_by_id.emplace(neuron_gene.innovation_id, &neuron_gene);
        }

        for (const NeuronGene &neuron_gene : fitter_parent.getNeuronGenes()) {
            NeuronGene inherited_neuron = neuron_gene;
            const auto weaker_neuron_entry = weaker_neurons_by_id.find(neuron_gene.innovation_id);
            if (weaker_neuron_entry != weaker_neurons_by_id.end() && weaker_neuron_entry->second->activation != neuron_gene.activation &&
                unitDistribution(randomGenerator) < 0.5) {
                inherited_neuron.activation = weaker_neuron_entry->second->activation;
                ++local_outcome.flipped_activations;
            }
            offspring.addNeuronGene(inherited_neuron);
        }

        for (const NeuronGene &neuron_gene : weaker_parent.getNeuronGenes()) {
            if (offspring.findNeuron(neuron_gene.innovation_id) == nullptr) {
                if (equal_fitness) {
                    offspring.addNeuronGene(neuron_gene);
                }
            }
        }

        for (const ConnectionGene &fitter_gene : fitter_parent.getConnectionGenes()) {
            const auto weaker_entry = weaker_by_id.find(fitter_gene.innovation_id);
            const ConnectionGene *matching_weaker = nullptr;

            if (weaker_entry != weaker_by_id.end()) {
                matching_weaker = weaker_entry->second;
            }

            if (matching_weaker != nullptr) {
                ++local_outcome.matching_genes;
                const ConnectionGene &chosen_gene = (unitDistribution(randomGenerator) < 0.5) ? fitter_gene : *matching_weaker;
                ConnectionGene inherited_gene = chosen_gene;

                if (!fitter_gene.enabled || !matching_weaker->enabled) {
                    if (unitDistribution(randomGenerator) < disabled_inheritance_rate) {
                        inherited_gene.enabled = false;
                        ++local_outcome.disabled_genes;
                    } else {
                        inherited_gene.enabled = true;
                    }
                }

                offspring.addConnectionGene(inherited_gene);
            } else {
                ++local_outcome.disjoint_excess_genes;
                // NEAT canon (Stanley & Miikkulainen 2002, p.108): all disjoint/excess genes are
                // always inherited from the fitter parent (both parents on equal fitness),
                // regardless of the enabled flag. Disabled state is handled at matching genes.
                offspring.addConnectionGene(fitter_gene);
                if (!fitter_gene.enabled) {
                    ++local_outcome.disabled_genes;
                }
            }
        }

        if (equal_fitness) {
            for (const ConnectionGene &weaker_gene : weaker_parent.getConnectionGenes()) {
                bool already_present = false;

                for (const ConnectionGene &offspring_gene : offspring.getConnectionGenes()) {
                    if (offspring_gene.innovation_id == weaker_gene.innovation_id) {
                        already_present = true;
                        break;
                    }
                }

                if (!already_present) {
                    ++local_outcome.disjoint_excess_genes;
                    offspring.addConnectionGene(weaker_gene);
                    if (!weaker_gene.enabled) {
                        ++local_outcome.disabled_genes;
                    }
                }
            }

            for (const NeuronGene &weaker_neuron : weaker_parent.getNeuronGenes()) {
                if (offspring.findNeuron(weaker_neuron.innovation_id) == nullptr) {
                    offspring.addNeuronGene(weaker_neuron);
                }
            }
        }

        offspring.sortGenesChronologically();
        // Unevaluated offspring carries no fitness: paper fitness is the mean of sampled
        // virtual energy (Sec. 3.1), so parental averaging would let untested genomes win
        // tournaments and broadcasts before their first evaluation.
        offspring.setFitness(0.0);
        offspring.setEvaluationCount(0);
        offspring.setAdjustedFitness(0.0);

        if (outcome != nullptr) {
            *outcome = local_outcome;
        }

        return offspring;
    }

}  // namespace odneat
