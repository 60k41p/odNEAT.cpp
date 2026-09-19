/* Genome representation for odNEAT: neuron genes, connection genes and whole-genome helpers.
 *
 * This header defines the flexible genetic encoding shared by odNEAT and NEAT: a list of neuron genes plus a list of connection genes carrying historical
 * markings. Matching genes align by InnovationId during crossover and speciation, while disjoint and excess genes are classified by chronological range.
 * Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 2.2, 3 and 4.1; Stanley and
 * Miikkulainen (2002) "Evolving Neural Networks through Augmenting Topologies", Sections 3.1-3.2 on genetic encoding.
 */

#ifndef ODNEAT_GENOME_H_
#define ODNEAT_GENOME_H_

#include <compare>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "innovation_clock.h"

namespace odneat {

    // NeuronType distinguishes sensor inputs, evolvable hidden units and motor outputs.
    enum class NeuronType : std::uint8_t { kInput = 0, kHidden = 1, kOutput = 2, kBias = 3 };

    // NeuronGene describes one node in the decoded neural network.
    // Citation: Stanley and Miikkulainen (2002), Section 3.1: node genes with type and historical marking.
    // Bias is a dedicated kBias neuron with constant activation 1.0 (Stanley canon), not a per-neuron field.
    struct NeuronGene {
        // Historical marking assigning chronology; equal values mean the same structural feature.
        InnovationId innovation_id{};
        // Functional role of the neuron within the controller.
        NeuronType neuron_type = NeuronType::kHidden;
        // Index of the input sensor driving this neuron when it is an input, -1 otherwise.
        int input_index = -1;
        // Index of the motor output driven by this neuron when it is an output, -1 otherwise.
        int output_index = -1;

        bool operator==(const NeuronGene &other) const = default;
    };

    // ConnectionGene describes one directed weighted edge between two neurons.
    // Citation: Stanley and Miikkulainen (2002), Section 3.1: in-node, out-node, weight, enabled bit and innovation number.
    struct ConnectionGene {
        // Historical marking assigning chronology; equal values mean the same structural feature.
        InnovationId innovation_id{};
        // Innovation identifier of the source neuron gene.
        InnovationId input_neuron_id{};
        // Innovation identifier of the target neuron gene.
        InnovationId output_neuron_id{};
        // Synaptic weight in [kWeightMinimum, kWeightMaximum].
        double weight = 0.0;
        // Whether the connection is expressed in the phenotype; disabled genes remain for crossover alignment.
        bool enabled = true;

        bool operator==(const ConnectionGene &other) const = default;
    };

    // Genome is a complete evolvable neural controller genotype plus its evaluated quality.
    // Citation: Silva et al. (2015), Sections 3.1-3.3 on fitness versus energy, internal populations and offspring generation.
    class Genome {
       public:
        // Constructs an empty genome with zero fitness and no evaluations.
        Genome();
        // Constructs a minimal fully connected input-to-output genome with fresh hidden-free topology.
        static Genome createMinimalGenome(int input_count, int output_count, InnovationClock &innovationClock);
        // Returns the neuron genes in arbitrary but stable order.
        const std::vector<NeuronGene> &getNeuronGenes() const;
        // Returns the connection genes in chronological order of innovation identifiers.
        const std::vector<ConnectionGene> &getConnectionGenes() const;
        // Returns the mutable neuron genes for mutation operators.
        std::vector<NeuronGene> &accessNeuronGenes();
        // Returns the mutable connection genes for mutation operators.
        std::vector<ConnectionGene> &accessConnectionGenes();
        // Adds a neuron gene and keeps internal indexing consistent.
        void addNeuronGene(const NeuronGene &neuron_gene);
        // Adds a connection gene to the genome.
        void addConnectionGene(const ConnectionGene &connection_gene);
        // Sorts connection genes chronologically so disjoint versus excess classification is well defined.
        void sortGenesChronologically();
        // Finds a neuron gene by innovation identifier, or nullptr when absent.
        const NeuronGene *findNeuron(const InnovationId &neuron_id) const;
        // Finds a mutable neuron gene by innovation identifier, or nullptr when absent.
        NeuronGene *findNeuronMutable(const InnovationId &neuron_id);
        // Counts enabled connection genes.
        std::size_t countEnabledConnections() const;
        // Counts hidden neuron genes added through evolution.
        std::size_t countHiddenNeurons() const;
        // Returns the effective parameter count C_fp = connections + neurons used in Tables 7 and 10.
        std::size_t computeComplexity() const;
        // Returns true when both genomes carry exactly the same innovation identifiers and weights.
        bool isIdenticalTo(const Genome &other) const;
        // Returns the averaged fitness score (mean sampled energy), distinct from instantaneous energy (Section 3.1).
        double getFitness() const;
        // Overwrites the averaged fitness score.
        void setFitness(double fitness_score);
        // Returns the number of energy samples averaged into the fitness score.
        int getEvaluationCount() const;
        // Sets the number of energy samples averaged into the fitness score.
        void setEvaluationCount(int evaluation_count);
        // Returns the fitness shared within the species (fitness divided by species size).
        double getAdjustedFitness() const;
        // Sets the fitness shared within the species.
        void setAdjustedFitness(double adjusted_fitness);

       private:
        // Neuron genes owned by this genome.
        std::vector<NeuronGene> neuron_genes_;
        // Connection genes owned by this genome.
        std::vector<ConnectionGene> connection_genes_;
        // Averaged virtual energy level sampled during evaluation; the evolutionary fitness.
        double fitness_ = 0.0;
        // Number of samples contributing to fitness_; used for incremental averaging on duplicate receipt.
        int evaluation_count_ = 0;
        // Fitness divided by species size after fitness sharing.
        double adjusted_fitness_ = 0.0;
    };

}  // namespace odneat

#endif  // ODNEAT_GENOME_H_
