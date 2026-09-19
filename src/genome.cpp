#include "genome.h"

#include <algorithm>
#include <random>

namespace odneat {

    Genome::Genome() = default;

    Genome Genome::createMinimalGenome(int input_count, int output_count, InnovationClock &innovationClock) {
        // Deterministic minimal topology shared by all robots so functionally identical
        // minimals have compatibility distance 0 (Stanley matching guarantee).
        // Uses reserved robot 0 with small timestamps; evolved genes use system time (~1.7e18 ns).
        // Includes one kBias neuron (Stanley canon) fully connected to outputs.
        // Weights are deterministic uniform(-1,1) to break symmetry identically across robots.
        (void)innovationClock;
        constexpr std::uint32_t kMinimalRobot = 0;
        constexpr std::uint64_t kMinimalConnBase = 1000000ULL;
        Genome minimal_genome{};
        std::uint64_t neuron_seq = 0;
        for (int input_index = 0; input_index < input_count; ++input_index) {
            NeuronGene input_gene{};
            input_gene.innovation_id = InnovationId{kMinimalRobot, neuron_seq, static_cast<std::uint32_t>(neuron_seq)};
            input_gene.neuron_type = NeuronType::kInput;
            input_gene.input_index = input_index;
            input_gene.output_index = -1;
            minimal_genome.addNeuronGene(input_gene);
            ++neuron_seq;
        }

        NeuronGene bias_gene{};
        bias_gene.innovation_id = InnovationId{kMinimalRobot, neuron_seq, static_cast<std::uint32_t>(neuron_seq)};
        bias_gene.neuron_type = NeuronType::kBias;
        bias_gene.input_index = -1;
        bias_gene.output_index = -1;
        minimal_genome.addNeuronGene(bias_gene);
        ++neuron_seq;

        for (int output_index = 0; output_index < output_count; ++output_index) {
            NeuronGene output_gene{};
            output_gene.innovation_id = InnovationId{kMinimalRobot, neuron_seq, static_cast<std::uint32_t>(neuron_seq)};
            output_gene.neuron_type = NeuronType::kOutput;
            output_gene.input_index = -1;
            output_gene.output_index = output_index;
            minimal_genome.addNeuronGene(output_gene);
            ++neuron_seq;
        }

        const std::vector<NeuronGene> input_snapshot = minimal_genome.getNeuronGenes();
        std::vector<NeuronGene> input_genes{};
        std::vector<NeuronGene> output_genes{};

        for (const NeuronGene &neuron_gene : input_snapshot) {
            if (neuron_gene.neuron_type == NeuronType::kInput || neuron_gene.neuron_type == NeuronType::kBias) {
                input_genes.push_back(neuron_gene);
            } else if (neuron_gene.neuron_type == NeuronType::kOutput) {
                output_genes.push_back(neuron_gene);
            }
        }

        // Deterministic RNG so all robots share identical initial weights (distance 0) with symmetry broken across edges.
        std::mt19937_64 weight_rng(
            static_cast<std::uint64_t>(0xC0FFEEULL + static_cast<std::uint64_t>(input_count) * 131ULL + static_cast<std::uint64_t>(output_count) * 17ULL));
        std::uniform_real_distribution<double> weight_dist(-1.0, 1.0);
        std::uint64_t conn_seq = 0;
        for (const NeuronGene &input_gene : input_genes) {
            for (const NeuronGene &output_gene : output_genes) {
                ConnectionGene full_connection{};
                const std::uint64_t stamp = kMinimalConnBase + conn_seq;
                full_connection.innovation_id = InnovationId{kMinimalRobot, stamp, static_cast<std::uint32_t>(stamp)};
                full_connection.input_neuron_id = input_gene.innovation_id;
                full_connection.output_neuron_id = output_gene.innovation_id;
                full_connection.weight = weight_dist(weight_rng);
                full_connection.enabled = true;
                minimal_genome.addConnectionGene(full_connection);
                ++conn_seq;
            }
        }

        minimal_genome.sortGenesChronologically();

        return minimal_genome;
    }

    const std::vector<NeuronGene> &Genome::getNeuronGenes() const { return neuron_genes_; }

    const std::vector<ConnectionGene> &Genome::getConnectionGenes() const { return connection_genes_; }

    std::vector<NeuronGene> &Genome::accessNeuronGenes() { return neuron_genes_; }

    std::vector<ConnectionGene> &Genome::accessConnectionGenes() { return connection_genes_; }

    void Genome::addNeuronGene(const NeuronGene &neuron_gene) { neuron_genes_.push_back(neuron_gene); }

    void Genome::addConnectionGene(const ConnectionGene &connection_gene) { connection_genes_.push_back(connection_gene); }

    void Genome::sortGenesChronologically() {
        std::sort(connection_genes_.begin(), connection_genes_.end(),
                  [](const ConnectionGene &left, const ConnectionGene &right) { return left.innovation_id < right.innovation_id; });
    }

    const NeuronGene *Genome::findNeuron(const InnovationId &neuron_id) const {
        for (const NeuronGene &neuron_gene : neuron_genes_) {
            if (neuron_gene.innovation_id == neuron_id) {
                return &neuron_gene;
            }
        }

        return nullptr;
    }

    NeuronGene *Genome::findNeuronMutable(const InnovationId &neuron_id) {
        for (NeuronGene &neuron_gene : neuron_genes_) {
            if (neuron_gene.innovation_id == neuron_id) {
                return &neuron_gene;
            }
        }

        return nullptr;
    }

    std::size_t Genome::countEnabledConnections() const {
        std::size_t enabled_count = 0;

        for (const ConnectionGene &connection_gene : connection_genes_) {
            if (connection_gene.enabled) {
                ++enabled_count;
            }
        }

        return enabled_count;
    }

    std::size_t Genome::countHiddenNeurons() const {
        std::size_t hidden_count = 0;

        for (const NeuronGene &neuron_gene : neuron_genes_) {
            if (neuron_gene.neuron_type == NeuronType::kHidden) {
                ++hidden_count;
            }
        }

        return hidden_count;
    }

    std::size_t Genome::computeComplexity() const { return connection_genes_.size() + neuron_genes_.size(); }

    bool Genome::isIdenticalTo(const Genome &other) const {
        if (neuron_genes_.size() != other.neuron_genes_.size() || connection_genes_.size() != other.connection_genes_.size()) {
            return false;
        }

        for (std::size_t neuron_index = 0; neuron_index < neuron_genes_.size(); ++neuron_index) {
            if (!(neuron_genes_[neuron_index] == other.neuron_genes_[neuron_index])) {
                return false;
            }
        }

        for (std::size_t connection_index = 0; connection_index < connection_genes_.size(); ++connection_index) {
            if (!(connection_genes_[connection_index] == other.connection_genes_[connection_index])) {
                return false;
            }
        }

        return true;
    }

    double Genome::getFitness() const { return fitness_; }

    void Genome::setFitness(double fitness_score) { fitness_ = fitness_score; }

    int Genome::getEvaluationCount() const { return evaluation_count_; }

    void Genome::setEvaluationCount(int evaluation_count) { evaluation_count_ = evaluation_count; }

    double Genome::getAdjustedFitness() const { return adjusted_fitness_; }

    void Genome::setAdjustedFitness(double adjusted_fitness) { adjusted_fitness_ = adjusted_fitness; }

}  // namespace odneat
