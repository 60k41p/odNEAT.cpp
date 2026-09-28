#include "network.h"

#include <cmath>
#include <limits>

#include "config.h"
#include "genome.h"

namespace odneat {

    double computeLogisticActivation(double weighted_sum, double bias, double slope) { return 1.0 / (1.0 + std::exp(-slope * (weighted_sum + bias))); }

    double computeActivation(ActivationFn activation_function, double weighted_sum, double slope) {
        // Periodic and scalar CPPN forms: square is the sign(sin(2πz)) wave (not z²), sawtooth/triangle are the arctan/arcsin closed forms, smooth_* are
        // the tanh-shaped variants. exp/log/inv/lelu are guarded to stay finite on arbitrary sums and keep recurrent phenotypes bounded: exp clamps its
        // argument, log/inv clamp near zero, lelu uses slope 0.1.
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kExpArgLimit = 15.0;
        constexpr double kNearZeroGuard = 1e-7;
        constexpr double kLeluSlope = 0.1;
        switch (activation_function) {
            case ActivationFn::kIdentity:
                return weighted_sum;
            case ActivationFn::kTanh:
                return std::tanh(weighted_sum);
            case ActivationFn::kSin:
                return std::sin(weighted_sum);
            case ActivationFn::kGaussian:
                return std::exp(-weighted_sum * weighted_sum);
            case ActivationFn::kAbs:
                return std::fabs(weighted_sum);
            case ActivationFn::kRelu:
                return weighted_sum > 0.0 ? weighted_sum : 0.0;
            case ActivationFn::kLelu:
                return weighted_sum >= 0.0 ? weighted_sum : kLeluSlope * weighted_sum;
            case ActivationFn::kSquare: {
                const double wave = std::sin(2.0 * kPi * weighted_sum);
                return wave > 0.0 ? 1.0 : (wave < 0.0 ? -1.0 : 0.0);
            }
            case ActivationFn::kSawtooth:
                // IEEE division carries the pole: 1/tan(πz) is ±inf at integers, atan maps it to ±π/2 for an exact closed-form sawtooth.
                return 2.0 / kPi * std::atan(1.0 / std::tan(kPi * weighted_sum));
            case ActivationFn::kTriangle: {
                const double wave = std::sin(2.0 * kPi * weighted_sum);
                const double clamped = std::min(1.0, std::max(-1.0, wave));
                return 2.0 / kPi * std::asin(clamped);
            }
            case ActivationFn::kSmoothSquare:
                return std::tanh(5.0 * std::sin(2.0 * weighted_sum));
            case ActivationFn::kSmoothSawtooth: {
                const double product = std::tanh(5.0 * std::sin(weighted_sum)) * std::cos(weighted_sum);
                const double clamped = std::min(1.0, std::max(-1.0, product));
                return std::asin(clamped);
            }
            case ActivationFn::kExp:
                return std::exp(std::min(kExpArgLimit, std::max(-kExpArgLimit, weighted_sum)));
            case ActivationFn::kLog:
                return std::log(std::max(kNearZeroGuard, weighted_sum));
            case ActivationFn::kInv:
                return std::fabs(weighted_sum) > kNearZeroGuard ? 1.0 / weighted_sum : (weighted_sum >= 0.0 ? 1.0 / kNearZeroGuard : -1.0 / kNearZeroGuard);
            case ActivationFn::kLogistic:
            default:
                return computeLogisticActivation(weighted_sum, 0.0, slope);
        }
    }

    RecurrentNetwork RecurrentNetwork::decodeFromGenome(const Genome &genome, int input_count, int output_count, double logistic_slope) {
        RecurrentNetwork decoded_network{};
        decoded_network.input_count_ = input_count;
        decoded_network.output_count_ = output_count;
        decoded_network.logistic_slope_ = logistic_slope;
        decoded_network.neurons_.reserve(genome.getNeuronGenes().size());
        decoded_network.output_positions_ = std::vector<std::size_t>(static_cast<std::size_t>(output_count), std::numeric_limits<std::size_t>::max());
        std::unordered_map<InnovationId, std::size_t, InnovationIdHash> position_by_id{};
        position_by_id.reserve(genome.getNeuronGenes().size() * 2);

        for (const NeuronGene &neuron_gene : genome.getNeuronGenes()) {
            CompactNeuron compact_neuron{};
            compact_neuron.neuron_id = neuron_gene.innovation_id;
            compact_neuron.is_input = (neuron_gene.neuron_type == NeuronType::kInput);
            compact_neuron.is_bias = (neuron_gene.neuron_type == NeuronType::kBias);
            compact_neuron.input_index = neuron_gene.input_index;
            compact_neuron.activation_function = neuron_gene.activation;
            compact_neuron.is_output = (neuron_gene.neuron_type == NeuronType::kOutput);
            compact_neuron.output_index = neuron_gene.output_index;
            compact_neuron.activation = compact_neuron.is_bias ? 1.0 : 0.0;
            const std::size_t position = decoded_network.neurons_.size();
            position_by_id.emplace(compact_neuron.neuron_id, position);
            decoded_network.neurons_.push_back(compact_neuron);
            if (compact_neuron.is_output && compact_neuron.output_index >= 0 && compact_neuron.output_index < output_count) {
                decoded_network.output_positions_[static_cast<std::size_t>(compact_neuron.output_index)] = position;
            }

            decoded_network.activation_map_.emplace(compact_neuron.neuron_id, compact_neuron.activation);
        }

        decoded_network.connections_.reserve(genome.getConnectionGenes().size());

        for (const ConnectionGene &connection_gene : genome.getConnectionGenes()) {
            if (!connection_gene.enabled) {
                continue;
            }

            const auto input_entry = position_by_id.find(connection_gene.input_neuron_id);
            const auto output_entry = position_by_id.find(connection_gene.output_neuron_id);

            if (input_entry == position_by_id.end() || output_entry == position_by_id.end()) {
                continue;
            }

            SnappedConnection snapped_connection{};
            snapped_connection.input_index = input_entry->second;
            snapped_connection.output_index = output_entry->second;
            snapped_connection.weight = connection_gene.weight;
            snapped_connection.enabled = true;
            decoded_network.connections_.push_back(snapped_connection);
        }

        return decoded_network;
    }

    std::vector<double> RecurrentNetwork::stepNetwork(const std::vector<double> &input_values) {
        for (std::size_t neuron_index = 0; neuron_index < neurons_.size(); ++neuron_index) {
            CompactNeuron &neuron = neurons_[neuron_index];
            if (neuron.is_input && neuron.input_index >= 0 && static_cast<std::size_t>(neuron.input_index) < input_values.size()) {
                neuron.activation = clampSensorInput(input_values[static_cast<std::size_t>(neuron.input_index)]);
                activation_map_[neuron.neuron_id] = neuron.activation;
            } else if (neuron.is_bias) {
                neuron.activation = 1.0;
                activation_map_[neuron.neuron_id] = 1.0;
            }
        }

        std::vector<double> nextActivations(neurons_.size(), 0.0);

        for (std::size_t neuron_index = 0; neuron_index < neurons_.size(); ++neuron_index) {
            nextActivations[neuron_index] = neurons_[neuron_index].activation;
        }

        std::vector<double> weightedSums(neurons_.size(), 0.0);

        for (const SnappedConnection &connection : connections_) {
            weightedSums[connection.output_index] += neurons_[connection.input_index].activation * connection.weight;
        }

        for (std::size_t neuron_index = 0; neuron_index < neurons_.size(); ++neuron_index) {
            CompactNeuron &neuron = neurons_[neuron_index];
            if (neuron.is_input || neuron.is_bias) {
                continue;
            }

            nextActivations[neuron_index] = computeActivation(neuron.activation_function, weightedSums[neuron_index], logistic_slope_);
        }

        for (std::size_t neuron_index = 0; neuron_index < neurons_.size(); ++neuron_index) {
            neurons_[neuron_index].activation = nextActivations[neuron_index];
            activation_map_[neurons_[neuron_index].neuron_id] = nextActivations[neuron_index];
        }

        std::vector<double> outputValues(static_cast<std::size_t>(output_count_), 0.5);

        for (int output_index = 0; output_index < output_count_; ++output_index) {
            const std::size_t position = output_positions_[static_cast<std::size_t>(output_index)];
            if (position != std::numeric_limits<std::size_t>::max() && position < neurons_.size()) {
                outputValues[static_cast<std::size_t>(output_index)] = neurons_[position].activation;
            }
        }

        return outputValues;
    }

    void RecurrentNetwork::resetActivations() {
        for (CompactNeuron &neuron : neurons_) {
            if (neuron.is_input) {
                continue;
            }

            if (neuron.is_bias) {
                neuron.activation = 1.0;
                activation_map_[neuron.neuron_id] = 1.0;
                continue;
            }

            neuron.activation = 0.0;
            activation_map_[neuron.neuron_id] = 0.0;
        }
    }

    int RecurrentNetwork::getInputCount() const { return input_count_; }

    int RecurrentNetwork::getOutputCount() const { return output_count_; }

    const std::unordered_map<InnovationId, double, InnovationIdHash> &RecurrentNetwork::getActivations() const { return activation_map_; }

    double rescaleOutputToWheelSpeed(double network_output) {
        const double clamped_output = std::min(1.0, std::max(0.0, network_output));
        return clamped_output * 2.0 - 1.0;
    }

    double clampSensorInput(double sensor_reading) {
        return std::min(DefaultConstants::kInputMaximum, std::max(DefaultConstants::kInputMinimum, sensor_reading));
    }

}  // namespace odneat
