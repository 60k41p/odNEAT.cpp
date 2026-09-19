/* Discrete-time recurrent neural network phenotype decoded from a Genome (Stanley bias-neuron model).
 *
 * This header evaluates evolved controllers: inputs copy normalised sensors in [0, 1], bias holds constant 1.0,
 * hidden/output apply logistic to weighted sums (bias via bias-neuron edges), all update synchronously so recurrent
 * loops use previous-step activations. Outputs in [0, 1] are linearly rescaled to [-1,
 * 1] wheel speeds by the caller. Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Section 4.1
 * and Tables 3-5.
 */

#ifndef ODNEAT_NETWORK_H_
#define ODNEAT_NETWORK_H_

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "genome.h"
#include "innovation_clock.h"

namespace odneat {

    // LogisticActivation evaluates 1 / (1 + exp(-slope * (sum + bias))).
    // Bias param is retained for API compat; phenotypes use bias-neuron edges so callers pass 0.
    double computeLogisticActivation(double weighted_sum, double bias, double slope);

    // RecurrentNetwork is the executable phenotype of a Genome with persistent activations across steps.
    class RecurrentNetwork {
       public:
        // Builds a network from a genome; input and output counts must match the controller layout.
        static RecurrentNetwork decodeFromGenome(const Genome &genome, int input_count, int output_count, double logistic_slope = 1.0);
        // Advances the network one control cycle and returns output_count values in [0, 1].
        std::vector<double> stepNetwork(const std::vector<double> &input_values);
        // Resets hidden/output to zero and bias to 1.0 (called when a new controller takes over).
        void resetActivations();
        // Returns the number of inputs this network expects.
        int getInputCount() const;
        // Returns the number of outputs this network produces.
        int getOutputCount() const;
        // Returns the current activation of every tracked neuron for debugging.
        const std::unordered_map<InnovationId, double, InnovationIdHash> &getActivations() const;

       private:
        // CompactSnappedConnection is a cache-friendly edge with integer neuron indices.
        struct SnappedConnection {
            // Index into ordered neuron storage for the source.
            std::size_t input_index = 0;
            // Index into ordered neuron storage for the target.
            std::size_t output_index = 0;
            // Synaptic weight.
            double weight = 0.0;
            // Whether this edge participates in evaluation.
            bool enabled = true;
        };
        // CompactNeuron stores type flags and runtime activation (Stanley bias-neuron model).
        struct CompactNeuron {
            // Full innovation identity for stable lookups (no truncation).
            InnovationId neuron_id{};
            // True for sensor-driven input neurons.
            bool is_input = false;
            // True for constant-1.0 bias neurons.
            bool is_bias = false;
            // Sensor slot copied into this neuron when is_input is true.
            int input_index = -1;
            // True for motor output neurons read after the synchronous update.
            bool is_output = false;
            // Motor slot written by this neuron when is_output is true.
            int output_index = -1;
            // Persistent activation carried across steps for recurrence.
            double activation = 0.0;
        };
        // Ordered neurons for cache-friendly synchronous updates.
        std::vector<CompactNeuron> neurons_;
        // Only enabled edges are decoded and fired.
        std::vector<SnappedConnection> connections_;
        // Positions of output neurons in neurons_ order, indexed by motor slot (SIZE_MAX when missing).
        std::vector<std::size_t> output_positions_;
        // Logistic slope applied to hidden and output neurons.
        double logistic_slope_ = 1.0;
        // Expected input dimensionality.
        int input_count_ = 0;
        // Expected output dimensionality.
        int output_count_ = 0;
        // Live activations keyed by full InnovationId for external inspection.
        std::unordered_map<InnovationId, double, InnovationIdHash> activation_map_{};
    };

    // Converts raw network outputs in [0, 1] to signed wheel speeds in [-1, 1].
    double rescaleOutputToWheelSpeed(double network_output);
    // Clamps a sensor reading into the network input range [0, 1].
    double clampSensorInput(double sensor_reading);

}  // namespace odneat

#endif  // ODNEAT_NETWORK_H_
