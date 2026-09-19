/* Unilateral genome broadcast policy for inter-robot reproduction.
 *
 * This header computes the per-control-cycle broadcast probability P(event) = mean adjusted fitness of the active genome's species divided by the sum over
 * local species. The rule promotes propagation of competitive topological innovations without handshakes. Citations: Silva et al. (2015) "odNEAT: An Algorithm
 * for Decentralised Online Evolution of Robotic Controllers", Section 3.2, Equation 2 and Algorithm 1.
 */

#ifndef ODNEAT_BROADCAST_H_
#define ODNEAT_BROADCAST_H_

#include <random>

namespace odneat {

    // BroadcastPolicy evaluates whether the active genome should be sent to neighbours this cycle.
    class BroadcastPolicy {
       public:
        // Constructs a policy with no state; probabilities derive from population fitness each cycle.
        BroadcastPolicy();
        // Computes P(event) = species_fitness / total_fitness, clamped to [0, 1]; zero when total is non-positive.
        double computeBroadcastProbability(double active_species_fitness, double total_species_fitness) const;
        // Samples a Bernoulli trial with the Equation 2 probability.
        bool shouldBroadcast(double active_species_fitness, double total_species_fitness, std::mt19937_64 &randomGenerator) const;
    };

}  // namespace odneat

#endif  // ODNEAT_BROADCAST_H_
