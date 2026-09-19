#include "broadcast.h"

#include <algorithm>

namespace odneat {

    BroadcastPolicy::BroadcastPolicy() = default;

    double BroadcastPolicy::computeBroadcastProbability(double active_species_fitness, double total_species_fitness) const {
        if (total_species_fitness <= 0.0) {
            return 0.0;
        }

        const double raw_probability = active_species_fitness / total_species_fitness;

        return std::clamp(raw_probability, 0.0, 1.0);
    }

    bool BroadcastPolicy::shouldBroadcast(double active_species_fitness, double total_species_fitness, std::mt19937_64 &randomGenerator) const {
        const double broadcast_probability = computeBroadcastProbability(active_species_fitness, total_species_fitness);

        if (broadcast_probability <= 0.0) {
            return false;
        }

        if (broadcast_probability >= 1.0) {
            return true;
        }

        std::bernoulli_distribution broadcastTrial(broadcast_probability);

        return broadcastTrial(randomGenerator);
    }

}  // namespace odneat
