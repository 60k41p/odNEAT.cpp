#ifndef ODNEAT_CONFIG_H_
#define ODNEAT_CONFIG_H_

/* Configuration parameters for the odNEAT algorithm with paper defaults.
 *
 * Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Section 4.1 (Experimental Setup, Table 2)
 * and Section 4.2 (Preliminary Performance Tuning); Stanley and Miikkulainen (2002) "Evolving Neural Networks through Augmenting Topologies", Section on
 * speciation coefficients.
 */

namespace odneat {

    // DefaultConstants centralises magic numbers shared across headers to keep the implementation DRY.
    struct DefaultConstants {
        // Genome weight bounds from Table 2: weights in [-10, 10].
        static constexpr double kWeightMinimum = -10.0;
        static constexpr double kWeightMaximum = 10.0;
        // Neural input range [0, 1] and output rescaling [0, 1] -> [-1, 1] (Section 4.1).
        static constexpr double kInputMinimum = 0.0;
        static constexpr double kInputMaximum = 1.0;
        // Logistic activation function (Table 2).
        static constexpr double logistic_slope = 1.0;
        // Compatibility threshold delta = 3.0 (Section 4.2).
        static constexpr double kCompatibilityThreshold = 3.0;
        // NEAT compatibility coefficients pinned per user request to Stanley defaults.
        static constexpr double kCompatibilityDisjointCoefficient = 1.0;
        static constexpr double kCompatibilityExcessCoefficient = 1.0;
        static constexpr double kCompatibilityWeightCoefficient = 0.4;
        // Normalisation size below which N is set to 1 (Stanley 2002, Section 3.3).
        static constexpr int kCompatibilitySmallGenomeSize = 20;
    };

    // OdneatParams aggregates all tunable rates for the evolutionary loop.
    // Citation: Silva et al. (2015), Section 4.1: crossover 0.25, mutation 0.1, add neuron 0.03, add connection 0.05, weight magnitude 0.5.
    struct OdneatParams {
        // Probability of recombining two parents versus cloning a single parent when generating offspring.
        double crossover_rate = 0.25;
        // Per-connection probability of perturbing the weight with Gaussian noise (bias via bias-neuron edges).
        double weight_perturb_rate = 0.10;
        // Standard deviation of the Gaussian weight perturbation.
        double weight_mutation_magnitude = 0.50;
        // Per-offspring probability of adding one new connection gene.
        double add_connection_rate = 0.05;
        // Per-offspring probability of adding one new neuron gene by splitting a connection.
        double add_neuron_rate = 0.03;
        // Probability of toggling a connection enabled flag during mutation. Not in either paper
        // (NEAT propagates disabled state via crossover only); defaults to off, opt in for experiments.
        double toggle_enabled_rate = 0.0;
        // Probability that an offspring inherits the disabled state when parents disagree (NEAT default 0.75).
        double disabled_inheritance_rate = 0.75;
        // Compatibility distance threshold for speciation (delta).
        double compatibility_threshold = DefaultConstants::kCompatibilityThreshold;
        // Compatibility coefficients c1 (excess), c2 (disjoint), c3 (weight).
        double disjoint_coefficient = DefaultConstants::kCompatibilityDisjointCoefficient;
        double excess_coefficient = DefaultConstants::kCompatibilityExcessCoefficient;
        double weight_difference_coefficient = DefaultConstants::kCompatibilityWeightCoefficient;
        // Maximum number of genomes stored in each robot's internal population (Table 2: 40).
        int internal_population_size = 40;
        // Minimum control cycles a new controller is protected from replacement (Section 4.1: 500 cycles / 50 s).
        int maturation_period_cycles = 500;
        // Number of most recently received genomes used to expire tabu entries (Section 4.1: last 50).
        int tabu_expiry_history_size = 50;
        // Control cycle duration in seconds (Table 2: 100 ms).
        double control_cycle_seconds = 0.10;
        // Weight clipping bounds.
        double weight_minimum = DefaultConstants::kWeightMinimum;
        double weight_maximum = DefaultConstants::kWeightMaximum;
    };

    // SimulationParams captures the shared robot and arena constants.
    // Citation: Silva et al. (2015), Section 4.1 and Table 2.
    struct SimulationParams {
        // Number of robots per group (Table 2: 5).
        int group_size = 5;
        // Genome broadcast / sensing range in metres (Table 2: 25 cm).
        double broadcast_range_metres = 0.25;
        // Arena edge length in metres (Table 2: 3 x 3 m).
        double arena_size_metres = 3.0;
        // e-puck diameter in metres (Section 4.1: 7.5 cm).
        double robot_diameter_metres = 0.075;
        // Effective wheel base in metres (implementation assumption; e-puck axle is narrower than body).
        double wheel_base_metres = 0.053;
        // Maximum linear speed per wheel in metres per second (Section 4.1: 13 cm/s).
        double maximum_wheel_speed = 0.13;
        // Gaussian sensor/actuator noise as fraction of saturation (Section 4.1: +/-5%).
        double noise_standard_deviation = 0.05;
        // Number of evenly spaced infrared sensors per modality (Tables 3-5: 8).
        int infrared_sensor_count = 8;
        // Light sensor range in metres (Section 4.5: 50 cm).
        double light_sensor_range_metres = 0.50;
        // Number of control cycles in the genome-count window for aggregation (Section 4.3: P = 10).
        int aggregation_genome_window = 10;
    };

}  // namespace odneat

#endif  // ODNEAT_CONFIG_H_
