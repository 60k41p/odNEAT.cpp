/* Task-specific virtual energy update rules for the three experimental tasks.
 *
 * This header implements the per-control-cycle energy deltas rewarding aggregation, integrated navigation with obstacle avoidance, and phototaxis. Each functor
 * is stateless and operates on already-sensed quantities so both the simulator and unit tests share one implementation. Citations: Silva et al. (2015) "odNEAT:
 * An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 4.3 (Equations 3-4, Table 3), 4.4 (Equation 5, Table 4) and 4.5 (Equation
 * 6, Table 5).
 */

#ifndef ODNEAT_TASKS_TASK_ENERGIES_H_
#define ODNEAT_TASKS_TASK_ENERGIES_H_

namespace odneat {

    // AggregationEnergyUpdater computes ΔE = alpha + gamma for grouping behaviour.
    // Citation: Silva et al. (2015), Section 4.3, Equations 3 and 4.
    class AggregationEnergyUpdater {
       public:
        // Reward per distinct genome received in the window (paper default 3.0 energy units).
        explicit AggregationEnergyUpdater(double reward_per_genome = 3.0);
        // Returns the alpha component proportional to distinct genomes received in the last P cycles.
        double computeProximityReward(int distinct_genome_count) const;
        // Returns the movement quality gamma from signed wheel speeds in [-1, 1]; -1 on counter-rotation, else OmegaS * omega_s.
        double computeMovementQuality(double left_wheel_speed, double right_wheel_speed) const;
        // Returns the full energy delta combining proximity and movement terms.
        double computeEnergyDelta(int distinct_genome_count, double left_wheel_speed, double right_wheel_speed) const;
        // Returns the configured per-genome reward.
        double getRewardPerGenome() const;

       private:
        // Energy units granted per distinct genome in the window.
        double reward_per_genome_;
    };

    // NavigationEnergyUpdater computes fnorm(V * (1 - sqrt(delta_v)) * (1 - dr) * (1 - dw)) for safe fast motion.
    // Citation: Silva et al. (2015), Section 4.4, Equation 5 adapted from Floreano and Mondada (1994).
    class NavigationEnergyUpdater {
       public:
        // Constructs a stateless updater; fnorm maps [0, 1] linearly into [-1, 1].
        NavigationEnergyUpdater();
        // Maps a raw quality in [0, 1] linearly into [-1, 1].
        double normaliseEnergy(double raw_quality) const;
        // Returns the energy delta from normalised speed sum V, wheel difference delta_v and peak robot/wall activations.
        double computeEnergyDelta(double speed_sum, double wheel_difference, double robot_activation, double wall_activation) const;
        // Derives V in [0, 1] from signed wheel speeds in [-1, 1] as (|vl| + |vr|) / 2.
        static double computeSpeedSum(double left_wheel_speed, double right_wheel_speed);
        // Derives delta_v in [0, 1] from signed wheel speeds as |vl - vr| / 2.
        static double computeWheelDifference(double left_wheel_speed, double right_wheel_speed);
    };

    // PhototaxisEnergyUpdater rewards closeness to the light source with a dead zone and idle penalty.
    // Citation: Silva et al. (2015), Section 4.5, Equation 6.
    class PhototaxisEnergyUpdater {
       public:
        // Constructs an updater with bright threshold 0.5 and idle penalty -0.01 (paper defaults).
        PhototaxisEnergyUpdater(double bright_threshold = 0.5, double idle_penalty = -0.01);
        // Returns Sr when bright, 0 in the dead zone and the idle penalty when no light is sensed.
        double computeEnergyDelta(double maximum_light_reading) const;
        // Returns the bright threshold above which proportional reward applies.
        double getBrightThreshold() const;
        // Returns the penalty applied when no light is sensed.
        double getIdlePenalty() const;

       private:
        // Light reading above which reward equals the reading.
        double bright_threshold_;
        // Penalty applied when the maximum light reading is exactly zero.
        double idle_penalty_;
    };

}  // namespace odneat

#endif  // ODNEAT_TASKS_TASK_ENERGIES_H_
