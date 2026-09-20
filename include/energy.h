/* Virtual energy tracking and fitness averaging for online evaluation.
 *
 * This header separates instantaneous task performance (virtual energy level E, updated every control cycle) from the evolutionary fitness score (mean of
 * sampled energy). Controllers start at a domain default, gain or lose energy through behaviour, and trigger replacement when energy reaches the minimum
 * threshold. Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 3.1, 4.3, 4.4 and 4.5.
 */

#ifndef ODNEAT_ENERGY_H_
#define ODNEAT_ENERGY_H_

namespace odneat {

    // EnergyTracker maintains the clamped virtual energy level of the active controller.
    class EnergyTracker {
       public:
        // Constructs a tracker with domain bounds, default refill value and minimum replacement threshold.
        EnergyTracker(double minimum_energy, double maximum_energy, double default_energy, double minimum_threshold);
        // Returns the current energy level.
        double getEnergy() const;
        // Overwrites the current energy level with clamping.
        void setEnergy(double energy_level);
        // Adds an energy delta and clamps into [minimum_energy, maximum_energy].
        void addEnergyDelta(double energy_delta);
        // Restores the default initial energy assigned to fresh controllers.
        void refillToDefault();
        // Returns true when energy has reached the replacement threshold.
        bool isDepleted() const;
        // Returns the minimum energy bound.
        double getMinimumEnergy() const;
        // Returns the maximum energy bound.
        double getMaximumEnergy() const;
        // Returns the default refill energy.
        double getDefaultEnergy() const;
        // Returns the replacement threshold.
        double getMinimumThreshold() const;

       private:
        // Lower clamp bound of the energy range.
        double minimum_energy_;
        // Upper clamp bound of the energy range.
        double maximum_energy_;
        // Energy assigned when a new controller starts executing.
        double default_energy_;
        // Threshold at or below which the controller is considered failed.
        double minimum_threshold_;
        // Current energy level.
        double energy_;
    };

    // FitnessAverager accumulates energy samples into the genome fitness score.
    // Citation: Silva et al. (2015), Section 3.1: fitness is the average of the virtual energy sampled at regular intervals.
    class FitnessAverager {
       public:
        // Constructs an empty averager with zero samples.
        FitnessAverager();
        // Adds one energy sample to the running mean.
        void addEnergySample(double energy_level);
        // Merges a remote energy reading for the same genome using incremental averaging (Section 3.2 duplicate handling).
        void mergeRemoteSample(double remote_energy_level);
        // Returns the current mean fitness.
        double getFitness() const;
        // Returns how many samples contributed to the mean.
        int getSampleCount() const;
        // Resets the accumulator for a fresh controller.
        void resetAverager();
        // Restores a persisted mean and sample count (checkpoint resume without replaying samples).
        void restoreState(double mean_fitness, int sample_count);

       private:
        // Running mean of sampled energy levels.
        double mean_fitness_;
        // Number of samples averaged so far.
        int sample_count_;
    };

}  // namespace odneat

#endif  // ODNEAT_ENERGY_H_
