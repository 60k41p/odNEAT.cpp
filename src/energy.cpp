#include "energy.h"

#include <algorithm>

namespace odneat {

    EnergyTracker::EnergyTracker(double minimum_energy, double maximum_energy, double default_energy, double minimum_threshold)
        : minimum_energy_(minimum_energy),
          maximum_energy_(maximum_energy),
          default_energy_(default_energy),
          minimum_threshold_(minimum_threshold),
          energy_(default_energy) {}

    double EnergyTracker::getEnergy() const { return energy_; }

    void EnergyTracker::setEnergy(double energy_level) { energy_ = std::clamp(energy_level, minimum_energy_, maximum_energy_); }

    void EnergyTracker::addEnergyDelta(double energy_delta) { setEnergy(energy_ + energy_delta); }

    void EnergyTracker::refillToDefault() { setEnergy(default_energy_); }

    bool EnergyTracker::isDepleted() const { return energy_ <= minimum_threshold_; }

    double EnergyTracker::getMinimumEnergy() const { return minimum_energy_; }

    double EnergyTracker::getMaximumEnergy() const { return maximum_energy_; }

    double EnergyTracker::getDefaultEnergy() const { return default_energy_; }

    double EnergyTracker::getMinimumThreshold() const { return minimum_threshold_; }

    FitnessAverager::FitnessAverager() : mean_fitness_(0.0), sample_count_(0) {}

    void FitnessAverager::addEnergySample(double energy_level) {
        ++sample_count_;
        mean_fitness_ += (energy_level - mean_fitness_) / static_cast<double>(sample_count_);
    }

    void FitnessAverager::mergeRemoteSample(double remote_energy_level) { addEnergySample(remote_energy_level); }

    double FitnessAverager::getFitness() const { return mean_fitness_; }

    int FitnessAverager::getSampleCount() const { return sample_count_; }

    void FitnessAverager::resetAverager() {
        mean_fitness_ = 0.0;
        sample_count_ = 0;
    }

}  // namespace odneat
