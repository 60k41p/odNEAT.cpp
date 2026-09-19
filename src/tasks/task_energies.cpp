#include "tasks/task_energies.h"

#include <algorithm>
#include <cmath>

namespace odneat {

    AggregationEnergyUpdater::AggregationEnergyUpdater(double reward_per_genome) : reward_per_genome_(reward_per_genome) {}

    double AggregationEnergyUpdater::computeProximityReward(int distinct_genome_count) const {
        return reward_per_genome_ * static_cast<double>(distinct_genome_count);
    }

    double AggregationEnergyUpdater::computeMovementQuality(double left_wheel_speed, double right_wheel_speed) const {
        if (left_wheel_speed * right_wheel_speed < 0.0) {
            return -1.0;
        }

        const double average_speed = (std::fabs(left_wheel_speed) + std::fabs(right_wheel_speed)) * 0.5;
        const double straightness = std::sqrt(std::max(0.0, left_wheel_speed * right_wheel_speed));

        return average_speed * straightness;
    }

    double AggregationEnergyUpdater::computeEnergyDelta(int distinct_genome_count, double left_wheel_speed, double right_wheel_speed) const {
        return computeProximityReward(distinct_genome_count) + computeMovementQuality(left_wheel_speed, right_wheel_speed);
    }

    double AggregationEnergyUpdater::getRewardPerGenome() const { return reward_per_genome_; }

    NavigationEnergyUpdater::NavigationEnergyUpdater() = default;

    double NavigationEnergyUpdater::normaliseEnergy(double raw_quality) const {
        const double clamped_quality = std::clamp(raw_quality, 0.0, 1.0);

        return clamped_quality * 2.0 - 1.0;
    }

    double NavigationEnergyUpdater::computeEnergyDelta(double speed_sum, double wheel_difference, double robot_activation, double wall_activation) const {
        const double clamped_speed = std::clamp(speed_sum, 0.0, 1.0);
        const double clamped_difference = std::clamp(wheel_difference, 0.0, 1.0);
        const double clamped_robot = std::clamp(robot_activation, 0.0, 1.0);
        const double clamped_wall = std::clamp(wall_activation, 0.0, 1.0);
        const double raw_quality = clamped_speed * (1.0 - std::sqrt(clamped_difference)) * (1.0 - clamped_robot) * (1.0 - clamped_wall);

        return normaliseEnergy(raw_quality);
    }

    double NavigationEnergyUpdater::computeSpeedSum(double left_wheel_speed, double right_wheel_speed) {
        return (std::fabs(left_wheel_speed) + std::fabs(right_wheel_speed)) * 0.5;
    }

    double NavigationEnergyUpdater::computeWheelDifference(double left_wheel_speed, double right_wheel_speed) {
        return std::fabs(left_wheel_speed - right_wheel_speed) * 0.5;
    }

    PhototaxisEnergyUpdater::PhototaxisEnergyUpdater(double bright_threshold, double idle_penalty)
        : bright_threshold_(bright_threshold), idle_penalty_(idle_penalty) {}

    double PhototaxisEnergyUpdater::computeEnergyDelta(double maximum_light_reading) const {
        if (maximum_light_reading <= 0.0) {
            return idle_penalty_;
        }

        if (maximum_light_reading > bright_threshold_) {
            return maximum_light_reading;
        }

        return 0.0;
    }

    double PhototaxisEnergyUpdater::getBrightThreshold() const { return bright_threshold_; }

    double PhototaxisEnergyUpdater::getIdlePenalty() const { return idle_penalty_; }

}  // namespace odneat
