/* Minimal headless 2D differential-drive simulator for e-puck-like robots.
 *
 * This header provides arena kinematics, multiplexed infrared sensing with communication, light sensing, Gaussian noise and range-limited genome exchange. It
 * is intentionally small and deterministic: poses integrate with fixed control cycles, sensors use analytic range falloff with angular selectivity, and the
 * channel delivers broadcasts to neighbours within range. Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic
 * Controllers", Section 4.1 and Tables 2-5 (e-puck model, 3x3 m arena, 100 ms cycles, 25 cm IR range, 50 cm light range, +/-5% noise).
 */

#ifndef ODNEAT_SIM_SIMULATION_H_
#define ODNEAT_SIM_SIMULATION_H_

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "config.h"
#include "genome.h"

namespace odneat {
    namespace sim {

        // RobotPose is the planar position and heading of one robot.
        struct RobotPose {
            // X coordinate in metres inside the arena.
            double position_x = 0.0;
            // Y coordinate in metres inside the arena.
            double position_y = 0.0;
            // Heading angle in radians, zero along +X.
            double heading_radians = 0.0;
        };

        // SimulatedRobot owns pose, wheel speeds and per-sensor fault flags for fault-injection studies.
        struct SimulatedRobot {
            // Current planar pose.
            RobotPose pose{};
            // Last commanded left wheel speed in [-1, 1].
            double left_wheel_speed = 0.0;
            // Last commanded right wheel speed in [-1, 1].
            double right_wheel_speed = 0.0;
            // Fault flags for 8 robot sensors, 8 wall sensors and 8 light sensors; faulty sensors read zero.
            std::vector<bool> robot_sensor_faults = std::vector<bool>(8, false);
            std::vector<bool> wall_sensor_faults = std::vector<bool>(8, false);
            std::vector<bool> light_sensor_faults = std::vector<bool>(8, false);
        };

        // SensorReadings bundles normalised infrared, light and energy inputs for one control cycle.
        struct SensorReadings {
            // Eight robot-detection activations in [0, 1].
            std::vector<double> robot_sensors = std::vector<double>(8, 0.0);
            // Eight wall-detection activations in [0, 1].
            std::vector<double> wall_sensors = std::vector<double>(8, 0.0);
            // Eight light-detection activations in [0, 1].
            std::vector<double> light_sensors = std::vector<double>(8, 0.0);
        };

        // Applies Gaussian noise with the configured standard deviation and clamps into [0, 1].
        double addSensorNoise(double clean_reading, std::mt19937_64 &randomGenerator, double noise_standard_deviation);
        // Integrates one differential-drive step for the given pose and wheel speeds.
        void integrateDifferentialDrive(
            RobotPose &pose, double left_wheel_speed, double right_wheel_speed, double time_step_seconds, const SimulationParams &simulation_params);
        // Keeps a pose inside the arena by clamping position (stall, heading unchanged) on wall contact.
        void resolveWallCollision(RobotPose &pose, const SimulationParams &simulation_params, double robot_radius_metres);
        // Computes eight directional infrared activations for walls using analytic arena ray distances.
        std::vector<double> computeWallSensors(const RobotPose &observer_pose, const SimulationParams &simulation_params);
        // Computes eight directional infrared activations for neighbouring robots with distance falloff.
        std::vector<double> computeRobotSensors(const RobotPose &observer_pose,
                                                const std::vector<SimulatedRobot> &all_robots,
                                                std::size_t observer_index,
                                                const SimulationParams &simulation_params);
        // Computes eight directional light activations (distance falloff with per-sensor cos gain).
        std::vector<double> computeLightSensors(const RobotPose &observer_pose, double light_x, double light_y, const SimulationParams &simulation_params);
        // Builds the controller input vector for aggregation (8 robot + 8 wall + energy + genome count = 18).
        std::vector<double> buildAggregationInputs(const SensorReadings &readings, double normalised_energy, double normalised_genome_count);
        // Builds the controller input vector for navigation (8 robot + 8 wall + energy = 17).
        std::vector<double> buildNavigationInputs(const SensorReadings &readings, double normalised_energy);
        // Builds the controller input vector for phototaxis (8 robot + 8 wall + 8 light + energy = 25).
        std::vector<double> buildPhototaxisInputs(const SensorReadings &readings, double normalised_energy);
        // Counts robots within broadcast range of the observer, excluding the observer itself.
        int countNeighboursInRange(const std::vector<SimulatedRobot> &all_robots, std::size_t observer_index, double range_metres);
        // Returns indices of robots within broadcast range of the observer, excluding the observer itself.
        std::vector<std::size_t> findNeighboursInRange(const std::vector<SimulatedRobot> &all_robots, std::size_t observer_index, double range_metres);
        // Places robots uniformly at random with a minimum separation for aggregation initialisation.
        void placeRobotsWithSeparation(std::vector<SimulatedRobot> &all_robots,
                                       std::mt19937_64 &randomGenerator,
                                       const SimulationParams &simulation_params,
                                       double minimum_separation_metres);
        // Places robots uniformly at random with random headings for navigation and phototaxis initialisation.
        void placeRobotsUniformly(std::vector<SimulatedRobot> &all_robots, std::mt19937_64 &randomGenerator, const SimulationParams &simulation_params);
        // Injects one random physical sensor fault following the Carlson et al. (2004) inspired protocol used in Section 6.1.
        bool injectRandomSensorFault(SimulatedRobot &robot, std::mt19937_64 &randomGenerator);
        // Counts faulty physical sensors on one robot (robot + wall + light banks).
        int countFaultySensors(const SimulatedRobot &robot);

    }  // namespace sim
}  // namespace odneat

#endif  // ODNEAT_SIM_SIMULATION_H_
