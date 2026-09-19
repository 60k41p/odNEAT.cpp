#include "sim/simulation.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace odneat {
    namespace sim {

        namespace {

            constexpr double kTwoPi = 2.0 * std::numbers::pi;

            double normaliseAngle(double heading_radians) {
                double wrapped_angle = std::fmod(heading_radians, kTwoPi);

                if (wrapped_angle > std::numbers::pi) {
                    wrapped_angle -= kTwoPi;
                }

                if (wrapped_angle < -std::numbers::pi) {
                    wrapped_angle += kTwoPi;
                }

                return wrapped_angle;
            }

            double distanceBetween(double first_x, double first_y, double second_x, double second_y) {
                const double delta_x = first_x - second_x;
                const double delta_y = first_y - second_y;

                return std::sqrt(delta_x * delta_x + delta_y * delta_y);
            }

        }  // namespace

        double addSensorNoise(double clean_reading, std::mt19937_64 &randomGenerator, double noise_standard_deviation) {
            if (noise_standard_deviation <= 0.0) {
                return std::clamp(clean_reading, 0.0, 1.0);
            }

            std::normal_distribution<double> noiseDistribution(0.0, noise_standard_deviation);

            return std::clamp(clean_reading + noiseDistribution(randomGenerator), 0.0, 1.0);
        }

        void integrateDifferentialDrive(
            RobotPose &pose, double left_wheel_speed, double right_wheel_speed, double time_step_seconds, const SimulationParams &simulation_params) {
            const double clamped_left = std::clamp(left_wheel_speed, -1.0, 1.0);
            const double clamped_right = std::clamp(right_wheel_speed, -1.0, 1.0);
            const double linear_velocity = (clamped_left + clamped_right) * 0.5 * simulation_params.maximum_wheel_speed;
            const double angular_velocity = (clamped_right - clamped_left) * simulation_params.maximum_wheel_speed / simulation_params.wheel_base_metres;
            pose.heading_radians = normaliseAngle(pose.heading_radians + angular_velocity * time_step_seconds);
            pose.position_x += linear_velocity * std::cos(pose.heading_radians) * time_step_seconds;
            pose.position_y += linear_velocity * std::sin(pose.heading_radians) * time_step_seconds;
        }

        void resolveWallCollision(RobotPose &pose, const SimulationParams &simulation_params, double robot_radius_metres) {
            // Stall on contact: clamp inside arena, keep heading (e-puck pushes against wall, no bounce).
            const double half_arena = simulation_params.arena_size_metres * 0.5;

            if (pose.position_x < -half_arena + robot_radius_metres) {
                pose.position_x = -half_arena + robot_radius_metres;
            }

            if (pose.position_x > half_arena - robot_radius_metres) {
                pose.position_x = half_arena - robot_radius_metres;
            }

            if (pose.position_y < -half_arena + robot_radius_metres) {
                pose.position_y = -half_arena + robot_radius_metres;
            }

            if (pose.position_y > half_arena - robot_radius_metres) {
                pose.position_y = half_arena - robot_radius_metres;
            }

            pose.heading_radians = normaliseAngle(pose.heading_radians);
        }

        std::vector<double> computeWallSensors(const RobotPose &observer_pose, const SimulationParams &simulation_params) {
            std::vector<double> wallActivations(8, 0.0);
            const double half_arena = simulation_params.arena_size_metres * 0.5;
            const double sensor_range = simulation_params.broadcast_range_metres;

            for (int sensor_index = 0; sensor_index < 8; ++sensor_index) {
                const double sensor_angle = observer_pose.heading_radians + static_cast<double>(sensor_index) * (kTwoPi / 8.0);
                const double direction_x = std::cos(sensor_angle);
                const double direction_y = std::sin(sensor_angle);
                double distance_to_wall = sensor_range;

                if (direction_x > 1e-9) {
                    distance_to_wall = std::min(distance_to_wall, (half_arena - observer_pose.position_x) / direction_x);
                } else if (direction_x < -1e-9) {
                    distance_to_wall = std::min(distance_to_wall, (observer_pose.position_x + half_arena) / -direction_x);
                }

                if (direction_y > 1e-9) {
                    distance_to_wall = std::min(distance_to_wall, (half_arena - observer_pose.position_y) / direction_y);
                } else if (direction_y < -1e-9) {
                    distance_to_wall = std::min(distance_to_wall, (observer_pose.position_y + half_arena) / -direction_y);
                }

                if (distance_to_wall < 0.0) {
                    distance_to_wall = 0.0;
                }

                if (distance_to_wall < sensor_range) {
                    wallActivations[static_cast<std::size_t>(sensor_index)] = 1.0 - (distance_to_wall / sensor_range);
                }
            }

            return wallActivations;
        }

        std::vector<double> computeRobotSensors(const RobotPose &observer_pose,
                                                const std::vector<SimulatedRobot> &all_robots,
                                                std::size_t observer_index,
                                                const SimulationParams &simulation_params) {
            std::vector<double> robotActivations(8, 0.0);
            const double sensor_range = simulation_params.broadcast_range_metres;

            for (std::size_t robot_index = 0; robot_index < all_robots.size(); ++robot_index) {
                if (robot_index == observer_index) {
                    continue;
                }

                const RobotPose &other_pose = all_robots[robot_index].pose;
                const double distance = distanceBetween(observer_pose.position_x, observer_pose.position_y, other_pose.position_x, other_pose.position_y);

                if (distance > sensor_range || distance <= 1e-9) {
                    continue;
                }

                const double bearing =
                    normaliseAngle(std::atan2(other_pose.position_y - observer_pose.position_y, other_pose.position_x - observer_pose.position_x) -
                                   observer_pose.heading_radians);
                int closest_sensor = 0;
                double closest_difference = kTwoPi;

                for (int sensor_index = 0; sensor_index < 8; ++sensor_index) {
                    const double sensor_angle = static_cast<double>(sensor_index) * (kTwoPi / 8.0);
                    double normalised_bearing = bearing;

                    if (normalised_bearing < 0.0) {
                        normalised_bearing += kTwoPi;
                    }

                    double angular_difference = std::fabs(normalised_bearing - sensor_angle);

                    if (angular_difference > std::numbers::pi) {
                        angular_difference = kTwoPi - angular_difference;
                    }

                    if (angular_difference < closest_difference) {
                        closest_difference = angular_difference;
                        closest_sensor = sensor_index;
                    }
                }

                const double falloff = 1.0 - (distance / sensor_range);
                const double directional_gain = std::max(0.0, std::cos(closest_difference));
                const double activation = falloff * (0.5 + 0.5 * directional_gain);
                std::size_t sensor_slot = static_cast<std::size_t>(closest_sensor);
                robotActivations[sensor_slot] = std::max(robotActivations[sensor_slot], activation);
            }

            return robotActivations;
        }

        std::vector<double> computeLightSensors(const RobotPose &observer_pose, double light_x, double light_y, const SimulationParams &simulation_params) {
            // Directional light sensing (Tables 3-5 assume 8 directional readings):
            // distance falloff times per-sensor angular gain max(0,cos(diff)) to the light bearing.
            std::vector<double> lightActivations(8, 0.0);
            const double distance = distanceBetween(observer_pose.position_x, observer_pose.position_y, light_x, light_y);

            if (distance > simulation_params.light_sensor_range_metres || distance <= 1e-9) {
                if (distance <= 1e-9) {
                    std::fill(lightActivations.begin(), lightActivations.end(), 1.0);
                }

                return lightActivations;
            }

            const double falloff = 1.0 - (distance / simulation_params.light_sensor_range_metres);
            const double bearing =
                normaliseAngle(std::atan2(light_y - observer_pose.position_y, light_x - observer_pose.position_x) - observer_pose.heading_radians);
            for (int sensor_index = 0; sensor_index < 8; ++sensor_index) {
                const double sensor_angle = static_cast<double>(sensor_index) * (kTwoPi / 8.0);
                double diff = std::fabs(bearing < 0.0 ? bearing + kTwoPi : bearing - sensor_angle);
                if (diff > std::numbers::pi) {
                    diff = kTwoPi - diff;
                }

                const double gain = std::max(0.0, std::cos(diff));
                lightActivations[static_cast<std::size_t>(sensor_index)] = falloff * gain;
            }

            return lightActivations;
        }

        std::vector<double> buildAggregationInputs(const SensorReadings &readings, double normalised_energy, double normalised_genome_count) {
            std::vector<double> controller_inputs{};
            controller_inputs.reserve(18);
            controller_inputs.insert(controller_inputs.end(), readings.robot_sensors.begin(), readings.robot_sensors.end());
            controller_inputs.insert(controller_inputs.end(), readings.wall_sensors.begin(), readings.wall_sensors.end());
            controller_inputs.push_back(std::clamp(normalised_energy, 0.0, 1.0));
            controller_inputs.push_back(std::clamp(normalised_genome_count, 0.0, 1.0));

            return controller_inputs;
        }

        std::vector<double> buildNavigationInputs(const SensorReadings &readings, double normalised_energy) {
            std::vector<double> controller_inputs{};
            controller_inputs.reserve(17);
            controller_inputs.insert(controller_inputs.end(), readings.robot_sensors.begin(), readings.robot_sensors.end());
            controller_inputs.insert(controller_inputs.end(), readings.wall_sensors.begin(), readings.wall_sensors.end());
            controller_inputs.push_back(std::clamp(normalised_energy, 0.0, 1.0));

            return controller_inputs;
        }

        std::vector<double> buildPhototaxisInputs(const SensorReadings &readings, double normalised_energy) {
            std::vector<double> controller_inputs{};
            controller_inputs.reserve(25);
            controller_inputs.insert(controller_inputs.end(), readings.robot_sensors.begin(), readings.robot_sensors.end());
            controller_inputs.insert(controller_inputs.end(), readings.wall_sensors.begin(), readings.wall_sensors.end());
            controller_inputs.insert(controller_inputs.end(), readings.light_sensors.begin(), readings.light_sensors.end());
            controller_inputs.push_back(std::clamp(normalised_energy, 0.0, 1.0));

            return controller_inputs;
        }

        int countNeighboursInRange(const std::vector<SimulatedRobot> &all_robots, std::size_t observer_index, double range_metres) {
            int neighbour_count = 0;

            for (std::size_t robot_index = 0; robot_index < all_robots.size(); ++robot_index) {
                if (robot_index == observer_index) {
                    continue;
                }

                const double distance = distanceBetween(all_robots[observer_index].pose.position_x, all_robots[observer_index].pose.position_y,
                                                        all_robots[robot_index].pose.position_x, all_robots[robot_index].pose.position_y);

                if (distance <= range_metres) {
                    ++neighbour_count;
                }
            }

            return neighbour_count;
        }

        std::vector<std::size_t> findNeighboursInRange(const std::vector<SimulatedRobot> &all_robots, std::size_t observer_index, double range_metres) {
            std::vector<std::size_t> neighbour_indices{};

            for (std::size_t robot_index = 0; robot_index < all_robots.size(); ++robot_index) {
                if (robot_index == observer_index) {
                    continue;
                }

                const double distance = distanceBetween(all_robots[observer_index].pose.position_x, all_robots[observer_index].pose.position_y,
                                                        all_robots[robot_index].pose.position_x, all_robots[robot_index].pose.position_y);

                if (distance <= range_metres) {
                    neighbour_indices.push_back(robot_index);
                }
            }

            return neighbour_indices;
        }

        void placeRobotsWithSeparation(std::vector<SimulatedRobot> &all_robots,
                                       std::mt19937_64 &randomGenerator,
                                       const SimulationParams &simulation_params,
                                       double minimum_separation_metres) {
            const double half_arena = simulation_params.arena_size_metres * 0.5 - simulation_params.robot_diameter_metres;
            std::uniform_real_distribution<double> positionDistribution(-half_arena, half_arena);
            std::uniform_real_distribution<double> headingDistribution(-std::numbers::pi, std::numbers::pi);

            for (std::size_t robot_index = 0; robot_index < all_robots.size(); ++robot_index) {
                RobotPose chosen_pose{};
                chosen_pose.position_x = positionDistribution(randomGenerator);
                chosen_pose.position_y = positionDistribution(randomGenerator);
                chosen_pose.heading_radians = headingDistribution(randomGenerator);
                for (int attempt_index = 0; attempt_index < 500; ++attempt_index) {
                    RobotPose candidate_pose{};
                    candidate_pose.position_x = positionDistribution(randomGenerator);
                    candidate_pose.position_y = positionDistribution(randomGenerator);
                    candidate_pose.heading_radians = headingDistribution(randomGenerator);
                    bool separated_enough = true;

                    for (std::size_t other_index = 0; other_index < robot_index; ++other_index) {
                        if (distanceBetween(candidate_pose.position_x, candidate_pose.position_y, all_robots[other_index].pose.position_x,
                                            all_robots[other_index].pose.position_y) < minimum_separation_metres) {
                            separated_enough = false;
                            break;
                        }
                    }

                    chosen_pose = candidate_pose;
                    if (separated_enough) {
                        break;
                    }
                }

                all_robots[robot_index].pose = chosen_pose;
            }
        }

        void placeRobotsUniformly(std::vector<SimulatedRobot> &all_robots, std::mt19937_64 &randomGenerator, const SimulationParams &simulation_params) {
            const double half_arena = simulation_params.arena_size_metres * 0.5 - simulation_params.robot_diameter_metres;
            std::uniform_real_distribution<double> positionDistribution(-half_arena, half_arena);
            std::uniform_real_distribution<double> headingDistribution(-std::numbers::pi, std::numbers::pi);

            for (SimulatedRobot &robot : all_robots) {
                robot.pose.position_x = positionDistribution(randomGenerator);
                robot.pose.position_y = positionDistribution(randomGenerator);
                robot.pose.heading_radians = headingDistribution(randomGenerator);
            }
        }

        bool injectRandomSensorFault(SimulatedRobot &robot, std::mt19937_64 &randomGenerator) {
            std::vector<std::vector<bool> *> sensor_banks{&robot.robot_sensor_faults, &robot.wall_sensor_faults, &robot.light_sensor_faults};
            std::vector<std::pair<std::vector<bool> *, int>> healthy_slots{};

            for (std::vector<bool> *bank : sensor_banks) {
                for (int sensor_index = 0; sensor_index < static_cast<int>(bank->size()); ++sensor_index) {
                    if (!(*bank)[static_cast<std::size_t>(sensor_index)]) {
                        healthy_slots.emplace_back(bank, sensor_index);
                    }
                }
            }

            if (healthy_slots.empty()) {
                return false;
            }

            std::uniform_int_distribution<std::size_t> slotPicker(0, healthy_slots.size() - 1);
            const auto [chosen_bank, chosen_index] = healthy_slots[slotPicker(randomGenerator)];
            (*chosen_bank)[static_cast<std::size_t>(chosen_index)] = true;

            return true;
        }

        int countFaultySensors(const SimulatedRobot &robot) {
            int faulty_count = 0;

            for (bool faulty_flag : robot.robot_sensor_faults) {
                if (faulty_flag) {
                    ++faulty_count;
                }
            }

            for (bool faulty_flag : robot.wall_sensor_faults) {
                if (faulty_flag) {
                    ++faulty_count;
                }
            }

            for (bool faulty_flag : robot.light_sensor_faults) {
                if (faulty_flag) {
                    ++faulty_count;
                }
            }

            return faulty_count;
        }

    }  // namespace sim
}  // namespace odneat
