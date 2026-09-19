/* Shared headless experiment driver for aggregation, navigation and phototaxis.
 * Citation: Silva et al. (2015), Sections 4.1 and 4.3-4.5 for setup, energy bounds and sensor layouts.
 */

#ifndef ODNEAT_DEMOS_EXPERIMENT_DRIVER_H_
#define ODNEAT_DEMOS_EXPERIMENT_DRIVER_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "agent.h"
#include "config.h"
#include "sim/simulation.h"
#include "tasks/task_energies.h"

namespace odneat {
    namespace demos {

        enum class TaskKind : unsigned char { kAggregation = 0, kNavigation = 1, kPhototaxis = 2 };

        inline TaskKind parseTaskKind(const std::string &task_name) {
            if (task_name == "navigation") {
                return TaskKind::kNavigation;
            }

            if (task_name == "phototaxis") {
                return TaskKind::kPhototaxis;
            }

            return TaskKind::kAggregation;
        }

        struct ExperimentOptions {
            TaskKind task_kind = TaskKind::kAggregation;
            int robot_count = 5;
            int control_cycles = 2000;
            std::uint64_t random_seed = 42;
            bool verbose_logging = false;
            // Ablation switches (paper Sec. 6.2): routed into every agent at creation.
            bool exchange_enabled = true;
            bool tabu_enabled = true;
            bool maturation_enabled = true;
            bool speciation_enabled = true;
            bool crossover_enabled = true;
        };

        struct ExperimentSummary {
            double mean_energy = 0.0;
            double mean_fitness = 0.0;
            double mean_complexity = 0.0;
            int total_replacements = 0;
            int total_broadcasts = 0;
        };

        // Runs a headless multirobot experiment and returns aggregate statistics.
        inline ExperimentSummary runHeadlessExperiment(const ExperimentOptions &options,
                                                       const OdneatParams &evolution_params,
                                                       const SimulationParams &simulation_params) {
            const int input_count = (options.task_kind == TaskKind::kAggregation) ? 18 : (options.task_kind == TaskKind::kNavigation ? 17 : 25);
            constexpr int output_count = 2;
            double minimum_energy = 0.0;
            double maximum_energy = 100.0;
            double default_energy = 50.0;

            if (options.task_kind == TaskKind::kAggregation) {
                maximum_energy = 2000.0;
                default_energy = 1000.0;
            }

            std::vector<OdneatAgent> agent_group{};
            agent_group.reserve(static_cast<std::size_t>(options.robot_count));

            OdneatParams agent_params = evolution_params;
            if (!options.crossover_enabled) {
                agent_params.crossover_rate = 0.0;
            }

            for (int robot_index = 0; robot_index < options.robot_count; ++robot_index) {
                agent_group.emplace_back(static_cast<std::uint32_t>(robot_index), input_count, output_count, minimum_energy, maximum_energy, default_energy,
                                         minimum_energy, agent_params, options.random_seed + static_cast<std::uint64_t>(robot_index) * 7919ULL);
                agent_group.back().setExchangeEnabled(options.exchange_enabled);
                agent_group.back().setTabuEnabled(options.tabu_enabled);
                agent_group.back().setMaturationEnabled(options.maturation_enabled);
                agent_group.back().setSpeciationEnabled(options.speciation_enabled);
            }

            std::vector<sim::SimulatedRobot> robotBodies(static_cast<std::size_t>(options.robot_count));
            std::mt19937_64 placementGenerator(options.random_seed);

            if (options.task_kind == TaskKind::kAggregation) {
                sim::placeRobotsWithSeparation(robotBodies, placementGenerator, simulation_params, 1.5);
            } else {
                sim::placeRobotsUniformly(robotBodies, placementGenerator, simulation_params);
            }

            AggregationEnergyUpdater aggregation_updater{};
            NavigationEnergyUpdater navigation_updater{};
            PhototaxisEnergyUpdater phototaxis_updater{};
            std::mt19937_64 noiseGenerator(options.random_seed + 12345ULL);
            std::normal_distribution<double> actuatorNoise(0.0, simulation_params.noise_standard_deviation);
            double light_position_x = 1.0;
            double light_position_y = 1.0;
            // Per-robot sliding window of received genome batches for exact distinct-over-P counting (Sec. 4.3).
            std::vector<std::deque<std::vector<Genome>>> genomeWindow(static_cast<std::size_t>(options.robot_count));
            ExperimentSummary summary{};

            auto countDistinctInWindow = [](const std::deque<std::vector<Genome>> &window) -> int {
                std::vector<Genome> unique_genomes{};
                for (const std::vector<Genome> &batch : window) {
                    for (const Genome &candidate : batch) {
                        bool already_seen = false;
                        for (const Genome &seen : unique_genomes) {
                            if (seen.isIdenticalTo(candidate)) {
                                already_seen = true;
                                break;
                            }
                        }

                        if (!already_seen) {
                            unique_genomes.push_back(candidate);
                        }
                    }
                }

                return static_cast<int>(unique_genomes.size());
            };

            for (int cycle_index = 0; cycle_index < options.control_cycles; ++cycle_index) {
                // Phototaxis dynamics: light jumps every 5 min = 3000 cycles at 100 ms (Sec. 4.5).
                if (options.task_kind == TaskKind::kPhototaxis && cycle_index > 0 && cycle_index % 3000 == 0) {
                    std::uniform_real_distribution<double> lightPlacement(-1.2, 1.2);
                    light_position_x = lightPlacement(noiseGenerator);
                    light_position_y = lightPlacement(noiseGenerator);
                }

                // Single unilateral broadcast draw per robot per cycle (Eq. 2); counted directly.
                std::vector<Genome> pendingBroadcasts(static_cast<std::size_t>(options.robot_count));
                std::vector<double> pendingEnergies(static_cast<std::size_t>(options.robot_count), 0.0);
                std::vector<bool> wantsBroadcast(static_cast<std::size_t>(options.robot_count), false);

                for (std::size_t robot_index = 0; robot_index < agent_group.size(); ++robot_index) {
                    wantsBroadcast[robot_index] = agent_group[robot_index].prepareBroadcast(pendingBroadcasts[robot_index], pendingEnergies[robot_index]);
                    if (wantsBroadcast[robot_index]) {
                        ++summary.total_broadcasts;
                    }
                }

                for (std::size_t robot_index = 0; robot_index < agent_group.size(); ++robot_index) {
                    const std::vector<std::size_t> neighbour_indices =
                        sim::findNeighboursInRange(robotBodies, robot_index, simulation_params.broadcast_range_metres);
                    sim::SensorReadings readings{};
                    readings.robot_sensors = sim::computeRobotSensors(robotBodies[robot_index].pose, robotBodies, robot_index, simulation_params);
                    readings.wall_sensors = sim::computeWallSensors(robotBodies[robot_index].pose, simulation_params);
                    readings.light_sensors = sim::computeLightSensors(robotBodies[robot_index].pose, light_position_x, light_position_y, simulation_params);

                    // Paper Sec. 4.1: every sensor subject to Gaussian noise (robot+wall+light).
                    for (double &sensor_value : readings.robot_sensors) {
                        sensor_value = sim::addSensorNoise(sensor_value, noiseGenerator, simulation_params.noise_standard_deviation);
                    }

                    for (double &sensor_value : readings.wall_sensors) {
                        sensor_value = sim::addSensorNoise(sensor_value, noiseGenerator, simulation_params.noise_standard_deviation);
                    }

                    for (double &sensor_value : readings.light_sensors) {
                        sensor_value = sim::addSensorNoise(sensor_value, noiseGenerator, simulation_params.noise_standard_deviation);
                    }

                    std::vector<ReceivedGenome> incoming_genomes{};

                    for (const std::size_t neighbour_index : neighbour_indices) {
                        if (wantsBroadcast[neighbour_index]) {
                            ReceivedGenome received_genome{};
                            received_genome.genome = pendingBroadcasts[neighbour_index];
                            received_genome.sender_energy = pendingEnergies[neighbour_index];
                            incoming_genomes.push_back(received_genome);
                        }
                    }

                    // Exact distinct-over-P window (Sec. 4.3: n different genomes in last P=10 cycles).
                    std::vector<Genome> incoming_batch{};
                    incoming_batch.reserve(incoming_genomes.size());
                    for (const ReceivedGenome &received : incoming_genomes) {
                        incoming_batch.push_back(received.genome);
                    }

                    genomeWindow[robot_index].push_back(std::move(incoming_batch));
                    while (static_cast<int>(genomeWindow[robot_index].size()) > simulation_params.aggregation_genome_window) {
                        genomeWindow[robot_index].pop_front();
                    }

                    const int distinct_count = countDistinctInWindow(genomeWindow[robot_index]);
                    const double max_distinct = static_cast<double>(std::max(1, options.robot_count - 1) * simulation_params.aggregation_genome_window);

                    std::vector<double> sensor_inputs{};
                    const double normalised_energy = agent_group[robot_index].getEnergy() / maximum_energy;

                    if (options.task_kind == TaskKind::kAggregation) {
                        sensor_inputs = sim::buildAggregationInputs(readings, normalised_energy, static_cast<double>(distinct_count) / max_distinct);
                    } else if (options.task_kind == TaskKind::kNavigation) {
                        sensor_inputs = sim::buildNavigationInputs(readings, normalised_energy);
                    } else {
                        sensor_inputs = sim::buildPhototaxisInputs(readings, normalised_energy);
                    }

                    // Split-phase Algorithm 1: incorporate -> step -> energy from fresh wheels -> update.
                    int accepted_count = 0;
                    int merged_count = 0;
                    int rejected_count = 0;
                    (void)accepted_count;
                    (void)merged_count;
                    (void)rejected_count;
                    agent_group[robot_index].incorporateExternal(incoming_genomes, accepted_count, merged_count, rejected_count);
                    const std::vector<double> raw_outputs = agent_group[robot_index].stepController(sensor_inputs);
                    double left_wheel = 0.0;
                    double right_wheel = 0.0;
                    if (raw_outputs.size() >= 2) {
                        left_wheel = rescaleOutputToWheelSpeed(raw_outputs[0]);
                        right_wheel = rescaleOutputToWheelSpeed(raw_outputs[1]);
                    }

                    // Actuator noise Sec. 4.1 (+/-5%): perturb wheel speeds then clamp.
                    left_wheel = std::clamp(left_wheel + actuatorNoise(noiseGenerator), -1.0, 1.0);
                    right_wheel = std::clamp(right_wheel + actuatorNoise(noiseGenerator), -1.0, 1.0);

                    double energy_delta = 0.0;
                    if (options.task_kind == TaskKind::kAggregation) {
                        energy_delta = aggregation_updater.computeEnergyDelta(distinct_count, left_wheel, right_wheel);
                    } else if (options.task_kind == TaskKind::kNavigation) {
                        double peak_robot = 0.0;
                        double peak_wall = 0.0;

                        for (const double sensor_value : readings.robot_sensors) {
                            peak_robot = std::max(peak_robot, sensor_value);
                        }

                        for (const double sensor_value : readings.wall_sensors) {
                            peak_wall = std::max(peak_wall, sensor_value);
                        }

                        energy_delta = navigation_updater.computeEnergyDelta(NavigationEnergyUpdater::computeSpeedSum(left_wheel, right_wheel),
                                                                             NavigationEnergyUpdater::computeWheelDifference(left_wheel, right_wheel),
                                                                             peak_robot, peak_wall);
                    } else {
                        double peak_light = 0.0;

                        for (const double sensor_value : readings.light_sensors) {
                            peak_light = std::max(peak_light, sensor_value);
                        }

                        energy_delta = phototaxis_updater.computeEnergyDelta(peak_light);
                    }

                    const bool did_replace = agent_group[robot_index].updateEnergyAndMaybeReplace(energy_delta);
                    robotBodies[robot_index].left_wheel_speed = left_wheel;
                    robotBodies[robot_index].right_wheel_speed = right_wheel;
                    sim::integrateDifferentialDrive(robotBodies[robot_index].pose, robotBodies[robot_index].left_wheel_speed,
                                                    robotBodies[robot_index].right_wheel_speed, evolution_params.control_cycle_seconds, simulation_params);
                    sim::resolveWallCollision(robotBodies[robot_index].pose, simulation_params, simulation_params.robot_diameter_metres * 0.5);

                    if (did_replace) {
                        ++summary.total_replacements;
                    }
                }
            }

            double energy_sum = 0.0;
            double fitness_sum = 0.0;
            double complexity_sum = 0.0;

            for (const OdneatAgent &agent : agent_group) {
                energy_sum += agent.getEnergy();
                fitness_sum += agent.getFitness();
                complexity_sum += static_cast<double>(agent.getActiveGenome().computeComplexity());
            }

            summary.mean_energy = energy_sum / static_cast<double>(agent_group.size());
            summary.mean_fitness = fitness_sum / static_cast<double>(agent_group.size());
            summary.mean_complexity = complexity_sum / static_cast<double>(agent_group.size());

            return summary;
        }

    }  // namespace demos
}  // namespace odneat

#endif  // ODNEAT_DEMOS_EXPERIMENT_DRIVER_H_
