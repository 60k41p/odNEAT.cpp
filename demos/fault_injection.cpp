#include <iostream>
#include <string>
#include <vector>

#include "config.h"
#include "experiment_driver.h"
#include "sim/simulation.h"

int main(int argument_count, char *argument_values[]) {
    std::string task_name = "navigation";
    int control_cycles = 3000;
    std::uint64_t random_seed = 7;

    for (int argument_index = 1; argument_index < argument_count; ++argument_index) {
        const std::string current_argument = argument_values[argument_index];

        if (current_argument == "--task" && argument_index + 1 < argument_count) {
            task_name = argument_values[++argument_index];
        } else if (current_argument == "--cycles" && argument_index + 1 < argument_count) {
            control_cycles = std::stoi(argument_values[++argument_index]);
        } else if (current_argument == "--seed" && argument_index + 1 < argument_count) {
            random_seed = static_cast<std::uint64_t>(std::stoull(argument_values[++argument_index]));
        }
    }

    odneat::OdneatParams evolution_params{};
    odneat::SimulationParams simulation_params{};
    odneat::demos::ExperimentOptions experiment_options{};
    experiment_options.task_kind = odneat::demos::parseTaskKind(task_name);
    experiment_options.control_cycles = control_cycles;
    experiment_options.random_seed = random_seed;
    std::mt19937_64 faultGenerator(random_seed + static_cast<std::uint64_t>(999ULL));
    std::vector<odneat::sim::SimulatedRobot> probeBodies(5);
    odneat::sim::placeRobotsUniformly(probeBodies, faultGenerator, simulation_params);
    int injected_faults = 0;

    for (int fault_index = 0; fault_index < 10; ++fault_index) {
        if (odneat::sim::injectRandomSensorFault(probeBodies[0], faultGenerator)) {
            ++injected_faults;
        }
    }

    const odneat::demos::ExperimentSummary summary = odneat::demos::runHeadlessExperiment(experiment_options, evolution_params, simulation_params);
    std::cout << "fault_demo injected=" << injected_faults << " faulty_sensors=" << odneat::sim::countFaultySensors(probeBodies[0]) << " task=" << task_name
              << " mean_energy=" << summary.mean_energy << " replacements=" << summary.total_replacements << "\n";

    return 0;
}
