#include <iostream>
#include <string>

#include "config.h"
#include "experiment_driver.h"

int main(int argument_count, char *argument_values[]) {
    std::string task_name = "aggregation";
    int control_cycles = 2000;
    std::uint64_t random_seed = 42;

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
    const odneat::demos::ExperimentSummary summary = odneat::demos::runHeadlessExperiment(experiment_options, evolution_params, simulation_params);
    std::cout << "task=" << task_name << " cycles=" << control_cycles << " seed=" << random_seed << " mean_energy=" << summary.mean_energy
              << " mean_fitness=" << summary.mean_fitness << " mean_complexity=" << summary.mean_complexity << " replacements=" << summary.total_replacements
              << " broadcasts=" << summary.total_broadcasts << "\n";

    return 0;
}
