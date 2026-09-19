#include <iostream>
#include <string>

#include "config.h"
#include "experiment_driver.h"

int main(int argument_count, char *argument_values[]) {
    std::string ablation_name = "full";

    if (argument_count > 1) {
        ablation_name = argument_values[1];
    }

    odneat::OdneatParams evolution_params{};
    odneat::SimulationParams simulation_params{};
    odneat::demos::ExperimentOptions experiment_options{};
    experiment_options.task_kind = odneat::demos::TaskKind::kAggregation;
    experiment_options.control_cycles = 1500;
    experiment_options.random_seed = 11;

    if (ablation_name == "minimal_population") {
        evolution_params.internal_population_size = 2;
    }

    if (ablation_name == "no_exchange") {
        experiment_options.exchange_enabled = false;
    }

    if (ablation_name == "no_tabu") {
        experiment_options.tabu_enabled = false;
    }

    if (ablation_name == "no_maturation") {
        experiment_options.maturation_enabled = false;
    }

    if (ablation_name == "no_niching") {
        experiment_options.speciation_enabled = false;
    }

    if (ablation_name == "no_crossover") {
        experiment_options.crossover_enabled = false;
    }

    const odneat::demos::ExperimentSummary summary = odneat::demos::runHeadlessExperiment(experiment_options, evolution_params, simulation_params);

    std::cout << "ablation=" << ablation_name << " mean_energy=" << summary.mean_energy << " mean_complexity=" << summary.mean_complexity
              << " replacements=" << summary.total_replacements << "\n";

    return 0;
}
