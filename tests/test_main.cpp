// Unified test runner: single `odneat_tests` executable for all suites.
// Usage: odneat_tests [filter] [--list]
//   No arguments runs every suite. An optional filter runs only the suites
//   whose names contain the filter string (e.g. `odneat_tests genome`).

#include <iostream>
#include <string>
#include <vector>

int run_innovation_clock_suite();
int run_genome_suite();
int run_compatibility_suite();
int run_crossover_suite();
int run_mutation_suite();
int run_network_suite();
int run_energy_suite();
int run_population_suite();
int run_tabu_suite();
int run_broadcast_suite();
int run_agent_suite();
int run_task_energies_suite();
int run_simulation_suite();
int run_integration_suite();
int run_serialization_suite();

namespace {

    struct TestSuite {
        const char *name;
        int (*run)();
    };

    const std::vector<TestSuite> kSuites = {
        {"test_innovation_clock", run_innovation_clock_suite},
        {"test_genome", run_genome_suite},
        {"test_compatibility", run_compatibility_suite},
        {"test_crossover", run_crossover_suite},
        {"test_mutation", run_mutation_suite},
        {"test_network", run_network_suite},
        {"test_energy", run_energy_suite},
        {"test_population", run_population_suite},
        {"test_tabu", run_tabu_suite},
        {"test_broadcast", run_broadcast_suite},
        {"test_agent", run_agent_suite},
        {"test_task_energies", run_task_energies_suite},
        {"test_simulation", run_simulation_suite},
        {"test_integration", run_integration_suite},
        {"test_serialization", run_serialization_suite},
    };

    void listSuites() {
        std::cout << "Available suites:\n";

        for (const TestSuite &suite : kSuites) {
            std::cout << "  " << suite.name << "\n";
        }
    }

}  // namespace

int main(int argument_count, char *argument_values[]) {
    std::string filter;

    for (int i = 1; i < argument_count; ++i) {
        const std::string argument = argument_values[i];

        if (argument == "--list") {
            listSuites();

            return 0;
        }

        if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: odneat_tests [filter] [--list]\n";
            listSuites();

            return 0;
        }

        filter = argument;
    }

    int ran_count = 0;
    int failed_count = 0;

    for (const TestSuite &suite : kSuites) {
        if (!filter.empty() && std::string(suite.name).find(filter) == std::string::npos) {
            continue;
        }

        ++ran_count;
        std::cout << "=== " << suite.name << " ===\n";
        const int result = suite.run();

        if (result != 0) {
            ++failed_count;
            std::cout << suite.name << " FAILED\n";
        }
    }

    if (ran_count == 0) {
        std::cout << "No suites match filter \"" << filter << "\".\n";
        listSuites();

        return 2;
    }

    std::cout << "odneat_tests: " << (ran_count - failed_count) << "/" << ran_count << " suites passed\n";

    return failed_count == 0 ? 0 : 1;
}
