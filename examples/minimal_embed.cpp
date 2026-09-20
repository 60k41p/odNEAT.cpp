// Minimal downstream embedding example: links odneat::odneat, runs a few
// Algorithm 1 cycles with a dummy task reward, then round-trips a JSON checkpoint.
//
// Build via add_subdirectory(odNEAT) + target_link_libraries(app PRIVATE odneat::odneat),
// or via find_package(odneat) after cmake --install.

#include <iostream>
#include <string>
#include <vector>

#include "agent.h"
#include "config.h"
#include "odneat.h"
#include "serialization.h"

int main() {
    odneat::OdneatParams evolution_params{};
    evolution_params.maturation_period_cycles = 10;
    evolution_params.internal_population_size = 10;

    odneat::OdneatAgent agent(0, 4, 2, 0.0, 100.0, 50.0, 0.0, evolution_params, 42ULL);
    odneat::OdneatAgent restored(0, 4, 2, 0.0, 100.0, 50.0, 0.0, evolution_params, 999ULL);

    const std::vector<double> sensors(4, 0.5);
    for (int cycle_index = 0; cycle_index < 20; ++cycle_index) {
        agent.executeControlCycle(sensors, 1.0, {});
    }

    std::string checkpoint_json{};
    std::string error_message{};
    if (!odneat::toCheckpointJson(agent, checkpoint_json, &error_message)) {
        std::cerr << "serialise failed: " << error_message << "\n";
        return 1;
    }
    if (!odneat::fromCheckpointJson(checkpoint_json, restored, &error_message)) {
        std::cerr << "restore failed: " << error_message << "\n";
        return 1;
    }
    if (!restored.getActiveGenome().isIdenticalTo(agent.getActiveGenome())) {
        std::cerr << "checkpoint mismatch\n";
        return 1;
    }

    std::cout << "minimal_embed ok: bytes=" << checkpoint_json.size() << " energy=" << restored.getEnergy() << "\n";
    return 0;
}
