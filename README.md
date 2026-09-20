# odNEAT.cpp

C++23 implementation of odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers.

Each virtual robot runs the odNEAT algorithm independently with:
- Local timestamp innovation clocks
- Speciated internal population
- Tabu filter
- Broadcast policy
- Maturation guard

A headless e-puck mini-simulator for aggregation, navigation and phototaxis is included.

## Build and test

Ninja is the default generator. If Ninja is unavailable, use the `release-make` fallback preset (requires Unix make).
Both presets generate `compile_commands.json`.

```bash
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
```

Fallback:

```bash
cmake --preset release-make
cmake --build --preset release-make
ctest --preset release-make --output-on-failure
```

## Demos

```bash
./build/run_experiment --task aggregation --cycles 2000 --seed 42
./build/fault_injection --task navigation --cycles 3000
./build/ablation full
./build/minimal_embed
```

## Library use

Consume as a static library through the public API (`include/odneat.h` umbrella).

```cmake
add_subdirectory(odNEAT)
target_link_libraries(my_app PRIVATE odneat::odneat)
```

Installed use:

```cmake
find_package(odneat 1.0 REQUIRED)
target_link_libraries(my_app PRIVATE odneat::odneat)
```

```bash
cmake --preset release
cmake --build --preset release
cmake --install build --prefix /path/to/install
```

Minimal embed (`examples/minimal_embed.cpp`):

```cpp
#include "odneat.h"

odneat::OdneatParams evolution_params{};
odneat::OdneatAgent agent(0, 4, 2, 0.0, 100.0, 50.0, 0.0, evolution_params, 42ULL);
agent.executeControlCycle(sensor_inputs, energy_delta, received_genomes);

std::string checkpoint_json{};
std::string error_message{};
odneat::toCheckpointJson(agent, checkpoint_json, &error_message);
odneat::fromCheckpointJson(checkpoint_json, restored_agent, &error_message);
```

JSON checkpoints are the language-agnostic interchange boundary (see
`docs/checkpoint_format.md`): full-agent resume is bit-identical including the
`mt19937_64` stream, populations, tabu history, energy, fitness, clock and
switches. Network activations reset on load; species recompute deterministically.

## Citations & Acknowledgements

This repository is an independent implementation of the following research papers:

> Silva, F., Urbano, P., Correia, L., & Christensen, A. L. (2015). *odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers*. **Evolutionary Computation, 23**(3), 421–449. https://doi.org/10.1162/EVCO_a_00141

```bibtex
@article{silva2015odneat,
  author    = {Silva, Fernando and Urbano, Paulo and Correia, Lu{\'\i}s and Christensen, Anders Lyhne},
  title     = {odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers},
  journal   = {Evolutionary Computation},
  volume    = {23},
  number    = {3},
  pages     = {421--449},
  year      = {2015},
  month     = sep,
  publisher = {MIT Press},
  doi       = {10.1162/EVCO_a_00141},
  url       = {https://doi.org/10.1162/EVCO_a_00141}
}
```

> Stanley, K. O., & Miikkulainen, R. (2002). *Evolving Neural Networks Through Augmenting Topologies*. **Evolutionary Computation, 10**(2), 99–127. https://doi.org/10.1162/106365602320169811

```bibtex
@article{stanley2002neat,
  author    = {Stanley, Kenneth O. and Miikkulainen, Risto},
  title     = {Evolving Neural Networks Through Augmenting Topologies},
  journal   = {Evolutionary Computation},
  volume    = {10},
  number    = {2},
  pages     = {99--127},
  year      = {2002},
  month     = jun,
  publisher = {MIT Press},
  doi       = {10.1162/106365602320169811},
  url       = {https://doi.org/10.1162/106365602320169811}
}
```
