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
```

## Citation & Acknowledgements

This repository is an independent implementation of the following research paper:

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
