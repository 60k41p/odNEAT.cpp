/* Umbrella header for downstream library use.
 *
 * Includes the full public API: per-robot evolution loop, decoded phenotype,
 * genetic encoding, speciation, energy accounting and JSON checkpoint helpers.
 * Prefer this header in downstream targets (`#include "odneat.h"`); individual
 * headers remain available for finer-grained includes.
 */

#ifndef ODNEAT_H_
#define ODNEAT_H_

#include "agent.h"
#include "broadcast.h"
#include "compatibility.h"
#include "config.h"
#include "crossover.h"
#include "energy.h"
#include "genome.h"
#include "innovation_clock.h"
#include "mutation.h"
#include "network.h"
#include "population.h"
#include "serialization.h"
#include "sim/simulation.h"
#include "tabu_list.h"
#include "tasks/task_energies.h"

#endif  // ODNEAT_H_
