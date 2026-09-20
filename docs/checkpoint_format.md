# Checkpoint JSON format (v1)

Full-agent checkpoints are the language-agnostic interchange boundary. Any JSON
parser can read or write them (e.g. Python `json.load`). Produced by
`toCheckpointJson` / `saveCheckpointToFile`, consumed by `fromCheckpointJson` /
`loadCheckpointFromFile` (`include/serialization.h`, glaze backend, DTO isolated
to `src/serialization.cpp`).

## Envelope

Top-level object, flat, `snake_case` keys (stable within a format version):

| Key | Type | Notes |
|---|---|---|
| `format` | string | Always `"odneat-agent-checkpoint"`. Mismatch fails. |
| `version` | int | Always `1`. Mismatch fails. |
| `robot_identifier` | uint | Must match the restoring agent's id. |
| `input_count`, `output_count` | int | Must match the restoring agent's dimensions. |
| `parameters` | object | All `OdneatParams` fields; adopted on restore. |
| `active` | object | Active `Genome` (see genome object). |
| `population` | array | Internal population in insertion order (1..N). |
| `tabu` | array | Tabu genomes (may be empty). |
| `recent_history` | array | Recent-history window (may be empty). |
| `minimum_energy`, `maximum_energy`, `default_energy`, `minimum_threshold` | double | Must match the restoring agent's bounds. |
| `current_energy` | double | Clamped into bounds on restore. |
| `fitness_mean` | double | Fitness averager mean. |
| `fitness_sample_count` | int | Samples in the mean (>= 0). |
| `clock_minted_count` | uint | Innovation clock counter. |
| `clock_last_timestamp` | uint64 | Innovation clock last timestamp (restored forward-only). |
| `random_state` | string | Full `mt19937_64` `operator<<` stream; future draws match exactly. |
| `maturation_cycles_remaining` | int | Guard counter. |
| `evaluation_count` | int | Controllers evaluated (>= 1). |
| `exchange_enabled`, `tabu_enabled`, `maturation_enabled`, `speciation_enabled` | bool | Ablation switches. |

Unknown top-level keys are ignored (forward compatibility). `Species` partitions
are recomputed on load; network activations reset. Single-genome documents use
`format: "odneat-genome"`, `version: 1` with `{genome: {...}}`.

## Genome object

```json
{
  "neurons": [{"innovation_id": {"robot_identifier": 0, "timestamp_nanoseconds": 7, "local_counter": 7},
               "neuron_type": "input", "input_index": 0, "output_index": -1}],
  "connections": [{"innovation_id": {...}, "input_neuron_id": {...}, "output_neuron_id": {...},
                   "weight": 0.5, "enabled": true}],
  "fitness": 50.0, "evaluation_count": 1, "adjusted_fitness": 50.0
}
```

`neuron_type` is one of `"input"`, `"hidden"`, `"output"`, `"bias"`.
Doubles round-trip exactly; weights are bit-identical after load.

## Python example

```python
import json
doc = json.load(open("checkpoint.json"))
print(doc["format"], doc["version"], len(doc["population"]))
print(doc["active"]["fitness"], doc["current_energy"])
```
