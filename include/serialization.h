/* JSON checkpoint helpers for library use (glaze backend, DTO isolated to the implementation).
 *
 * The JSON documents produced here are the language-agnostic interchange boundary: any
 * language with a JSON parser can read or write checkpoints (see docs/checkpoint_schema.json).
 * Field names are snake_case and stable within a format version. Unknown fields are ignored
 * on read so newer writers stay loadable by older readers; mismatched format or version fails.
 *
 * Full-agent checkpoints capture everything needed to resume evolution bit-identically:
 * active genome, internal population, tabu list with recent history, energy bounds and level,
 * fitness mean and sample count, innovation clock state, full mt19937_64 stream state
 * (via operator<< / operator>> round-trip), maturation and evaluation counters, parameters
 * and ablation switches. Recurrent-network activations are ephemeral and reset on load;
 * species partitions are recomputed deterministically on load.
 */

#ifndef ODNEAT_SERIALIZATION_H_
#define ODNEAT_SERIALIZATION_H_

#include <string>
#include <string_view>

namespace odneat {

    class Genome;
    class OdneatAgent;

    // Format identifier written into every checkpoint document.
    inline constexpr const char *kCheckpointFormat = "odneat-agent-checkpoint";
    // Format version written into every checkpoint document.
    inline constexpr int kCheckpointVersion = 1;
    // Format identifier for single-genome documents.
    inline constexpr const char *kGenomeFormat = "odneat-genome";
    // Format version for single-genome documents.
    inline constexpr int kGenomeVersion = 1;

    // Serialises one genome (topology, weights, fitness statistics) to minified JSON.
    // Returns false with an optional message on failure; json_output is untouched on failure.
    bool toGenomeJson(const Genome &genome, std::string &json_output, std::string *error_message = nullptr);
    // Parses a document produced by toGenomeJson; returns false on format, version or content errors.
    bool fromGenomeJson(std::string_view json_input, Genome &genome_output, std::string *error_message = nullptr);
    // File helpers wrapping the string API with std::ofstream / std::ifstream (binary-safe, UTF-8 JSON).
    bool saveGenomeToFile(const std::string &file_path, const Genome &genome, std::string *error_message = nullptr);
    // Loads a genome checkpoint file written by saveGenomeToFile.
    bool loadGenomeFromFile(const std::string &file_path, Genome &genome_output, std::string *error_message = nullptr);

    // Serialises the full agent state to minified JSON; suitable for resume and transport.
    bool toCheckpointJson(const OdneatAgent &agent, std::string &json_output, std::string *error_message = nullptr);
    // Serialises the full agent state to human-readable indented JSON (same content, larger).
    bool toCheckpointJsonPretty(const OdneatAgent &agent, std::string &json_output, std::string *error_message = nullptr);
    // Restores an agent from toCheckpointJson output. The target agent must have been
    // constructed with matching robot identifier, input/output counts and energy bounds;
    // parameters are adopted from the checkpoint. Returns false without partial mutation on failure.
    bool fromCheckpointJson(std::string_view json_input, OdneatAgent &agent_output, std::string *error_message = nullptr);
    // File helpers for full-agent checkpoints.
    bool saveCheckpointToFile(const std::string &file_path, const OdneatAgent &agent, std::string *error_message = nullptr);
    // Loads a full-agent checkpoint file written by saveCheckpointToFile.
    bool loadCheckpointFromFile(const std::string &file_path, OdneatAgent &agent_output, std::string *error_message = nullptr);

}  // namespace odneat

#endif  // ODNEAT_SERIALIZATION_H_
