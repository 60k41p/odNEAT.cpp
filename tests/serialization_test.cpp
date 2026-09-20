#include "serialization.h"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "agent.h"
#include "config.h"
#include "genome.h"
#include "innovation_clock.h"

namespace {

    int checkCondition(bool condition, const char *message, int &failure_count) {
        if (!condition) {
            std::cout << "FAIL: " << message << "\n";
            ++failure_count;
        }
        return failure_count;
    }

    bool genomesEqual(const odneat::Genome &left, const odneat::Genome &right) { return left.isIdenticalTo(right); }

    odneat::OdneatAgent makeEvolvedAgent(std::uint32_t robot_identifier, std::uint64_t seed) {
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent agent(robot_identifier, 4, 2, 0.0, 100.0, 50.0, 0.0, params, seed);
        const std::vector<double> sensors(4, 0.5);
        for (int cycle_index = 0; cycle_index < 30; ++cycle_index) {
            agent.executeControlCycle(sensors, (cycle_index % 3 == 0) ? -15.0 : 2.0, {});
        }
        agent.forceControllerReplacement();
        for (int cycle_index = 0; cycle_index < 8; ++cycle_index) {
            agent.executeControlCycle(sensors, 1.5, {});
        }
        return agent;
    }

}  // namespace

int run_serialization_suite() {
    int failure_count = 0;

    // Genome round-trip preserves topology, weights and fitness statistics bit-exactly.
    {
        odneat::InnovationClock clock(3);
        odneat::Genome genome = odneat::Genome::createMinimalGenome(4, 2, clock);
        genome.accessConnectionGenes().front().weight = 3.141592653589793;
        genome.setFitness(42.5);
        genome.setEvaluationCount(7);
        genome.setAdjustedFitness(21.25);

        std::string json_output{};
        std::string error_message{};
        checkCondition(odneat::toGenomeJson(genome, json_output, &error_message), "genome serialises", failure_count);
        checkCondition(!json_output.empty(), "genome json non-empty", failure_count);

        odneat::Genome restored{};
        checkCondition(odneat::fromGenomeJson(json_output, restored, &error_message), "genome parses", failure_count);
        checkCondition(genomesEqual(genome, restored), "genome round-trip identical", failure_count);
        checkCondition(restored.getFitness() == 42.5 && restored.getEvaluationCount() == 7, "genome fitness preserved", failure_count);
        checkCondition(restored.getConnectionGenes().front().weight == 3.141592653589793, "genome weight bit-exact", failure_count);
    }

    // Full-agent checkpoint round-trips every resumable field.
    {
        odneat::OdneatAgent agent = makeEvolvedAgent(9, 12345ULL);
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(9, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 999ULL);

        std::string json_output{};
        std::string error_message{};
        checkCondition(odneat::toCheckpointJson(agent, json_output, &error_message), "checkpoint serialises", failure_count);

        checkCondition(odneat::fromCheckpointJson(json_output, restored, &error_message), "checkpoint restores", failure_count);
        checkCondition(genomesEqual(agent.getActiveGenome(), restored.getActiveGenome()), "active genome identical", failure_count);
        checkCondition(agent.getPopulation().getCurrentSize() == restored.getPopulation().getCurrentSize(), "population size preserved", failure_count);
        for (std::size_t genome_index = 0; genome_index < agent.getPopulation().getCurrentSize(); ++genome_index) {
            if (!genomesEqual(agent.getPopulation().getGenomes()[genome_index], restored.getPopulation().getGenomes()[genome_index])) {
                std::cout << "FAIL: population genome identical\n";
                ++failure_count;
                break;
            }
        }
        checkCondition(agent.getTabuList().getTabuSize() == restored.getTabuList().getTabuSize(), "tabu size preserved", failure_count);
        checkCondition(agent.getTabuList().getRecentHistory().size() == restored.getTabuList().getRecentHistory().size(), "recent history preserved",
                       failure_count);
        checkCondition(agent.getEnergy() == restored.getEnergy(), "energy preserved", failure_count);
        checkCondition(agent.getFitness() == restored.getFitness() && agent.getFitnessSampleCount() == restored.getFitnessSampleCount(), "fitness preserved",
                       failure_count);
        checkCondition(agent.getMaturationCyclesRemaining() == restored.getMaturationCyclesRemaining(), "maturation preserved", failure_count);
        checkCondition(agent.getEvaluationCount() == restored.getEvaluationCount(), "evaluation count preserved", failure_count);
        checkCondition(agent.getParameters().internal_population_size == restored.getParameters().internal_population_size, "params preserved", failure_count);
        checkCondition(agent.isExchangeEnabled() == restored.isExchangeEnabled(), "flags preserved", failure_count);

        // Full mt19937_64 stream state: future draws match exactly.
        bool random_matches = true;
        for (int draw_index = 0; draw_index < 8; ++draw_index) {
            if (agent.accessRandomGenerator()() != restored.accessRandomGenerator()()) {
                random_matches = false;
                break;
            }
        }
        checkCondition(random_matches, "random stream resumes", failure_count);

        // Post-restore evolution still runs.
        const std::vector<double> sensors(4, 0.5);
        restored.executeControlCycle(sensors, 1.0, {});
        checkCondition(restored.getFitnessSampleCount() == agent.getFitnessSampleCount() + 1, "restored agent steps", failure_count);
    }

    // Pretty documents parse through the same path.
    {
        odneat::OdneatAgent agent = makeEvolvedAgent(11, 777ULL);
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(11, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 888ULL);
        std::string pretty_json{};
        std::string error_message{};
        checkCondition(odneat::toCheckpointJsonPretty(agent, pretty_json, &error_message), "pretty serialises", failure_count);
        checkCondition(odneat::fromCheckpointJson(pretty_json, restored, &error_message), "pretty parses", failure_count);
        checkCondition(genomesEqual(agent.getActiveGenome(), restored.getActiveGenome()), "pretty round-trip identical", failure_count);
    }

    // Unknown fields are ignored for forward compatibility.
    {
        odneat::OdneatAgent agent = makeEvolvedAgent(13, 555ULL);
        std::string json_output{};
        std::string error_message{};
        checkCondition(odneat::toCheckpointJson(agent, json_output, &error_message), "checkpoint serialises", failure_count);
        const std::string::size_type insert_at = json_output.find_last_of('}');
        std::string extended = json_output;
        if (insert_at != std::string::npos) {
            extended.insert(insert_at, ",\"future_field\":123");
        }
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(13, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 666ULL);
        checkCondition(odneat::fromCheckpointJson(extended, restored, &error_message), "unknown keys ignored", failure_count);
    }

    // Corrupt, version-mismatched and incompatible documents fail with diagnostics.
    {
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(13, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 666ULL);
        std::string error_message{};
        checkCondition(!odneat::fromCheckpointJson("{not json", restored, &error_message), "corrupt fails", failure_count);
        checkCondition(!error_message.empty(), "corrupt reports error", failure_count);

        odneat::OdneatAgent agent = makeEvolvedAgent(13, 555ULL);
        std::string json_output{};
        checkCondition(odneat::toCheckpointJson(agent, json_output, &error_message), "checkpoint serialises", failure_count);
        std::string bad_version = json_output;
        const std::string::size_type version_pos = bad_version.find("\"version\"");
        if (version_pos != std::string::npos) {
            const std::string::size_type colon = bad_version.find(':', version_pos);
            const std::string::size_type end = bad_version.find_first_of(",}", colon);
            if (colon != std::string::npos && end != std::string::npos) {
                bad_version.replace(colon + 1, end - colon - 1, "999");
            }
        }
        checkCondition(!odneat::fromCheckpointJson(bad_version, restored, &error_message), "bad version fails", failure_count);

        odneat::OdneatAgent other_robot(99, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 666ULL);
        checkCondition(!odneat::fromCheckpointJson(json_output, other_robot, &error_message), "robot mismatch fails", failure_count);
    }

    // File helpers round-trip through disk.
    {
        odneat::OdneatAgent agent = makeEvolvedAgent(17, 31337ULL);
        const std::filesystem::path file_path = std::filesystem::temp_directory_path() / "odneat_serialization_test.json";
        std::string error_message{};
        checkCondition(odneat::saveCheckpointToFile(file_path.string(), agent, &error_message), "checkpoint saves", failure_count);
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(17, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 4242ULL);
        checkCondition(odneat::loadCheckpointFromFile(file_path.string(), restored, &error_message), "checkpoint loads", failure_count);
        checkCondition(genomesEqual(agent.getActiveGenome(), restored.getActiveGenome()), "file round-trip identical", failure_count);
        std::error_code remove_error{};
        std::filesystem::remove(file_path, remove_error);
    }

    // Single-genome file helpers round-trip through disk.
    {
        odneat::InnovationClock clock(21);
        odneat::Genome genome = odneat::Genome::createMinimalGenome(3, 2, clock);
        genome.setFitness(7.5);
        genome.setEvaluationCount(3);
        const std::filesystem::path file_path = std::filesystem::temp_directory_path() / "odneat_genome_test.json";
        std::string error_message{};
        checkCondition(odneat::saveGenomeToFile(file_path.string(), genome, &error_message), "genome saves", failure_count);
        odneat::Genome restored{};
        checkCondition(odneat::loadGenomeFromFile(file_path.string(), restored, &error_message), "genome loads", failure_count);
        checkCondition(genomesEqual(genome, restored) && restored.getFitness() == 7.5, "genome file round-trip identical", failure_count);
        std::error_code remove_error{};
        std::filesystem::remove(file_path, remove_error);
    }

    // Genome and checkpoint documents reject malformed inputs with diagnostics.
    {
        odneat::Genome genome_output{};
        std::string error_message{};
        checkCondition(!odneat::fromGenomeJson("{not json", genome_output, &error_message), "genome corrupt fails", failure_count);
        checkCondition(!error_message.empty(), "genome corrupt reports error", failure_count);

        odneat::InnovationClock clock(23);
        odneat::Genome genome = odneat::Genome::createMinimalGenome(2, 2, clock);
        std::string json_output{};
        checkCondition(odneat::toGenomeJson(genome, json_output, &error_message), "genome serialises", failure_count);

        std::string bad_format = json_output;
        const std::string::size_type format_pos = bad_format.find(odneat::kGenomeFormat);
        if (format_pos != std::string::npos) {
            bad_format.replace(format_pos, std::string(odneat::kGenomeFormat).size(), "unknown-format");
        }
        checkCondition(!odneat::fromGenomeJson(bad_format, genome_output, &error_message), "genome bad format fails", failure_count);

        std::string bad_version = json_output;
        const std::string::size_type version_pos = bad_version.find("\"version\"");
        if (version_pos != std::string::npos) {
            const std::string::size_type colon = bad_version.find(':', version_pos);
            const std::string::size_type end = bad_version.find_first_of(",}", colon);
            if (colon != std::string::npos && end != std::string::npos) {
                bad_version.replace(colon + 1, end - colon - 1, "999");
            }
        }
        checkCondition(!odneat::fromGenomeJson(bad_version, genome_output, &error_message), "genome bad version fails", failure_count);

        std::string bad_neuron = json_output;
        const std::string::size_type neuron_pos = bad_neuron.find("\"input\"");
        if (neuron_pos != std::string::npos) {
            bad_neuron.replace(neuron_pos, std::string("\"input\"").size(), "\"cylon\"");
        }
        checkCondition(!odneat::fromGenomeJson(bad_neuron, genome_output, &error_message), "genome bad neuron fails", failure_count);

        // Empty gene lists are rejected.
        std::string empty_genes =
            R"({"format":"odneat-genome","version":1,"genome":{"neurons":[],"connections":[],"fitness":0.0,"evaluation_count":0,"adjusted_fitness":0.0}})";
        checkCondition(!odneat::fromGenomeJson(empty_genes, genome_output, &error_message), "genome empty genes fails", failure_count);

        // Checkpoint envelope rejects an unknown format too.
        odneat::OdneatAgent agent = makeEvolvedAgent(29, 555ULL);
        std::string checkpoint_json{};
        checkCondition(odneat::toCheckpointJson(agent, checkpoint_json, &error_message), "checkpoint serialises", failure_count);
        std::string bad_checkpoint_format = checkpoint_json;
        const std::string::size_type checkpoint_format_pos = bad_checkpoint_format.find(odneat::kCheckpointFormat);
        if (checkpoint_format_pos != std::string::npos) {
            bad_checkpoint_format.replace(checkpoint_format_pos, std::string(odneat::kCheckpointFormat).size(), "unknown-format");
        }
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(29, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 666ULL);
        checkCondition(!odneat::fromCheckpointJson(bad_checkpoint_format, restored, &error_message), "checkpoint bad format fails", failure_count);
    }

    // Missing files fail with diagnostics instead of crashing.
    {
        odneat::OdneatParams params{};
        params.maturation_period_cycles = 5;
        params.internal_population_size = 10;
        odneat::OdneatAgent restored(31, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 666ULL);
        odneat::Genome genome_output{};
        std::string error_message{};
        const std::string missing = (std::filesystem::temp_directory_path() / "odneat_does_not_exist_12345.json").string();
        checkCondition(!odneat::loadCheckpointFromFile(missing, restored, &error_message), "missing checkpoint fails", failure_count);
        checkCondition(!odneat::loadGenomeFromFile(missing, genome_output, &error_message), "missing genome fails", failure_count);
    }

    if (failure_count == 0) {
        std::cout << "test_serialization passed\n";
    }

    return failure_count == 0 ? 0 : 1;
}
