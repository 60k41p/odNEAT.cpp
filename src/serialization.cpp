#include "serialization.h"

#include <cstdint>
#include <fstream>
#include <glaze/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "agent.h"
#include "config.h"
#include "genome.h"
#include "innovation_clock.h"

namespace odneat {
    namespace checkpoint_dto {

        struct InnovationIdDto {
            std::uint32_t robot_identifier = 0;
            std::uint64_t timestamp_nanoseconds = 0;
            std::uint32_t local_counter = 0;
        };

        struct NeuronDto {
            InnovationIdDto innovation_id{};
            std::string neuron_type{};
            int input_index = -1;
            int output_index = -1;
        };

        struct ConnectionDto {
            InnovationIdDto innovation_id{};
            InnovationIdDto input_neuron_id{};
            InnovationIdDto output_neuron_id{};
            double weight = 0.0;
            bool enabled = true;
        };

        struct GenomeDto {
            std::vector<NeuronDto> neurons{};
            std::vector<ConnectionDto> connections{};
            double fitness = 0.0;
            int evaluation_count = 0;
            double adjusted_fitness = 0.0;
        };

        struct GenomeDocDto {
            std::string format{};
            int version = 0;
            GenomeDto genome{};
        };

        struct ParamsDto {
            double crossover_rate = 0.0;
            double weight_perturb_rate = 0.0;
            double weight_mutation_magnitude = 0.0;
            double add_connection_rate = 0.0;
            double add_neuron_rate = 0.0;
            double toggle_enabled_rate = 0.0;
            double disabled_inheritance_rate = 0.0;
            double compatibility_threshold = 0.0;
            double disjoint_coefficient = 0.0;
            double excess_coefficient = 0.0;
            double weight_difference_coefficient = 0.0;
            int internal_population_size = 0;
            int maturation_period_cycles = 0;
            int tabu_expiry_history_size = 0;
            double control_cycle_seconds = 0.0;
            double weight_minimum = 0.0;
            double weight_maximum = 0.0;
        };

        struct CheckpointDto {
            std::string format{};
            int version = 0;
            std::uint32_t robot_identifier = 0;
            int input_count = 0;
            int output_count = 0;
            ParamsDto parameters{};
            GenomeDto active{};
            std::vector<GenomeDto> population{};
            std::vector<GenomeDto> tabu{};
            std::vector<GenomeDto> recent_history{};
            double minimum_energy = 0.0;
            double maximum_energy = 0.0;
            double default_energy = 0.0;
            double minimum_threshold = 0.0;
            double current_energy = 0.0;
            double fitness_mean = 0.0;
            int fitness_sample_count = 0;
            std::uint32_t clock_minted_count = 0;
            std::uint64_t clock_last_timestamp = 0;
            std::string random_state{};
            int maturation_cycles_remaining = 0;
            int evaluation_count = 0;
            bool exchange_enabled = true;
            bool tabu_enabled = true;
            bool maturation_enabled = true;
            bool speciation_enabled = true;
        };

    }  // namespace checkpoint_dto

    namespace {

        void setError(std::string *error_message, const std::string &message) {
            if (error_message != nullptr) {
                *error_message = message;
            }
        }

        checkpoint_dto::InnovationIdDto toIdDto(const InnovationId &innovation_id) {
            checkpoint_dto::InnovationIdDto dto{};
            dto.robot_identifier = innovation_id.robot_identifier;
            dto.timestamp_nanoseconds = innovation_id.timestamp_nanoseconds;
            dto.local_counter = innovation_id.local_counter;
            return dto;
        }

        InnovationId fromIdDto(const checkpoint_dto::InnovationIdDto &dto) {
            InnovationId innovation_id{};
            innovation_id.robot_identifier = dto.robot_identifier;
            innovation_id.timestamp_nanoseconds = dto.timestamp_nanoseconds;
            innovation_id.local_counter = dto.local_counter;
            return innovation_id;
        }

        std::string neuronTypeToString(NeuronType neuron_type) {
            switch (neuron_type) {
                case NeuronType::kInput:
                    return "input";
                case NeuronType::kHidden:
                    return "hidden";
                case NeuronType::kOutput:
                    return "output";
                case NeuronType::kBias:
                    return "bias";
            }
            return "hidden";
        }

        bool neuronTypeFromString(const std::string &name, NeuronType &neuron_type) {
            if (name == "input") {
                neuron_type = NeuronType::kInput;
                return true;
            }
            if (name == "hidden") {
                neuron_type = NeuronType::kHidden;
                return true;
            }
            if (name == "output") {
                neuron_type = NeuronType::kOutput;
                return true;
            }
            if (name == "bias") {
                neuron_type = NeuronType::kBias;
                return true;
            }
            return false;
        }

        checkpoint_dto::GenomeDto toGenomeDto(const Genome &genome) {
            checkpoint_dto::GenomeDto dto{};
            const std::vector<NeuronGene> &neurons = genome.getNeuronGenes();
            dto.neurons.reserve(neurons.size());
            for (const NeuronGene &neuron_gene : neurons) {
                checkpoint_dto::NeuronDto neuron_dto{};
                neuron_dto.innovation_id = toIdDto(neuron_gene.innovation_id);
                neuron_dto.neuron_type = neuronTypeToString(neuron_gene.neuron_type);
                neuron_dto.input_index = neuron_gene.input_index;
                neuron_dto.output_index = neuron_gene.output_index;
                dto.neurons.push_back(neuron_dto);
            }
            const std::vector<ConnectionGene> &connections = genome.getConnectionGenes();
            dto.connections.reserve(connections.size());
            for (const ConnectionGene &connection_gene : connections) {
                checkpoint_dto::ConnectionDto connection_dto{};
                connection_dto.innovation_id = toIdDto(connection_gene.innovation_id);
                connection_dto.input_neuron_id = toIdDto(connection_gene.input_neuron_id);
                connection_dto.output_neuron_id = toIdDto(connection_gene.output_neuron_id);
                connection_dto.weight = connection_gene.weight;
                connection_dto.enabled = connection_gene.enabled;
                dto.connections.push_back(connection_dto);
            }
            dto.fitness = genome.getFitness();
            dto.evaluation_count = genome.getEvaluationCount();
            dto.adjusted_fitness = genome.getAdjustedFitness();
            return dto;
        }

        bool fromGenomeDto(const checkpoint_dto::GenomeDto &dto, Genome &genome, std::string *error_message) {
            if (dto.neurons.empty() || dto.connections.empty()) {
                setError(error_message, "genome has no genes");
                return false;
            }
            Genome parsed{};
            for (const checkpoint_dto::NeuronDto &neuron_dto : dto.neurons) {
                NeuronGene neuron_gene{};
                neuron_gene.innovation_id = fromIdDto(neuron_dto.innovation_id);
                if (!neuronTypeFromString(neuron_dto.neuron_type, neuron_gene.neuron_type)) {
                    setError(error_message, "unknown neuron type: " + neuron_dto.neuron_type);
                    return false;
                }
                neuron_gene.input_index = neuron_dto.input_index;
                neuron_gene.output_index = neuron_dto.output_index;
                parsed.addNeuronGene(neuron_gene);
            }
            for (const checkpoint_dto::ConnectionDto &connection_dto : dto.connections) {
                ConnectionGene connection_gene{};
                connection_gene.innovation_id = fromIdDto(connection_dto.innovation_id);
                connection_gene.input_neuron_id = fromIdDto(connection_dto.input_neuron_id);
                connection_gene.output_neuron_id = fromIdDto(connection_dto.output_neuron_id);
                connection_gene.weight = connection_dto.weight;
                connection_gene.enabled = connection_dto.enabled;
                parsed.addConnectionGene(connection_gene);
            }
            parsed.sortGenesChronologically();
            parsed.setFitness(dto.fitness);
            parsed.setEvaluationCount(dto.evaluation_count);
            parsed.setAdjustedFitness(dto.adjusted_fitness);
            genome = parsed;
            return true;
        }

        checkpoint_dto::ParamsDto toParamsDto(const OdneatParams &parameters) {
            checkpoint_dto::ParamsDto dto{};
            dto.crossover_rate = parameters.crossover_rate;
            dto.weight_perturb_rate = parameters.weight_perturb_rate;
            dto.weight_mutation_magnitude = parameters.weight_mutation_magnitude;
            dto.add_connection_rate = parameters.add_connection_rate;
            dto.add_neuron_rate = parameters.add_neuron_rate;
            dto.toggle_enabled_rate = parameters.toggle_enabled_rate;
            dto.disabled_inheritance_rate = parameters.disabled_inheritance_rate;
            dto.compatibility_threshold = parameters.compatibility_threshold;
            dto.disjoint_coefficient = parameters.disjoint_coefficient;
            dto.excess_coefficient = parameters.excess_coefficient;
            dto.weight_difference_coefficient = parameters.weight_difference_coefficient;
            dto.internal_population_size = parameters.internal_population_size;
            dto.maturation_period_cycles = parameters.maturation_period_cycles;
            dto.tabu_expiry_history_size = parameters.tabu_expiry_history_size;
            dto.control_cycle_seconds = parameters.control_cycle_seconds;
            dto.weight_minimum = parameters.weight_minimum;
            dto.weight_maximum = parameters.weight_maximum;
            return dto;
        }

        OdneatParams fromParamsDto(const checkpoint_dto::ParamsDto &dto) {
            OdneatParams parameters{};
            parameters.crossover_rate = dto.crossover_rate;
            parameters.weight_perturb_rate = dto.weight_perturb_rate;
            parameters.weight_mutation_magnitude = dto.weight_mutation_magnitude;
            parameters.add_connection_rate = dto.add_connection_rate;
            parameters.add_neuron_rate = dto.add_neuron_rate;
            parameters.toggle_enabled_rate = dto.toggle_enabled_rate;
            parameters.disabled_inheritance_rate = dto.disabled_inheritance_rate;
            parameters.compatibility_threshold = dto.compatibility_threshold;
            parameters.disjoint_coefficient = dto.disjoint_coefficient;
            parameters.excess_coefficient = dto.excess_coefficient;
            parameters.weight_difference_coefficient = dto.weight_difference_coefficient;
            parameters.internal_population_size = dto.internal_population_size;
            parameters.maturation_period_cycles = dto.maturation_period_cycles;
            parameters.tabu_expiry_history_size = dto.tabu_expiry_history_size;
            parameters.control_cycle_seconds = dto.control_cycle_seconds;
            parameters.weight_minimum = dto.weight_minimum;
            parameters.weight_maximum = dto.weight_maximum;
            return parameters;
        }

        checkpoint_dto::CheckpointDto toCheckpointDto(const AgentCheckpointState &state) {
            checkpoint_dto::CheckpointDto dto{};
            dto.format = kCheckpointFormat;
            dto.version = kCheckpointVersion;
            dto.robot_identifier = state.robot_identifier;
            dto.input_count = state.input_count;
            dto.output_count = state.output_count;
            dto.parameters = toParamsDto(state.parameters);
            dto.active = toGenomeDto(state.active_genome);
            dto.population.reserve(state.population_genomes.size());
            for (const Genome &genome : state.population_genomes) {
                dto.population.push_back(toGenomeDto(genome));
            }
            dto.tabu.reserve(state.tabu_genomes.size());
            for (const Genome &genome : state.tabu_genomes) {
                dto.tabu.push_back(toGenomeDto(genome));
            }
            dto.recent_history.reserve(state.recent_history.size());
            for (const Genome &genome : state.recent_history) {
                dto.recent_history.push_back(toGenomeDto(genome));
            }
            dto.minimum_energy = state.minimum_energy;
            dto.maximum_energy = state.maximum_energy;
            dto.default_energy = state.default_energy;
            dto.minimum_threshold = state.minimum_threshold;
            dto.current_energy = state.current_energy;
            dto.fitness_mean = state.fitness_mean;
            dto.fitness_sample_count = state.fitness_sample_count;
            dto.clock_minted_count = state.clock_minted_count;
            dto.clock_last_timestamp = state.clock_last_timestamp;
            dto.random_state = state.random_state;
            dto.maturation_cycles_remaining = state.maturation_cycles_remaining;
            dto.evaluation_count = state.evaluation_count;
            dto.exchange_enabled = state.exchange_enabled;
            dto.tabu_enabled = state.tabu_enabled;
            dto.maturation_enabled = state.maturation_enabled;
            dto.speciation_enabled = state.speciation_enabled;
            return dto;
        }

        bool fromCheckpointDto(const checkpoint_dto::CheckpointDto &dto, AgentCheckpointState &state, std::string *error_message) {
            if (dto.format != kCheckpointFormat) {
                setError(error_message, "unknown checkpoint format: " + dto.format);
                return false;
            }
            if (dto.version != kCheckpointVersion) {
                setError(error_message, "unsupported checkpoint version");
                return false;
            }
            AgentCheckpointState parsed{};
            parsed.robot_identifier = dto.robot_identifier;
            parsed.input_count = dto.input_count;
            parsed.output_count = dto.output_count;
            parsed.parameters = fromParamsDto(dto.parameters);
            if (!fromGenomeDto(dto.active, parsed.active_genome, error_message)) {
                return false;
            }
            parsed.population_genomes.reserve(dto.population.size());
            for (const checkpoint_dto::GenomeDto &genome_dto : dto.population) {
                Genome genome{};
                if (!fromGenomeDto(genome_dto, genome, error_message)) {
                    return false;
                }
                parsed.population_genomes.push_back(std::move(genome));
            }
            parsed.tabu_genomes.reserve(dto.tabu.size());
            for (const checkpoint_dto::GenomeDto &genome_dto : dto.tabu) {
                Genome genome{};
                if (!fromGenomeDto(genome_dto, genome, error_message)) {
                    return false;
                }
                parsed.tabu_genomes.push_back(std::move(genome));
            }
            parsed.recent_history.reserve(dto.recent_history.size());
            for (const checkpoint_dto::GenomeDto &genome_dto : dto.recent_history) {
                Genome genome{};
                if (!fromGenomeDto(genome_dto, genome, error_message)) {
                    return false;
                }
                parsed.recent_history.push_back(std::move(genome));
            }
            parsed.minimum_energy = dto.minimum_energy;
            parsed.maximum_energy = dto.maximum_energy;
            parsed.default_energy = dto.default_energy;
            parsed.minimum_threshold = dto.minimum_threshold;
            parsed.current_energy = dto.current_energy;
            parsed.fitness_mean = dto.fitness_mean;
            parsed.fitness_sample_count = dto.fitness_sample_count;
            parsed.clock_minted_count = dto.clock_minted_count;
            parsed.clock_last_timestamp = dto.clock_last_timestamp;
            parsed.random_state = dto.random_state;
            parsed.maturation_cycles_remaining = dto.maturation_cycles_remaining;
            parsed.evaluation_count = dto.evaluation_count;
            parsed.exchange_enabled = dto.exchange_enabled;
            parsed.tabu_enabled = dto.tabu_enabled;
            parsed.maturation_enabled = dto.maturation_enabled;
            parsed.speciation_enabled = dto.speciation_enabled;
            state = std::move(parsed);
            return true;
        }

        bool writeCheckpointDto(const checkpoint_dto::CheckpointDto &dto, std::string &json_output, bool pretty, std::string *error_message) {
            json_output.clear();
            glz::error_ctx write_error{};
            if (pretty) {
                write_error = glz::write<glz::opts{.prettify = true}>(dto, json_output);
            } else {
                write_error = glz::write_json(dto, json_output);
            }
            if (write_error) {
                json_output.clear();
                setError(error_message, "failed to serialise checkpoint");
                return false;
            }
            return true;
        }

    }  // namespace

    bool toGenomeJson(const Genome &genome, std::string &json_output, std::string *error_message) {
        checkpoint_dto::GenomeDocDto document{};
        document.format = kGenomeFormat;
        document.version = kGenomeVersion;
        document.genome = toGenomeDto(genome);
        json_output.clear();
        const glz::error_ctx write_error = glz::write_json(document, json_output);
        if (write_error) {
            json_output.clear();
            setError(error_message, "failed to serialise genome");
            return false;
        }
        return true;
    }

    bool fromGenomeJson(std::string_view json_input, Genome &genome_output, std::string *error_message) {
        checkpoint_dto::GenomeDocDto document{};
        const std::string buffer{json_input};
        const glz::error_ctx parse_error = glz::read<glz::opts{.error_on_unknown_keys = false}>(document, buffer);
        if (parse_error) {
            setError(error_message, glz::format_error(parse_error, buffer));
            return false;
        }
        if (document.format != kGenomeFormat) {
            setError(error_message, "unknown genome format: " + document.format);
            return false;
        }
        if (document.version != kGenomeVersion) {
            setError(error_message, "unsupported genome version");
            return false;
        }
        Genome parsed{};
        if (!fromGenomeDto(document.genome, parsed, error_message)) {
            return false;
        }
        genome_output = parsed;
        return true;
    }

    bool saveGenomeToFile(const std::string &file_path, const Genome &genome, std::string *error_message) {
        std::string json_output{};
        if (!toGenomeJson(genome, json_output, error_message)) {
            return false;
        }
        std::ofstream output_stream{file_path, std::ios::binary | std::ios::trunc};
        if (!output_stream) {
            setError(error_message, "cannot open file for writing: " + file_path);
            return false;
        }
        output_stream << json_output;
        output_stream.flush();
        if (!output_stream) {
            setError(error_message, "failed to write file: " + file_path);
            return false;
        }
        return true;
    }

    bool loadGenomeFromFile(const std::string &file_path, Genome &genome_output, std::string *error_message) {
        std::ifstream input_stream{file_path, std::ios::binary};
        if (!input_stream) {
            setError(error_message, "cannot open file for reading: " + file_path);
            return false;
        }
        std::ostringstream content_stream{};
        content_stream << input_stream.rdbuf();
        if (input_stream.bad()) {
            setError(error_message, "failed to read file: " + file_path);
            return false;
        }
        return fromGenomeJson(content_stream.str(), genome_output, error_message);
    }

    bool toCheckpointJson(const OdneatAgent &agent, std::string &json_output, std::string *error_message) {
        return writeCheckpointDto(toCheckpointDto(agent.exportCheckpointState()), json_output, false, error_message);
    }

    bool toCheckpointJsonPretty(const OdneatAgent &agent, std::string &json_output, std::string *error_message) {
        return writeCheckpointDto(toCheckpointDto(agent.exportCheckpointState()), json_output, true, error_message);
    }

    bool fromCheckpointJson(std::string_view json_input, OdneatAgent &agent_output, std::string *error_message) {
        const std::string buffer{json_input};
        checkpoint_dto::CheckpointDto dto{};
        const glz::error_ctx parse_error = glz::read<glz::opts{.error_on_unknown_keys = false}>(dto, buffer);
        if (parse_error) {
            setError(error_message, glz::format_error(parse_error, buffer));
            return false;
        }
        AgentCheckpointState state{};
        if (!fromCheckpointDto(dto, state, error_message)) {
            return false;
        }
        if (!agent_output.restoreCheckpointState(state, error_message)) {
            return false;
        }
        return true;
    }

    bool saveCheckpointToFile(const std::string &file_path, const OdneatAgent &agent, std::string *error_message) {
        std::string json_output{};
        if (!toCheckpointJson(agent, json_output, error_message)) {
            return false;
        }
        std::ofstream output_stream{file_path, std::ios::binary | std::ios::trunc};
        if (!output_stream) {
            setError(error_message, "cannot open file for writing: " + file_path);
            return false;
        }
        output_stream << json_output;
        output_stream.flush();
        if (!output_stream) {
            setError(error_message, "failed to write file: " + file_path);
            return false;
        }
        return true;
    }

    bool loadCheckpointFromFile(const std::string &file_path, OdneatAgent &agent_output, std::string *error_message) {
        std::ifstream input_stream{file_path, std::ios::binary};
        if (!input_stream) {
            setError(error_message, "cannot open file for reading: " + file_path);
            return false;
        }
        std::ostringstream content_stream{};
        content_stream << input_stream.rdbuf();
        if (input_stream.bad()) {
            setError(error_message, "failed to read file: " + file_path);
            return false;
        }
        return fromCheckpointJson(content_stream.str(), agent_output, error_message);
    }

}  // namespace odneat
