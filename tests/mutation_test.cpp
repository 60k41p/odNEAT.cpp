#include "mutation.h"

#include <iostream>
#include <random>

#include "config.h"
#include "genome.h"
#include "innovation_clock.h"

int run_mutation_suite() {
    int failure_count = 0;
    odneat::InnovationClock innovation_clock(5);
    odneat::Genome genome = odneat::Genome::createMinimalGenome(3, 2, innovation_clock);
    const std::size_t initial_connections = genome.getConnectionGenes().size();
    const std::size_t initial_neurons = genome.getNeuronGenes().size();
    std::mt19937_64 randomGenerator(123ULL);
    odneat::OdneatParams parameters{};
    parameters.weight_perturb_rate = 1.0;
    parameters.add_connection_rate = 1.0;
    parameters.add_neuron_rate = 1.0;
    odneat::MutationOutcome outcome{};
    odneat::mutateGenome(genome, innovation_clock, randomGenerator, parameters, &outcome);

    if (outcome.perturbed_weights == 0) {
        std::cout << "FAIL: weights not perturbed\n";
        ++failure_count;
    }

    if (!outcome.added_connection && genome.getConnectionGenes().size() <= initial_connections) {
        std::cout << "FAIL: connection not added\n";
        ++failure_count;
    }

    if (!outcome.added_neuron && genome.getNeuronGenes().size() <= initial_neurons) {
        std::cout << "FAIL: neuron not added\n";
        ++failure_count;
    }

    if (genome.countHiddenNeurons() == 0) {
        std::cout << "FAIL: expected hidden neuron\n";
        ++failure_count;
    }

    if (outcome.perturbed_biases != 0) {
        std::cout << "FAIL: bias model has no per-neuron perturbation\n";
        ++failure_count;
    }

    // Cloned offspring is unevaluated even though the parent was evaluated.
    {
        odneat::InnovationClock clock(4);
        odneat::Genome parent = odneat::Genome::createMinimalGenome(2, 1, clock);
        parent.setFitness(80.0);
        parent.setEvaluationCount(5);
        odneat::OdneatParams params{};
        params.crossover_rate = 0.0;
        params.add_connection_rate = 0.0;
        params.add_neuron_rate = 0.0;
        params.toggle_enabled_rate = 0.0;
        std::mt19937_64 rng(41ULL);
        const odneat::Genome child = odneat::generateOffspring(parent, &parent, clock, rng, params);
        if (child.getFitness() != 0.0 || child.getEvaluationCount() != 0) {
            std::cout << "FAIL: clone offspring unevaluated\n";
            ++failure_count;
        }
    }

    // Enable-bit toggling defaults off (absent from both papers); explicit opt-in still flips.
    {
        if (odneat::OdneatParams{}.toggle_enabled_rate != 0.0) {
            std::cout << "FAIL: toggle defaults off\n";
            ++failure_count;
        }

        odneat::InnovationClock clock(12);
        odneat::Genome g = odneat::Genome::createMinimalGenome(2, 1, clock);
        odneat::OdneatParams params{};
        params.weight_perturb_rate = 0.0;
        params.add_connection_rate = 0.0;
        params.add_neuron_rate = 0.0;
        params.toggle_enabled_rate = 1.0;
        std::mt19937_64 rng(43ULL);
        odneat::MutationOutcome toggle_outcome{};
        odneat::mutateGenome(g, clock, rng, params, &toggle_outcome);
        if (!toggle_outcome.toggled_enabled) {
            std::cout << "FAIL: opt-in toggle flips\n";
            ++failure_count;
        }
    }

    // Weight clipping holds at both bounds under forced perturbation.
    {
        odneat::InnovationClock clock(6);
        odneat::Genome g = odneat::Genome::createMinimalGenome(2, 1, clock);
        for (auto &cg : g.accessConnectionGenes()) cg.weight = 9.9;
        odneat::OdneatParams params{};
        params.weight_perturb_rate = 1.0;
        params.weight_mutation_magnitude = 5.0;
        params.add_connection_rate = 0.0;
        params.add_neuron_rate = 0.0;
        params.toggle_enabled_rate = 0.0;
        std::mt19937_64 rng(5ULL);
        odneat::mutateGenome(g, clock, rng, params, nullptr);
        for (const auto &cg : g.getConnectionGenes()) {
            if (cg.weight < -10.0 || cg.weight > 10.0) {
                std::cout << "FAIL: weight clipping\n";
                ++failure_count;
                break;
            }
        }
    }

    // Structural split invariants: old disabled, in-weight 1.0, out-weight preserved.
    {
        odneat::InnovationClock clock(8);
        odneat::Genome g = odneat::Genome::createMinimalGenome(2, 1, clock);
        const double original = g.getConnectionGenes().front().weight;
        const odneat::InnovationId src = g.getConnectionGenes().front().input_neuron_id;
        const odneat::InnovationId dst = g.getConnectionGenes().front().output_neuron_id;
        std::mt19937_64 rng(9ULL);
        // Force the split onto connection 0 by disabling all others first.
        for (std::size_t i = 1; i < g.accessConnectionGenes().size(); ++i) g.accessConnectionGenes()[i].enabled = false;
        if (!odneat::addRandomNeuron(g, clock, rng)) {
            std::cout << "FAIL: split failed\n";
            ++failure_count;
        } else {
            bool old_off = false, in_one = false, out_kept = false;
            for (const auto &cg : g.getConnectionGenes()) {
                if (cg.input_neuron_id == src && cg.output_neuron_id == dst && !cg.enabled) old_off = true;
                if (cg.output_neuron_id != dst && cg.weight == 1.0 && cg.enabled) in_one = true;
                if (cg.input_neuron_id != src && cg.output_neuron_id == dst && cg.weight == original && cg.enabled) out_kept = true;
            }

            if (!old_off || !in_one || !out_kept) {
                std::cout << "FAIL: split invariants\n";
                ++failure_count;
            }

            if (g.countHiddenNeurons() != 1) {
                std::cout << "FAIL: split adds one hidden\n";
                ++failure_count;
            }
        }
    }

    // Failure edges: too few neurons, and no enabled connection to split.
    {
        odneat::InnovationClock clock(11);
        odneat::Genome tiny;
        odneat::NeuronGene only{};
        only.innovation_id = clock.nextInnovationId();
        tiny.addNeuronGene(only);
        std::mt19937_64 rng(13ULL);
        if (odneat::addRandomConnection(tiny, clock, rng, -10.0, 10.0)) {
            std::cout << "FAIL: single neuron cannot connect\n";
            ++failure_count;
        }

        odneat::Genome locked = odneat::Genome::createMinimalGenome(2, 1, clock);
        for (auto &cg : locked.accessConnectionGenes()) cg.enabled = false;
        if (odneat::addRandomNeuron(locked, clock, rng)) {
            std::cout << "FAIL: no enabled edge to split\n";
            ++failure_count;
        }
    }

    // Unrestricted recurrent still forbids dead sinks into inputs/bias.
    {
        odneat::InnovationClock clock(14);
        odneat::Genome g = odneat::Genome::createMinimalGenome(3, 2, clock);
        std::mt19937_64 rng(17ULL);
        bool bad_sink = false;
        for (int i = 0; i < 200; ++i) {
            odneat::Genome trial = g;
            if (odneat::addRandomConnection(trial, clock, rng, -10.0, 10.0)) {
                const auto &nc = trial.getConnectionGenes().back();
                const odneat::NeuronGene *tgt = trial.findNeuron(nc.output_neuron_id);
                if (tgt != nullptr && (tgt->neuron_type == odneat::NeuronType::kInput || tgt->neuron_type == odneat::NeuronType::kBias)) {
                    bad_sink = true;
                    break;
                }
            }
        }

        if (bad_sink) {
            std::cout << "FAIL: input/bias sink added\n";
            ++failure_count;
        }
    }

    if (failure_count == 0) {
        std::cout << "test_mutation passed\n";
    }

    return failure_count == 0 ? 0 : 1;
}
