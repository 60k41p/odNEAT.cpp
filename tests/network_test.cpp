#include "network.h"

#include <iostream>
#include <vector>

#include "genome.h"
#include "innovation_clock.h"
int run_network_suite() {
    int f = 0;
    odneat::InnovationClock c(0);
    odneat::Genome g = odneat::Genome::createMinimalGenome(2, 2, c);
    odneat::RecurrentNetwork n = odneat::RecurrentNetwork::decodeFromGenome(g, 2, 2);
    std::vector<double> inp{1.0, 0.0};
    std::vector<double> out = n.stepNetwork(inp);

    if (out.size() != 2) {
        std::cout << "FAIL: output size\n";
        ++f;
    }

    for (double v : out) {
        if (v < 0.0 || v > 1.0) {
            std::cout << "FAIL: output range\n";
            ++f;
            break;
        }
    }

    if (odneat::rescaleOutputToWheelSpeed(0.0) != -1.0 || odneat::rescaleOutputToWheelSpeed(1.0) != 1.0) {
        std::cout << "FAIL: rescale\n";
        ++f;
    }

    if (odneat::computeLogisticActivation(0.0, 0.0, 1.0) != 0.5) {
        std::cout << "FAIL: logistic\n";
        ++f;
    }

    // Bias neuron drives outputs with constant 1.0 regardless of sensor inputs.
    {
        odneat::InnovationClock c2(1);
        odneat::Genome g2 = odneat::Genome::createMinimalGenome(2, 1, c2);
        for (auto &cg : g2.accessConnectionGenes()) {
            const odneat::NeuronGene *src = g2.findNeuron(cg.input_neuron_id);
            cg.weight = (src != nullptr && src->neuron_type == odneat::NeuronType::kBias) ? 5.0 : 0.0;
        }

        odneat::RecurrentNetwork n2 = odneat::RecurrentNetwork::decodeFromGenome(g2, 2, 1);
        const std::vector<double> low = n2.stepNetwork({0.0, 0.0});
        n2.resetActivations();
        const std::vector<double> high = n2.stepNetwork({1.0, 1.0});
        if (low.size() != 1 || high.size() != 1 || low[0] < 0.9 || high[0] < 0.9) {
            std::cout << "FAIL: bias constant drive\n";
            ++f;
        }
    }

    // Disabled edges are ignored: all-disabled genome yields neutral 0.5 outputs.
    {
        odneat::InnovationClock c3(2);
        odneat::Genome g3 = odneat::Genome::createMinimalGenome(2, 2, c3);
        for (auto &cg : g3.accessConnectionGenes()) cg.enabled = false;
        odneat::RecurrentNetwork n3 = odneat::RecurrentNetwork::decodeFromGenome(g3, 2, 2);
        const std::vector<double> out3 = n3.stepNetwork({1.0, 1.0});
        if (out3[0] != 0.5 || out3[1] != 0.5) {
            std::cout << "FAIL: disabled ignored\n";
            ++f;
        }
    }

    // Dangling references and missing outputs fall back safely instead of crashing.
    {
        odneat::Genome dangling;
        odneat::ConnectionGene bad{};
        bad.innovation_id = odneat::InnovationId{0, 1, 1};
        bad.input_neuron_id = odneat::InnovationId{0, 2, 2};
        bad.output_neuron_id = odneat::InnovationId{0, 3, 3};
        bad.weight = 3.0;
        dangling.addConnectionGene(bad);
        odneat::RecurrentNetwork nd = odneat::RecurrentNetwork::decodeFromGenome(dangling, 1, 2);
        const std::vector<double> outd = nd.stepNetwork({1.0});
        if (outd.size() != 2 || outd[0] != 0.5 || outd[1] != 0.5) {
            std::cout << "FAIL: dangling/missing fallback\n";
            ++f;
        }
    }

    // Reset clears hidden/output but restores bias to 1.0.
    {
        odneat::InnovationClock c4(4);
        odneat::Genome g4 = odneat::Genome::createMinimalGenome(1, 1, c4);
        odneat::RecurrentNetwork n4 = odneat::RecurrentNetwork::decodeFromGenome(g4, 1, 1);
        (void)n4.stepNetwork({1.0});
        n4.resetActivations();
        bool bias_one = false, hidden_zero = true;
        for (const auto &[id, act] : n4.getActivations()) {
            const odneat::NeuronGene *ng = g4.findNeuron(id);
            if (ng != nullptr && ng->neuron_type == odneat::NeuronType::kBias) {
                if (act == 1.0) bias_one = true;
            }

            if (ng != nullptr && ng->neuron_type == odneat::NeuronType::kOutput) {
                if (act != 0.0) hidden_zero = false;
            }
        }

        if (!bias_one || !hidden_zero) {
            std::cout << "FAIL: reset semantics\n";
            ++f;
        }
    }

    // Out-of-range sensor inputs are clamped, outputs stay in range.
    {
        odneat::InnovationClock c5(5);
        odneat::Genome g5 = odneat::Genome::createMinimalGenome(2, 2, c5);
        odneat::RecurrentNetwork n5 = odneat::RecurrentNetwork::decodeFromGenome(g5, 2, 2);
        const std::vector<double> out5 = n5.stepNetwork({-5.0, 99.0});
        for (double v : out5) {
            if (v < 0.0 || v > 1.0) {
                std::cout << "FAIL: clamp range\n";
                ++f;
                break;
            }
        }

        if (odneat::clampSensorInput(-1.0) != 0.0 || odneat::clampSensorInput(2.0) != 1.0) {
            std::cout << "FAIL: clamp edges\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_network passed\n";

    return f == 0 ? 0 : 1;
}
