#include "genome.h"

#include <algorithm>
#include <iostream>

#include "config.h"
#include "innovation_clock.h"

namespace {

    int checkCondition(bool condition, const char *message, int &failure_count) {
        if (!condition) {
            std::cout << "FAIL: " << message << "\n";
            ++failure_count;
        }

        return failure_count;
    }

}  // namespace

int run_genome_suite() {
    int failure_count = 0;
    odneat::InnovationClock innovation_clock(3);
    odneat::Genome minimal_genome = odneat::Genome::createMinimalGenome(2, 2, innovation_clock);
    checkCondition(minimal_genome.getNeuronGenes().size() == 5, "minimal genome has 2 inputs + bias + 2 outputs", failure_count);
    checkCondition(minimal_genome.getConnectionGenes().size() == 6, "minimal genome fully connects inputs+bias to outputs", failure_count);
    checkCondition(minimal_genome.countEnabledConnections() == 6, "all minimal connections enabled", failure_count);
    checkCondition(minimal_genome.computeComplexity() == 11, "complexity equals neurons plus connections", failure_count);
    // Deterministic minimal: different robots share identical initial topology (distance 0).
    odneat::InnovationClock other_clock(7);
    odneat::Genome other_minimal = odneat::Genome::createMinimalGenome(2, 2, other_clock);
    checkCondition(other_minimal.isIdenticalTo(minimal_genome), "deterministic minimals match across robots", failure_count);
    odneat::Genome copied_genome = minimal_genome;
    checkCondition(copied_genome.isIdenticalTo(minimal_genome), "copy is identical", failure_count);
    copied_genome.accessConnectionGenes().front().weight = 1.5;
    checkCondition(!copied_genome.isIdenticalTo(minimal_genome), "weight change breaks identity", failure_count);
    odneat::InnovationClock second_clock(3);
    odneat::Genome second_minimal = odneat::Genome::createMinimalGenome(18, 2, second_clock);
    checkCondition(second_minimal.getNeuronGenes().size() == 21, "aggregation layout has 18+1+2 neurons", failure_count);
    checkCondition(second_minimal.getConnectionGenes().size() == 38, "aggregation layout fully connected with bias", failure_count);
    // findNeuron hit/miss (mutable + const paths).
    const odneat::InnovationId known_id = minimal_genome.getNeuronGenes().front().innovation_id;
    checkCondition(minimal_genome.findNeuron(known_id) != nullptr, "findNeuron hits known id", failure_count);
    checkCondition(minimal_genome.findNeuron(odneat::InnovationId{9999, 9999, 9999}) == nullptr, "findNeuron misses unknown id", failure_count);
    checkCondition(minimal_genome.findNeuronMutable(known_id) != nullptr, "findNeuronMutable hits known id", failure_count);
    // sortGenesChronologically orders out-of-order inserts.
    odneat::Genome shuffled = minimal_genome;
    std::reverse(shuffled.accessConnectionGenes().begin(), shuffled.accessConnectionGenes().end());
    shuffled.sortGenesChronologically();
    {
        bool sorted = true;
        const auto &conns = shuffled.getConnectionGenes();
        for (std::size_t i = 1; i < conns.size(); ++i) {
            if (conns[i].innovation_id < conns[i - 1].innovation_id) {
                sorted = false;
                break;
            }
        }

        checkCondition(sorted, "connections sorted chronologically", failure_count);
    }

    // Enabled/hidden counts exclude disabled and non-hidden types (bias not counted as hidden).
    odneat::Genome counted = minimal_genome;
    counted.accessConnectionGenes().front().enabled = false;
    checkCondition(counted.countEnabledConnections() + 1 == minimal_genome.countEnabledConnections(), "disabled excluded from enabled count", failure_count);
    checkCondition(minimal_genome.countHiddenNeurons() == 0, "minimal has no hidden neurons", failure_count);
    checkCondition(second_minimal.countHiddenNeurons() == 0, "bias excluded from hidden count", failure_count);
    // Size mismatch breaks identity (distinct from weight-change path).
    odneat::Genome extended = minimal_genome;
    extended.addNeuronGene(minimal_genome.getNeuronGenes().front());
    checkCondition(!extended.isIdenticalTo(minimal_genome), "neuron count mismatch breaks identity", failure_count);

    if (failure_count == 0) {
        std::cout << "test_genome passed\n";
    }

    return failure_count == 0 ? 0 : 1;
}
