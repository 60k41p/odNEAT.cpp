#include "compatibility.h"

#include <iostream>
#include <vector>

#include "config.h"
#include "genome.h"
#include "innovation_clock.h"

int run_compatibility_suite() {
    int failure_count = 0;
    auto report_failure = [&](bool condition, const char *message) {
        if (!condition) {
            std::cout << "FAIL: " << message << "\n";
            ++failure_count;
        }
    };

    odneat::InnovationClock innovation_clock(0);
    odneat::Genome first_genome = odneat::Genome::createMinimalGenome(3, 2, innovation_clock);
    odneat::Genome second_genome = first_genome;
    const double identical_distance = odneat::computeCompatibilityDistance(first_genome, second_genome, 1.0, 1.0, 0.4);
    report_failure(identical_distance == 0.0, "identical genomes have zero distance");
    second_genome.accessConnectionGenes().front().weight += 2.0;
    const double weight_distance = odneat::computeCompatibilityDistance(first_genome, second_genome, 1.0, 1.0, 0.4);
    report_failure(weight_distance > 0.0, "weight change increases distance");
    odneat::OdneatParams default_params{};
    report_failure(odneat::areGenomesCompatible(first_genome, second_genome, default_params.disjoint_coefficient, default_params.excess_coefficient,
                                                default_params.weight_difference_coefficient, default_params.compatibility_threshold),
                   "small weight change stays compatible");
    std::vector<odneat::Genome> population_genomes{first_genome, second_genome, first_genome};
    const std::vector<odneat::Species> species_list = odneat::speciatePopulation(population_genomes, 1.0, 1.0, 0.4, 3.0);
    report_failure(!species_list.empty(), "speciation yields species");
    // Empty genomes have zero distance (no matching/disjoint/excess to compare).
    {
        odneat::Genome empty_a, empty_b;
        const odneat::CompatibilityBreakdown bd = odneat::analyseCompatibility(empty_a, empty_b, 1.0, 1.0, 0.4);
        report_failure(bd.distance == 0.0 && bd.normaliser == 1.0, "empty genomes zero distance");
    }

    // Threshold is strict <: distance exactly at threshold is incompatible.
    {
        odneat::Genome a, b;
        const double d = odneat::computeCompatibilityDistance(a, b, 1.0, 1.0, 0.4);
        report_failure(!odneat::areGenomesCompatible(a, b, 1.0, 1.0, 0.4, d), "threshold boundary is strict");
    }

    // Disjoint (inside range) vs excess (outside range) classification.
    {
        odneat::Genome base, other;
        const odneat::InnovationId n1{0, 10, 10}, n2{0, 20, 20}, n3{0, 30, 30};
        for (const auto id : {n1, n2, n3}) {
            odneat::NeuronGene ng{};
            ng.innovation_id = id;
            base.addNeuronGene(ng);
            other.addNeuronGene(ng);
        }

        auto mk_conn = [](std::uint64_t ts, odneat::InnovationId src, odneat::InnovationId dst) {
            odneat::ConnectionGene cg{};
            cg.innovation_id = odneat::InnovationId{0, ts, static_cast<std::uint32_t>(ts)};
            cg.input_neuron_id = src;
            cg.output_neuron_id = dst;
            return cg;
        };
        base.addConnectionGene(mk_conn(10, n1, n2));
        base.addConnectionGene(mk_conn(20, n2, n3));
        base.addConnectionGene(mk_conn(30, n1, n3));
        other.addConnectionGene(mk_conn(10, n1, n2));
        other.addConnectionGene(mk_conn(25, n1, n2));  // inside shared range -> disjoint
        other.addConnectionGene(mk_conn(30, n1, n3));
        const odneat::CompatibilityBreakdown disjoint_bd = odneat::analyseCompatibility(base, other, 1.0, 1.0, 0.4);
        report_failure(disjoint_bd.matching_count == 2 && disjoint_bd.disjoint_count == 2 && disjoint_bd.excess_count == 0, "inside-range extras are disjoint");
        odneat::Genome far = base;
        far.addConnectionGene(mk_conn(99, n1, n1));
        const odneat::CompatibilityBreakdown excess_bd = odneat::analyseCompatibility(base, far, 1.0, 1.0, 0.4);
        report_failure(excess_bd.excess_count == 1 && excess_bd.disjoint_count == 0, "outside-range extras are excess");
    }

    // Normaliser switches from 1 to larger genome size at 20 genes.
    {
        odneat::Genome small_a, small_b;
        odneat::Genome big_a, big_b;
        for (std::uint64_t ts = 1; ts <= 25; ++ts) {
            odneat::ConnectionGene cg{};
            cg.innovation_id = odneat::InnovationId{0, ts, static_cast<std::uint32_t>(ts)};
            big_a.addConnectionGene(cg);
            big_b.addConnectionGene(cg);
        }

        big_b.accessConnectionGenes().back().weight = 5.0;
        const odneat::CompatibilityBreakdown small_bd = odneat::analyseCompatibility(small_a, small_a, 1.0, 1.0, 0.4);
        const odneat::CompatibilityBreakdown big_bd = odneat::analyseCompatibility(big_a, big_b, 1.0, 1.0, 0.4);
        report_failure(small_bd.normaliser == 1.0, "small genomes normaliser 1");
        report_failure(big_bd.normaliser == 25.0, "large genomes normalise by size");
    }

    // Fitness sharing divides by species size; refresh averages adjusted fitness.
    {
        odneat::Genome g1 = first_genome, g2 = first_genome;
        g1.setFitness(10.0);
        g2.setFitness(20.0);
        std::vector<odneat::Genome> members{g1, g2};
        odneat::Species species{};
        species.species_id = 0;
        species.representative_index = 0;
        species.member_indices = {0, 1};
        const std::vector<odneat::Species> one{species};
        odneat::applyFitnessSharing(members, one);
        report_failure(members[0].getAdjustedFitness() == 5.0 && members[1].getAdjustedFitness() == 10.0, "sharing divides by size");
        std::vector<odneat::Species> live{species};
        odneat::refreshSpeciesFitness(members, live);
        report_failure(live[0].average_adjusted_fitness == 7.5, "species average of adjusted fitness");
    }

    if (failure_count == 0) {
        std::cout << "test_compatibility passed\n";
    }

    return failure_count == 0 ? 0 : 1;
}
