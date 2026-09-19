#include "crossover.h"

#include <iostream>
#include <random>

#include "genome.h"
#include "innovation_clock.h"
#include "mutation.h"

int run_crossover_suite() {
    int failure_count = 0;
    odneat::InnovationClock innovation_clock(1);
    odneat::Genome first_parent = odneat::Genome::createMinimalGenome(2, 1, innovation_clock);
    odneat::Genome second_parent = first_parent;
    second_parent.accessConnectionGenes().front().weight = 3.0;
    first_parent.setFitness(10.0);
    second_parent.setFitness(10.0);
    std::mt19937_64 randomGenerator(42ULL);
    odneat::CrossoverOutcome outcome{};
    const odneat::Genome offspring = odneat::crossoverGenomes(first_parent, second_parent, randomGenerator, 0.75, &outcome);

    if (offspring.getConnectionGenes().empty()) {
        std::cout << "FAIL: offspring has no connections\n";
        ++failure_count;
    }

    if (outcome.matching_genes == 0) {
        std::cout << "FAIL: expected matching genes\n";
        ++failure_count;
    }

    // Unevaluated offspring carries no parental fitness (paper Sec. 3.1: fitness is sampled mean).
    if (offspring.getFitness() != 0.0 || offspring.getEvaluationCount() != 0) {
        std::cout << "FAIL: offspring fitness unevaluated\n";
        ++failure_count;
    }

    // Disabled disjoint genes of the fitter parent are still inherited (NEAT canon p.108).
    {
        odneat::InnovationClock clock(5);
        odneat::Genome fit = odneat::Genome::createMinimalGenome(2, 1, clock);
        odneat::Genome weak = fit;
        std::mt19937_64 rng(31ULL);
        odneat::InnovationClock mut_clock(5);
        (void)odneat::addRandomConnection(fit, mut_clock, rng, -10.0, 10.0);
        fit.accessConnectionGenes().back().enabled = false;
        fit.setFitness(50.0);
        weak.setFitness(1.0);
        const odneat::InnovationId extra = fit.getConnectionGenes().back().innovation_id;
        std::mt19937_64 xrng(33ULL);
        const odneat::Genome child = odneat::crossoverGenomes(fit, weak, xrng, 0.75, nullptr);
        bool inherited = false;
        for (const auto &cg : child.getConnectionGenes()) {
            if (cg.innovation_id == extra) {
                inherited = true;
                break;
            }
        }

        if (!inherited) {
            std::cout << "FAIL: disabled fitter disjoint inherited\n";
            ++failure_count;
        }

        if (child.getFitness() != 0.0) {
            std::cout << "FAIL: unequal offspring fitness unevaluated\n";
            ++failure_count;
        }
    }

    // Unequal fitness: disjoint/excess come from the fitter parent only.
    {
        odneat::InnovationClock clock(2);
        odneat::Genome fit = odneat::Genome::createMinimalGenome(2, 1, clock);
        odneat::Genome weak = fit;
        std::mt19937_64 rng(7ULL);
        odneat::InnovationClock mut_clock(2);
        (void)odneat::addRandomConnection(fit, mut_clock, rng, -10.0, 10.0);
        fit.setFitness(100.0);
        weak.setFitness(1.0);
        const std::size_t fit_conns = fit.getConnectionGenes().size();
        const std::size_t weak_conns = weak.getConnectionGenes().size();
        std::mt19937_64 xrng(11ULL);
        const odneat::Genome child = odneat::crossoverGenomes(fit, weak, xrng, 0.75, nullptr);
        if (fit_conns > weak_conns) {
            if (child.getConnectionGenes().size() < fit_conns) {
                std::cout << "FAIL: fitter disjoint missing\n";
                ++failure_count;
            }

            bool has_weak_only = false;
            for (const auto &gene : weak.getConnectionGenes()) {
                bool in_fit = false;
                for (const auto &fg : fit.getConnectionGenes()) {
                    if (fg.innovation_id == gene.innovation_id) {
                        in_fit = true;
                        break;
                    }
                }

                if (!in_fit) {
                    for (const auto &cg : child.getConnectionGenes()) {
                        if (cg.innovation_id == gene.innovation_id) {
                            has_weak_only = true;
                            break;
                        }
                    }
                }
            }

            if (has_weak_only) {
                std::cout << "FAIL: weaker-only genes inherited on unequal fitness\n";
                ++failure_count;
            }
        }
    }

    // Disabled inheritance extremes are deterministic regardless of RNG.
    {
        odneat::InnovationClock clock(3);
        odneat::Genome p1 = odneat::Genome::createMinimalGenome(2, 1, clock);
        odneat::Genome p2 = p1;
        p1.accessConnectionGenes().front().enabled = false;
        p1.setFitness(5.0);
        p2.setFitness(5.0);
        std::mt19937_64 rng_a(21ULL), rng_b(22ULL);
        const odneat::InnovationId target = p1.getConnectionGenes().front().innovation_id;
        const odneat::Genome always_off = odneat::crossoverGenomes(p1, p2, rng_a, 1.0, nullptr);
        const odneat::Genome always_on = odneat::crossoverGenomes(p1, p2, rng_b, 0.0, nullptr);
        auto find_enabled = [&](const odneat::Genome &g) -> bool {
            for (const auto &cg : g.getConnectionGenes()) {
                if (cg.innovation_id == target) return cg.enabled;
            }

            return true;
        };
        if (find_enabled(always_off)) {
            std::cout << "FAIL: rate 1.0 must disable\n";
            ++failure_count;
        }

        if (!find_enabled(always_on)) {
            std::cout << "FAIL: rate 0.0 must enable\n";
            ++failure_count;
        }
    }

    if (failure_count == 0) {
        std::cout << "test_crossover passed\n";
    }

    return failure_count == 0 ? 0 : 1;
}
