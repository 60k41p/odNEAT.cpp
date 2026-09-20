#include "population.h"

#include <iostream>
#include <random>

#include "genome.h"
#include "innovation_clock.h"
int run_population_suite() {
    int f = 0;
    odneat::InternalPopulation p(4, 1.0, 1.0, 0.4, 3.0);
    odneat::InnovationClock c(0);
    odneat::Genome g = odneat::Genome::createMinimalGenome(2, 2, c);
    g.setFitness(10.0);
    g.setEvaluationCount(1);

    if (p.addGenome(g) != odneat::PopulationAcceptance::kAccepted) {
        std::cout << "FAIL: add\n";
        ++f;
    }

    if (p.addGenome(g) != odneat::PopulationAcceptance::kDuplicateMerged) {
        std::cout << "FAIL: dup\n";
        ++f;
    }

    for (int i = 0; i < 6; ++i) {
        odneat::Genome h = odneat::Genome::createMinimalGenome(2, 2, c);
        // Deterministic minimals are identical; perturb weight to make distinct genomes for cap test.
        h.accessConnectionGenes().front().weight = 10.0 + static_cast<double>(i) * 2.0;
        h.setFitness((double)i);
        h.setEvaluationCount(1);
        odneat::Genome ev;
        bool has = false;
        p.addGenomeWithEviction(h, ev, has);
    }

    if (p.getCurrentSize() != 4) {
        std::cout << "FAIL: cap\n";
        ++f;
    }

    std::mt19937_64 rng(1);

    if (p.selectParentSpecies(rng) < 0) {
        std::cout << "FAIL: select\n";
        ++f;
    }

    // Incremental duplicate merge averages by sample count: (10*1+20)/2 = 15, count 2.
    {
        odneat::InternalPopulation q(4, 1.0, 1.0, 0.4, 3.0);
        odneat::InnovationClock clock(2);
        odneat::Genome base = odneat::Genome::createMinimalGenome(2, 2, clock);
        base.setFitness(10.0);
        base.setEvaluationCount(1);
        q.addGenome(base);
        if (!q.mergeDuplicateFitness(base, 20.0)) {
            std::cout << "FAIL: merge found\n";
            ++f;
        } else {
            const double merged = q.getGenomes().front().getFitness();
            if (merged != 15.0 || q.getGenomes().front().getEvaluationCount() != 2) {
                std::cout << "FAIL: merge math\n";
                ++f;
            }
        }

        if (q.mergeDuplicateFitness(odneat::Genome(), 5.0)) {
            std::cout << "FAIL: merge missing\n";
            ++f;
        }
    }

    // Eviction replaces the worst adjusted genome and reports it.
    {
        odneat::InternalPopulation full(2, 1.0, 1.0, 0.4, 3.0);
        odneat::InnovationClock clock(3);
        odneat::Genome high = odneat::Genome::createMinimalGenome(2, 2, clock);
        high.accessConnectionGenes().front().weight = 1.0;
        high.setFitness(100.0);
        high.setEvaluationCount(1);
        odneat::Genome low = odneat::Genome::createMinimalGenome(2, 2, clock);
        low.accessConnectionGenes().front().weight = -1.0;
        low.setFitness(0.0);
        low.setEvaluationCount(1);
        full.addGenome(high);
        full.addGenome(low);
        odneat::Genome newcomer = odneat::Genome::createMinimalGenome(2, 2, clock);
        newcomer.accessConnectionGenes().front().weight = 0.0;
        newcomer.setFitness(50.0);
        newcomer.setEvaluationCount(1);
        odneat::Genome evicted;
        bool had = false;
        full.addGenomeWithEviction(newcomer, evicted, had);
        if (!had || full.getCurrentSize() != 2) {
            std::cout << "FAIL: eviction reported\n";
            ++f;
        }

        if (evicted.getFitness() != 0.0) {
            std::cout << "FAIL: worst evicted\n";
            ++f;
        }
    }

    // Empty removal fails; single-member tournament is deterministic.
    {
        odneat::InternalPopulation empty(4, 1.0, 1.0, 0.4, 3.0);
        odneat::Genome removed;
        if (empty.removeWorstGenome(removed)) {
            std::cout << "FAIL: empty remove\n";
            ++f;
        }

        if (empty.selectParentSpecies(rng) != -1) {
            std::cout << "FAIL: empty select\n";
            ++f;
        }

        odneat::InnovationClock clock(4);
        odneat::Genome solo = odneat::Genome::createMinimalGenome(1, 1, clock);
        solo.setFitness(7.0);
        solo.setEvaluationCount(1);
        empty.addGenome(solo);
        const int species = empty.selectParentSpecies(rng);
        if (species < 0 || empty.tournamentSelectInSpecies(species, rng) != 0) {
            std::cout << "FAIL: solo tournament\n";
            ++f;
        }

        if (empty.findSpeciesFitnessForGenome(99) != 0.0) {
            std::cout << "FAIL: missing species fitness\n";
            ++f;
        }

        // Zero total fitness falls back to uniform choice instead of -1.
        {
            odneat::InternalPopulation flat(4, 1.0, 1.0, 0.4, 3.0);
            odneat::InnovationClock zclock(9);
            odneat::Genome z1 = odneat::Genome::createMinimalGenome(1, 1, zclock);
            z1.accessConnectionGenes().front().weight = 1.0;
            z1.setFitness(0.0);
            z1.setEvaluationCount(1);
            odneat::Genome z2 = odneat::Genome::createMinimalGenome(1, 1, zclock);
            z2.accessConnectionGenes().front().weight = 2.0;
            z2.setFitness(0.0);
            z2.setEvaluationCount(1);
            flat.addGenome(z1);
            flat.addGenome(z2);
            if (flat.selectParentSpecies(rng) < 0) {
                std::cout << "FAIL: zero-total fallback\n";
                ++f;
            }
        }

        empty.clearPopulation();
        if (!empty.isEmpty() || empty.isFull()) {
            std::cout << "FAIL: clear state\n";
            ++f;
        }
    }

    // "No niching" ablation: single shared species with unshared (raw) fitness.
    {
        odneat::InternalPopulation pop(4, 1.0, 1.0, 0.4, 3.0);
        odneat::InnovationClock clock(6);
        odneat::Genome g1 = odneat::Genome::createMinimalGenome(1, 1, clock);
        g1.accessConnectionGenes().front().weight = 1.0;
        g1.setFitness(10.0);
        g1.setEvaluationCount(1);
        odneat::Genome g2 = odneat::Genome::createMinimalGenome(1, 1, clock);
        g2.accessConnectionGenes().front().weight = 9.0;
        g2.setFitness(20.0);
        g2.setEvaluationCount(1);
        pop.addGenome(g1);
        pop.addGenome(g2);
        pop.setNichingEnabled(false);
        if (pop.isNichingEnabled()) {
            std::cout << "FAIL: niching flag\n";
            ++f;
        }

        if (pop.getSpecies().size() != 1 || pop.getSpecies().front().member_indices.size() != 2) {
            std::cout << "FAIL: single shared species\n";
            ++f;
        }

        if (pop.getGenomes()[0].getAdjustedFitness() != 10.0 || pop.getGenomes()[1].getAdjustedFitness() != 20.0) {
            std::cout << "FAIL: no fitness sharing\n";
            ++f;
        }

        pop.setNichingEnabled(true);
        if (!pop.isNichingEnabled() || pop.getSpecies().empty()) {
            std::cout << "FAIL: niching restored\n";
            ++f;
        }
    }

    // Stored-fitness sync updates the copy and sharing without re-partitioning.
    {
        odneat::InternalPopulation pop(4, 1.0, 1.0, 0.4, 3.0);
        odneat::InnovationClock clock(7);
        odneat::Genome g = odneat::Genome::createMinimalGenome(1, 1, clock);
        g.setFitness(10.0);
        g.setEvaluationCount(1);
        pop.addGenome(g);
        const std::size_t species_before = pop.getSpecies().size();
        if (!pop.syncStoredFitness(g, 40.0, 4)) {
            std::cout << "FAIL: sync found\n";
            ++f;
        }

        if (pop.getGenomes().front().getFitness() != 40.0 || pop.getGenomes().front().getEvaluationCount() != 4) {
            std::cout << "FAIL: sync values\n";
            ++f;
        }

        if (pop.getSpecies().size() != species_before) {
            std::cout << "FAIL: sync keeps partition\n";
            ++f;
        }

        if (pop.syncStoredFitness(odneat::Genome(), 1.0, 1)) {
            std::cout << "FAIL: sync missing\n";
            ++f;
        }
    }

    // restoreGenomes replaces contents verbatim and re-speciates exactly once.
    {
        odneat::InternalPopulation pop(4, 1.0, 1.0, 0.4, 3.0);
        odneat::InnovationClock clock(9);
        odneat::Genome first = odneat::Genome::createMinimalGenome(2, 2, clock);
        first.setFitness(10.0);
        first.setEvaluationCount(2);
        odneat::Genome second = odneat::Genome::createMinimalGenome(2, 2, clock);
        second.accessConnectionGenes().front().weight += 5.0;
        second.setFitness(30.0);
        second.setEvaluationCount(1);
        pop.restoreGenomes({first, second});
        if (pop.getCurrentSize() != 2) {
            std::cout << "FAIL: restore size\n";
            ++f;
        } else {
            if (!pop.getGenomes()[0].isIdenticalTo(first) || !pop.getGenomes()[1].isIdenticalTo(second)) {
                std::cout << "FAIL: restore order\n";
                ++f;
            }
            if (pop.getGenomes()[0].getFitness() != 10.0 || pop.getGenomes()[0].getEvaluationCount() != 2) {
                std::cout << "FAIL: restore fitness\n";
                ++f;
            }
        }
        if (pop.getSpecies().empty()) {
            std::cout << "FAIL: restore speciates\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_population passed\n";

    return f == 0 ? 0 : 1;
}
