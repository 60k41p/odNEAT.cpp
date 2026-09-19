#include "agent.h"

#include <iostream>
#include <vector>

#include "config.h"
int run_agent_suite() {
    int f = 0;
    odneat::OdneatParams params;
    params.maturation_period_cycles = 5;
    odneat::OdneatAgent agent(0, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 42ULL);

    if (agent.getPopulation().getCurrentSize() == 0) {
        std::cout << "FAIL: pop\n";
        ++f;
    }

    std::vector<double> s(4, 0.5);
    odneat::AgentStepResult r = agent.executeControlCycle(s, 1.0, {});

    if (r.did_replace_controller) {
        std::cout << "FAIL: replace\n";
        ++f;
    }

    for (int i = 0; i < 10; ++i) {
        agent.executeControlCycle(s, -100.0, {});
    }

    if (agent.getEvaluationCount() < 2) {
        std::cout << "FAIL: depletion\n";
        ++f;
    }

    // Maturation guard delays replacement until the protection period ticks out.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 3;
        odneat::OdneatAgent guarded(1, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 7ULL);
        const int before = guarded.getEvaluationCount();
        odneat::AgentStepResult first = guarded.executeControlCycle(s, -1000.0, {});
        if (first.did_replace_controller) {
            std::cout << "FAIL: maturation blocks\n";
            ++f;
        }

        bool eventually = first.did_replace_controller;
        for (int i = 0; i < 4 && !eventually; ++i) {
            eventually = guarded.executeControlCycle(s, 0.0, {}).did_replace_controller;
        }

        if (!eventually || guarded.getEvaluationCount() != before + 1) {
            std::cout << "FAIL: replacement after guard\n";
            ++f;
        }

        if (guarded.getTabuList().getTabuSize() == 0) {
            std::cout << "FAIL: failed genome tabu-listed\n";
            ++f;
        }
    }

    // Maturation disabled allows immediate replacement on depletion.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 50;
        odneat::OdneatAgent exposed(2, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 9ULL);
        exposed.setMaturationEnabled(false);
        if (!exposed.executeControlCycle(s, -1000.0, {}).did_replace_controller) {
            std::cout << "FAIL: no-maturation immediate\n";
            ++f;
        }
    }

    // Exchange off discards novel genomes; tabu off accepts them.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        odneat::OdneatAgent receiver(3, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 11ULL);
        odneat::OdneatAgent sender(4, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 13ULL);
        sender.forceControllerReplacement();
        odneat::ReceivedGenome incoming{sender.getActiveGenome(), sender.getEnergy()};
        // Force novelty: perturb a weight so it is not an identical duplicate.
        incoming.genome.accessConnectionGenes().front().weight += 4.5;
        receiver.setExchangeEnabled(false);
        int accepted = 0, merged = 0, rejected = 0;
        receiver.incorporateExternal({incoming}, accepted, merged, rejected);
        if (accepted != 0) {
            std::cout << "FAIL: exchange off discards\n";
            ++f;
        }

        receiver.setExchangeEnabled(true);
        receiver.setTabuEnabled(false);
        receiver.incorporateExternal({incoming}, accepted, merged, rejected);
        if (accepted != 1) {
            std::cout << "FAIL: tabu off accepts\n";
            ++f;
        }
    }

    // Identical genomes merge instead of growing the population; similar-to-tabu rejects.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        odneat::OdneatAgent receiver(5, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 17ULL);
        const std::size_t before_pop = receiver.getPopulation().getCurrentSize();
        odneat::ReceivedGenome dup{receiver.getActiveGenome(), 77.0};
        int accepted = 0, merged = 0, rejected = 0;
        receiver.incorporateExternal({dup}, accepted, merged, rejected);
        if (merged != 1 || receiver.getPopulation().getCurrentSize() != before_pop) {
            std::cout << "FAIL: duplicate merges\n";
            ++f;
        }

        receiver.forceControllerReplacement();  // failed active enters tabu
        odneat::ReceivedGenome similar{receiver.getTabuList().getTabuGenomes().front(), 10.0};
        similar.genome.accessConnectionGenes().front().weight += 0.05;  // still compatible
        receiver.incorporateExternal({similar}, accepted, merged, rejected);
        if (rejected != 1) {
            std::cout << "FAIL: tabu rejects similar\n";
            ++f;
        }
    }

    // Split API matches legacy single-phase behavior for wheels and energy.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        odneat::OdneatAgent legacy(6, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 23ULL);
        odneat::OdneatAgent split(6, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 23ULL);
        odneat::AgentStepResult lr = legacy.executeControlCycle(s, 2.5, {});
        int accepted = 0, merged = 0, rejected = 0;
        split.incorporateExternal({}, accepted, merged, rejected);
        const std::vector<double> raw = split.stepController(s);
        double lw = 0.0, rw = 0.0;
        if (raw.size() >= 2) {
            lw = odneat::rescaleOutputToWheelSpeed(raw[0]);
            rw = odneat::rescaleOutputToWheelSpeed(raw[1]);
        }

        const bool replaced = split.updateEnergyAndMaybeReplace(2.5);
        if (replaced != lr.did_replace_controller || lw != lr.left_wheel_speed || rw != lr.right_wheel_speed || split.getEnergy() != legacy.getEnergy()) {
            std::cout << "FAIL: split matches legacy\n";
            ++f;
        }

        // Forced replacement refills energy and rearms maturation.
        split.forceControllerReplacement();
        if (split.getEvaluationCount() != 2 || split.getEnergy() != 50.0 || split.getMaturationCyclesRemaining() != 100) {
            std::cout << "FAIL: forced replacement state\n";
            ++f;
        }
    }

    // Speciation switch actually toggles the population niching scheme.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        odneat::OdneatAgent agent(7, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 29ULL);
        agent.setSpeciationEnabled(false);
        if (agent.getPopulation().getSpecies().size() != 1) {
            std::cout << "FAIL: speciation off single species\n";
            ++f;
        }

        agent.setSpeciationEnabled(true);
        if (agent.getPopulation().isNichingEnabled() != true) {
            std::cout << "FAIL: speciation restored\n";
            ++f;
        }
    }

    // Evicted active copy falls back to a neutral broadcast estimate instead of stalling.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        mp.internal_population_size = 1;
        odneat::OdneatAgent agent(8, 2, 1, 0.0, 100.0, 50.0, 0.0, mp, 31ULL);
        odneat::InnovationClock clock(77);
        odneat::Genome newcomer = odneat::Genome::createMinimalGenome(2, 1, clock);
        newcomer.accessConnectionGenes().front().weight = 9.0;
        newcomer.setFitness(0.0);
        newcomer.setEvaluationCount(1);
        int accepted = 0, merged = 0, rejected = 0;
        agent.incorporateExternal({odneat::ReceivedGenome{newcomer, 100.0}}, accepted, merged, rejected);
        if (accepted != 1) {
            std::cout << "FAIL: eviction insert\n";
            ++f;
        }

        odneat::Genome broadcast;
        double energy = 0.0;
        // Single-species fallback mean equals the total, so P = 1 deterministically.
        if (!agent.prepareBroadcast(broadcast, energy)) {
            std::cout << "FAIL: fallback broadcasts\n";
            ++f;
        }

        if (!broadcast.isIdenticalTo(agent.getActiveGenome()) || energy != agent.getEnergy()) {
            std::cout << "FAIL: fallback packages active\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_agent passed\n";

    return f == 0 ? 0 : 1;
}
