#include "agent.h"

#include <iostream>
#include <string>
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

    // Introspection getters reflect construction arguments and runtime state.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 100;
        odneat::OdneatAgent agent(12, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 37ULL);
        if (agent.getInputCount() != 4 || agent.getOutputCount() != 2) {
            std::cout << "FAIL: controller dims\n";
            ++f;
        }
        if (agent.getMinimumEnergy() != 0.0 || agent.getMaximumEnergy() != 100.0 || agent.getDefaultEnergy() != 50.0 || agent.getMinimumThreshold() != 0.0) {
            std::cout << "FAIL: energy bounds\n";
            ++f;
        }
        if (agent.getParameters().maturation_period_cycles != 100) {
            std::cout << "FAIL: parameters\n";
            ++f;
        }
        if (!agent.isExchangeEnabled() || !agent.isTabuEnabled() || !agent.isMaturationEnabled() || !agent.isSpeciationEnabled()) {
            std::cout << "FAIL: flags default on\n";
            ++f;
        }
        agent.setExchangeEnabled(false);
        agent.setTabuEnabled(false);
        if (agent.isExchangeEnabled() || agent.isTabuEnabled()) {
            std::cout << "FAIL: flags toggle\n";
            ++f;
        }
    }

    // Direct export/restore round-trips state; every validation rejects without partial mutation.
    {
        odneat::OdneatParams mp;
        mp.maturation_period_cycles = 10;
        mp.internal_population_size = 10;
        odneat::OdneatAgent source(14, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 41ULL);
        const std::vector<double> sensors(4, 0.5);
        for (int i = 0; i < 12; ++i) {
            source.executeControlCycle(sensors, 1.0, {});
        }
        odneat::AgentCheckpointState state = source.exportCheckpointState();
        odneat::OdneatAgent target(14, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 43ULL);
        std::string error_message{};
        if (!target.restoreCheckpointState(state, &error_message)) {
            std::cout << "FAIL: direct restore\n";
            ++f;
        }
        if (!target.getActiveGenome().isIdenticalTo(source.getActiveGenome()) || target.getEnergy() != source.getEnergy()) {
            std::cout << "FAIL: direct restore values\n";
            ++f;
        }
        // Nullptr error sink also restores.
        odneat::OdneatAgent null_target(14, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 47ULL);
        if (!null_target.restoreCheckpointState(state, nullptr)) {
            std::cout << "FAIL: restore nullptr error\n";
            ++f;
        }

        auto expect_reject = [&](odneat::AgentCheckpointState bad, const char *message) {
            odneat::OdneatAgent candidate(14, 4, 2, 0.0, 100.0, 50.0, 0.0, mp, 53ULL);
            const odneat::Genome before = candidate.getActiveGenome();
            std::string local_error{};
            if (candidate.restoreCheckpointState(bad, &local_error)) {
                std::cout << "FAIL: " << message << " accepted\n";
                ++f;
            }
            if (local_error.empty()) {
                std::cout << "FAIL: " << message << " reports error\n";
                ++f;
            }
            if (!candidate.getActiveGenome().isIdenticalTo(before)) {
                std::cout << "FAIL: " << message << " untouched\n";
                ++f;
            }
            // Nullptr sink rejects too.
            if (candidate.restoreCheckpointState(bad, nullptr)) {
                std::cout << "FAIL: " << message << " nullptr rejects\n";
                ++f;
            }
        };

        odneat::AgentCheckpointState bad_dims = state;
        bad_dims.input_count = 6;
        expect_reject(bad_dims, "dims mismatch");

        odneat::AgentCheckpointState bad_energy = state;
        bad_energy.maximum_energy = 999.0;
        expect_reject(bad_energy, "energy bounds mismatch");

        odneat::AgentCheckpointState bad_active = state;
        bad_active.active_genome = odneat::Genome();
        expect_reject(bad_active, "empty active");

        odneat::AgentCheckpointState bad_population = state;
        bad_population.population_genomes.clear();
        expect_reject(bad_population, "empty population");

        odneat::AgentCheckpointState bad_capacity = state;
        bad_capacity.parameters.internal_population_size = 0;
        expect_reject(bad_capacity, "population exceeds capacity");

        odneat::AgentCheckpointState bad_counters = state;
        bad_counters.evaluation_count = 0;
        expect_reject(bad_counters, "invalid counters");

        odneat::AgentCheckpointState bad_random = state;
        bad_random.random_state.clear();
        expect_reject(bad_random, "empty random state");

        odneat::AgentCheckpointState corrupt_random = state;
        corrupt_random.random_state = "not a valid mt19937_64 stream !!!";
        expect_reject(corrupt_random, "corrupt random state");
    }

    if (f == 0) std::cout << "test_agent passed\n";

    return f == 0 ? 0 : 1;
}
