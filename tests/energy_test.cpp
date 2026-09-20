#include "energy.h"

#include <iostream>
int run_energy_suite() {
    int f = 0;
    odneat::EnergyTracker t(0.0, 100.0, 50.0, 0.0);

    if (t.getEnergy() != 50.0) {
        std::cout << "FAIL: default\n";
        ++f;
    }

    t.addEnergyDelta(60.0);

    if (t.getEnergy() != 100.0) {
        std::cout << "FAIL: clamp max\n";
        ++f;
    }

    t.addEnergyDelta(-200.0);

    if (t.getEnergy() != 0.0) {
        std::cout << "FAIL: clamp min\n";
        ++f;
    }

    if (!t.isDepleted()) {
        std::cout << "FAIL: depleted\n";
        ++f;
    }

    t.refillToDefault();

    if (t.getEnergy() != 50.0) {
        std::cout << "FAIL: refill\n";
        ++f;
    }

    odneat::FitnessAverager a;
    a.addEnergySample(10.0);
    a.addEnergySample(20.0);

    if (a.getFitness() != 15.0) {
        std::cout << "FAIL: mean\n";
        ++f;
    }

    a.mergeRemoteSample(30.0);

    if (a.getSampleCount() != 3) {
        std::cout << "FAIL: count\n";
        ++f;
    }

    if (a.getFitness() != 20.0) {
        std::cout << "FAIL: remote merge mean\n";
        ++f;
    }

    a.resetAverager();

    if (a.getSampleCount() != 0 || a.getFitness() != 0.0) {
        std::cout << "FAIL: reset\n";
        ++f;
    }

    // Threshold is inclusive: energy exactly at threshold counts as depleted.
    {
        odneat::EnergyTracker edge(0.0, 100.0, 50.0, 10.0);
        edge.setEnergy(10.0);
        if (!edge.isDepleted()) {
            std::cout << "FAIL: threshold inclusive\n";
            ++f;
        }

        edge.setEnergy(10.0001);
        if (edge.isDepleted()) {
            std::cout << "FAIL: above threshold alive\n";
            ++f;
        }

        edge.setEnergy(500.0);
        if (edge.getEnergy() != 100.0) {
            std::cout << "FAIL: set clamp max\n";
            ++f;
        }

        edge.setEnergy(-50.0);
        if (edge.getEnergy() != 0.0) {
            std::cout << "FAIL: set clamp min\n";
            ++f;
        }
    }

    // restoreState resumes the running mean without replaying individual samples.
    {
        odneat::FitnessAverager restored;
        restored.restoreState(20.0, 3);
        if (restored.getFitness() != 20.0 || restored.getSampleCount() != 3) {
            std::cout << "FAIL: restore state\n";
            ++f;
        }

        restored.addEnergySample(30.0);
        if (restored.getSampleCount() != 4 || restored.getFitness() != 22.5) {
            std::cout << "FAIL: resume after restore\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_energy passed\n";

    return f == 0 ? 0 : 1;
}
