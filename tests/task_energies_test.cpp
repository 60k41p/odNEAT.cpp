#include "tasks/task_energies.h"

#include <iostream>
int run_task_energies_suite() {
    int f = 0;
    odneat::AggregationEnergyUpdater agg;

    if (agg.computeProximityReward(3) != 9.0) {
        std::cout << "FAIL: alpha\n";
        ++f;
    }

    if (agg.computeMovementQuality(1.0, -1.0) != -1.0) {
        std::cout << "FAIL: gamma\n";
        ++f;
    }

    if (agg.computeMovementQuality(1.0, 1.0) <= 0.0) {
        std::cout << "FAIL: forward\n";
        ++f;
    }

    odneat::NavigationEnergyUpdater nav;

    if (nav.normaliseEnergy(0.0) != -1.0 || nav.normaliseEnergy(1.0) != 1.0) {
        std::cout << "FAIL: fnorm\n";
        ++f;
    }

    if (nav.computeEnergyDelta(1.0, 0.0, 0.0, 0.0) != 1.0) {
        std::cout << "FAIL: nav\n";
        ++f;
    }

    odneat::PhototaxisEnergyUpdater photo;

    if (photo.computeEnergyDelta(0.8) != 0.8) {
        std::cout << "FAIL: bright\n";
        ++f;
    }

    if (photo.computeEnergyDelta(0.3) != 0.0) {
        std::cout << "FAIL: dead\n";
        ++f;
    }

    if (photo.computeEnergyDelta(0.0) != -0.01) {
        std::cout << "FAIL: idle\n";
        ++f;
    }

    // Aggregation edges: stopped scores 0; backward equals forward (abs-based speeds).
    if (agg.computeMovementQuality(0.0, 0.0) != 0.0) {
        std::cout << "FAIL: stopped\n";
        ++f;
    }

    if (agg.computeMovementQuality(-1.0, -1.0) != agg.computeMovementQuality(1.0, 1.0)) {
        std::cout << "FAIL: backward symmetric\n";
        ++f;
    }

    if (agg.computeEnergyDelta(2, 1.0, 1.0) != agg.computeProximityReward(2) + agg.computeMovementQuality(1.0, 1.0)) {
        std::cout << "FAIL: delta sum\n";
        ++f;
    }

    // Navigation edges: full blockage or max spin both bottom out at -1.
    if (nav.computeEnergyDelta(1.0, 0.0, 1.0, 0.0) != -1.0) {
        std::cout << "FAIL: robot block\n";
        ++f;
    }

    if (nav.computeEnergyDelta(1.0, 1.0, 0.0, 0.0) != -1.0) {
        std::cout << "FAIL: spin bottoms\n";
        ++f;
    }

    if (nav.computeSpeedSum(1.0, -1.0) != 1.0 || nav.computeWheelDifference(1.0, -1.0) != 1.0) {
        std::cout << "FAIL: speed helpers\n";
        ++f;
    }

    // Phototaxis boundary: exactly at threshold falls in the dead zone; negatives penalize.
    if (photo.computeEnergyDelta(0.5) != 0.0) {
        std::cout << "FAIL: threshold edge\n";
        ++f;
    }

    if (photo.computeEnergyDelta(-0.4) != -0.01) {
        std::cout << "FAIL: negative idle\n";
        ++f;
    }

    if (f == 0) std::cout << "test_task_energies passed\n";

    return f == 0 ? 0 : 1;
}
