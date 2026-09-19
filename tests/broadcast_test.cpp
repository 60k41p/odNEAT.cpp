#include "broadcast.h"

#include <iostream>
#include <random>
int run_broadcast_suite() {
    int f = 0;
    odneat::BroadcastPolicy pol;

    if (pol.computeBroadcastProbability(1.0, 4.0) != 0.25) {
        std::cout << "FAIL: prob\n";
        ++f;
    }

    if (pol.computeBroadcastProbability(1.0, 0.0) != 0.0) {
        std::cout << "FAIL: zero\n";
        ++f;
    }

    std::mt19937_64 rng(42);

    if (!pol.shouldBroadcast(5.0, 5.0, rng)) {
        std::cout << "FAIL: p1\n";
        ++f;
    }

    if (pol.shouldBroadcast(0.0, 5.0, rng)) {
        std::cout << "FAIL: p0\n";
        ++f;
    }

    // Clamping through both the probability and sampling paths.
    if (pol.computeBroadcastProbability(9.0, 4.0) != 1.0) {
        std::cout << "FAIL: clamp high\n";
        ++f;
    }

    if (pol.computeBroadcastProbability(-2.0, 4.0) != 0.0) {
        std::cout << "FAIL: clamp low\n";
        ++f;
    }

    if (pol.computeBroadcastProbability(1.0, -3.0) != 0.0) {
        std::cout << "FAIL: negative total\n";
        ++f;
    }

    std::mt19937_64 rng2(99);
    if (!pol.shouldBroadcast(9.0, 4.0, rng2)) {
        std::cout << "FAIL: clamped p1 broadcasts\n";
        ++f;
    }

    if (pol.shouldBroadcast(-2.0, 4.0, rng2)) {
        std::cout << "FAIL: clamped p0 silent\n";
        ++f;
    }

    if (f == 0) std::cout << "test_broadcast passed\n";

    return f == 0 ? 0 : 1;
}
