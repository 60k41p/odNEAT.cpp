#include "innovation_clock.h"

#include <iostream>

int run_innovation_clock_suite() {
    int f = 0;
    auto expect = [&](bool cond, const char *msg) {
        if (!cond) {
            std::cout << "FAIL: " << msg << "\n";
            ++f;
        }
    };

    // Timestamp dominates robot id: cross-robot comparison stays chronological.
    const odneat::InnovationId early{5, 100, 0};
    const odneat::InnovationId late{1, 200, 0};
    expect(early < late, "timestamp-first ordering across robots");
    expect(!(late < early), "ordering antisymmetric");
    expect((odneat::InnovationId{2, 100, 0} < odneat::InnovationId{9, 100, 0}), "robot tiebreak same timestamp");
    expect((odneat::InnovationId{1, 50, 0} < odneat::InnovationId{1, 50, 1}), "counter tiebreak");

    // Monotonic minting under fast successive calls (system clock may repeat).
    odneat::InnovationClock clock(3);
    odneat::InnovationId first = clock.nextInnovationId();
    odneat::InnovationId second = clock.nextInnovationId();
    expect(first < second, "successive ids strictly increase");
    expect(first.robot_identifier == 3 && second.robot_identifier == 3, "robot stamped");
    expect(clock.getMintedCount() == 2, "mint count tracks");

    // innovationIdAt bumps when timestamp would go backwards.
    odneat::InnovationClock det(9);
    odneat::InnovationId d1 = det.innovationIdAt(500);
    odneat::InnovationId d2 = det.innovationIdAt(500);
    expect(d1.timestamp_nanoseconds == 500, "deterministic timestamp honored");
    expect(d2.timestamp_nanoseconds == 501, "repeated timestamp bumped monotonic");
    expect(d1 < d2, "deterministic ids ordered");

    // Export/restore prevents restart collisions and stays strictly monotonic.
    odneat::InnovationClock before(1);
    (void)before.nextInnovationId();
    const auto state = before.exportState();
    odneat::InnovationClock after(1, state.first, state.second);
    after.restoreState(state.first, state.second);
    odneat::InnovationId fresh = after.nextInnovationId();
    expect(fresh.timestamp_nanoseconds > state.second, "restored clock strictly advances");
    expect(fresh.local_counter == state.first, "restored counter continues");

    // innovationIdHash is deterministic over the full identity.
    odneat::InnovationIdHash hasher;
    expect(hasher(early) == odneat::InnovationIdHash{}(early), "hash deterministic");

    if (f == 0) std::cout << "test_innovation_clock passed\n";
    return f == 0 ? 0 : 1;
}
