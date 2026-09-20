#include <cstddef>
#include <iostream>
#include <vector>

#include "genome.h"
#include "innovation_clock.h"
#include "tabu_list.h"
int run_tabu_suite() {
    int f = 0;
    odneat::TabuList t(1.0, 1.0, 0.4, 3.0, 50);
    odneat::InnovationClock c(0);
    odneat::Genome g = odneat::Genome::createMinimalGenome(2, 2, c);

    if (!t.approvesCandidate(g)) {
        std::cout << "FAIL: empty\n";
        ++f;
    }

    t.addTabuGenome(g);

    if (t.approvesCandidate(g)) {
        std::cout << "FAIL: identical\n";
        ++f;
    }

    for (int i = 0; i < 60; ++i) {
        odneat::Genome o = odneat::Genome::createMinimalGenome(5, 2, c);
        t.observeReceivedGenome(o);
    }

    if (t.getTabuSize() != 0) {
        std::cout << "FAIL: expiry\n";
        ++f;
    }

    // Compatibility-based filtering: small weight shift still blocked, far genome passes.
    {
        odneat::TabuList f2(1.0, 1.0, 0.4, 3.0, 50);
        odneat::InnovationClock c2(1);
        odneat::Genome base = odneat::Genome::createMinimalGenome(2, 2, c2);
        f2.addTabuGenome(base);
        odneat::Genome near = base;
        near.accessConnectionGenes().front().weight += 0.1;
        if (f2.approvesCandidate(near)) {
            std::cout << "FAIL: near-tabu blocked\n";
            ++f;
        }

        odneat::Genome far = odneat::Genome::createMinimalGenome(6, 2, c2);
        if (!f2.approvesCandidate(far)) {
            std::cout << "FAIL: far genome passes\n";
            ++f;
        }
    }

    // Expiry retains entries with a similar genome still in the recent window.
    {
        odneat::TabuList f3(1.0, 1.0, 0.4, 3.0, 4);
        odneat::InnovationClock c3(2);
        odneat::Genome blocked = odneat::Genome::createMinimalGenome(2, 2, c3);
        f3.addTabuGenome(blocked);
        f3.observeReceivedGenome(blocked);
        if (f3.getTabuSize() != 1) {
            std::cout << "FAIL: similar retains tabu\n";
            ++f;
        }

        for (int i = 0; i < 6; ++i) {
            odneat::Genome other = odneat::Genome::createMinimalGenome(6, 2, c3);
            f3.observeReceivedGenome(other);
        }

        if (f3.getTabuSize() != 0) {
            std::cout << "FAIL: window slides expiry\n";
            ++f;
        }

        if (f3.getRecentHistory().size() > 4) {
            std::cout << "FAIL: history capped\n";
            ++f;
        }

        f3.clearTabuList();
        if (f3.getTabuSize() != 0 || !f3.getRecentHistory().empty()) {
            std::cout << "FAIL: clear\n";
            ++f;
        }
    }

    // restoreState replaces contents verbatim, keeping only the most recent window entries.
    {
        odneat::TabuList f4(1.0, 1.0, 0.4, 3.0, 4);
        odneat::InnovationClock c4(3);
        odneat::Genome blocked = odneat::Genome::createMinimalGenome(2, 2, c4);
        std::vector<odneat::Genome> history{};
        for (int i = 0; i < 6; ++i) {
            odneat::Genome other = odneat::Genome::createMinimalGenome(6, 2, c4);
            other.accessConnectionGenes().front().weight = 1.0 + static_cast<double>(i);
            history.push_back(other);
        }
        f4.restoreState({blocked}, history);
        if (f4.getTabuSize() != 1 || f4.getRecentHistory().size() != 4) {
            std::cout << "FAIL: restore trims window\n";
            ++f;
        } else {
            // Only the last four histories are kept, in order.
            for (std::size_t i = 0; i < 4; ++i) {
                if (!f4.getRecentHistory()[i].isIdenticalTo(history[i + 2])) {
                    std::cout << "FAIL: restore keeps newest\n";
                    ++f;
                    break;
                }
            }
        }
        if (f4.approvesCandidate(blocked)) {
            std::cout << "FAIL: restore keeps tabu\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_tabu passed\n";

    return f == 0 ? 0 : 1;
}
