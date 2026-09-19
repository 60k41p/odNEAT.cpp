#include <iostream>
#include <vector>

#include "agent.h"
#include "config.h"
int run_integration_suite() {
    int f = 0;
    odneat::OdneatParams params;
    params.maturation_period_cycles = 10;
    params.internal_population_size = 10;
    odneat::OdneatAgent a(0, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 1ULL);
    odneat::OdneatAgent b(1, 4, 2, 0.0, 100.0, 50.0, 0.0, params, 2ULL);
    std::vector<double> s(4, 0.8);

    for (int i = 0; i < 50; ++i) {
        odneat::Genome bg;
        double be = 0.0;
        bool wa = a.prepareBroadcast(bg, be);
        bool wb = b.prepareBroadcast(bg, be);
        std::vector<odneat::ReceivedGenome> to_a, to_b;

        if (wb) {
            odneat::ReceivedGenome g;
            g.genome = b.getActiveGenome();
            g.sender_energy = b.getEnergy();
            to_a.push_back(g);
        }

        if (wa) {
            odneat::ReceivedGenome g;
            g.genome = a.getActiveGenome();
            g.sender_energy = a.getEnergy();
            to_b.push_back(g);
        }

        a.executeControlCycle(s, 0.5, to_a);
        b.executeControlCycle(s, 0.5, to_b);
    }

    if (a.getPopulation().getCurrentSize() == 0 || b.getPopulation().getCurrentSize() == 0) {
        std::cout << "FAIL: pops\n";
        ++f;
    }

    // Sustained depletion with no maturation forces a replacement every cycle and tabu-lists failures.
    {
        odneat::OdneatParams harsh;
        harsh.maturation_period_cycles = 0;
        harsh.internal_population_size = 10;
        odneat::OdneatAgent x(10, 4, 2, 0.0, 100.0, 50.0, 0.0, harsh, 101ULL);
        odneat::OdneatAgent y(11, 4, 2, 0.0, 100.0, 50.0, 0.0, harsh, 202ULL);
        const int start_evals = x.getEvaluationCount();
        for (int i = 0; i < 5; ++i) {
            odneat::Genome bx;
            double be = 0.0;
            const bool wb = y.prepareBroadcast(bx, be);
            std::vector<odneat::ReceivedGenome> to_x;
            if (wb) to_x.push_back(odneat::ReceivedGenome{y.getActiveGenome(), y.getEnergy()});
            x.executeControlCycle(s, -1000.0, to_x);
            y.executeControlCycle(s, 0.5, {});
        }

        if (x.getEvaluationCount() != start_evals + 5) {
            std::cout << "FAIL: replacement cadence\n";
            ++f;
        }

        if (x.getTabuList().getTabuSize() == 0) {
            std::cout << "FAIL: failures remembered\n";
            ++f;
        }

        if (x.getEnergy() != 50.0) {
            std::cout << "FAIL: refill after replace\n";
            ++f;
        }
    }

    if (f == 0) std::cout << "test_integration passed\n";

    return f == 0 ? 0 : 1;
}
