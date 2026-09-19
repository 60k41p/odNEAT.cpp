/* Crossover of NEAT genomes aligned by historical markings.
 *
 * This header recombines two parent genomes without costly topological analysis: matching genes (shared InnovationId) align directly, while disjoint and excess
 * genes are inherited from the fitter parent, or from both parents on fitness ties. Disabled genes propagate with a configurable probability. Citations: Silva
 * et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 2.2 and 3.3; Stanley and Miikkulainen (2002)
 * "Evolving Neural Networks through Augmenting Topologies", Section 3.2 on crossover.
 */

#ifndef ODNEAT_CROSSOVER_H_
#define ODNEAT_CROSSOVER_H_

#include <random>

namespace odneat {

    class Genome;

    // CrossoverOutcome summarises how many genes of each class the offspring received.
    struct CrossoverOutcome {
        // Number of matching genes randomly chosen between parents.
        int matching_genes = 0;
        // Number of disjoint or excess genes inherited from the fitter (or both) parents.
        int disjoint_excess_genes = 0;
        // Number of inherited connections left disabled in the offspring.
        int disabled_genes = 0;
    };

    // Recombines two parents into one offspring genome; fitness decides disjoint/excess inheritance on non-ties.
    // The fitter parent is passed first; pass equal fitness to inherit disjoint/excess genes from both parents.
    Genome crossoverGenomes(const Genome &fitter_parent,
                            const Genome &weaker_parent,
                            std::mt19937_64 &randomGenerator,
                            double disabled_inheritance_rate,
                            CrossoverOutcome *outcome = nullptr);

}  // namespace odneat

#endif  // ODNEAT_CROSSOVER_H_
