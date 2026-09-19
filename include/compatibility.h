/* Speciation by genomic compatibility distance with fitness sharing.
 *
 * This header protects topological innovations by clustering similar genomes into species and sharing fitness within each niche, so young small species are not
 * discarded prematurely. Distance follows NEAT: delta = c1*E/N + c2*D/N + c3*mean weight difference of matching genes, with disjoint inside and excess outside
 * the innovation range. Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 2.2, 3
 * and 5.1; Stanley and Miikkulainen (2002) "Evolving Neural Networks through Augmenting Topologies", Section 3.3 on speciation.
 */

#ifndef ODNEAT_COMPATIBILITY_H_
#define ODNEAT_COMPATIBILITY_H_

#include <cstddef>
#include <vector>

namespace odneat {

    class Genome;

    // CompatibilityBreakdown exposes the disjoint, excess and weight components of one distance computation for tests and analysis.
    struct CompatibilityBreakdown {
        // Number of disjoint connection genes (non-matching inside the shared innovation range).
        std::size_t disjoint_count = 0;
        // Number of excess connection genes (non-matching outside the shared innovation range).
        std::size_t excess_count = 0;
        // Mean absolute weight difference over matching connection genes, zero when none match.
        double mean_weight_difference = 0.0;
        // Number of matching connection genes sharing an InnovationId.
        std::size_t matching_count = 0;
        // Normalisation factor N (larger genome size, or 1 for small genomes).
        double normaliser = 1.0;
        // Final weighted distance value.
        double distance = 0.0;
    };

    // Computes the NEAT compatibility distance between two genomes using the given coefficients.
    double computeCompatibilityDistance(
        const Genome &first_genome, const Genome &second_genome, double disjoint_coefficient, double excess_coefficient, double weight_difference_coefficient);
    // Computes the distance and returns every intermediate component for inspection.
    CompatibilityBreakdown analyseCompatibility(
        const Genome &first_genome, const Genome &second_genome, double disjoint_coefficient, double excess_coefficient, double weight_difference_coefficient);
    // Returns true when the distance between two genomes is below the speciation threshold.
    bool areGenomesCompatible(const Genome &first_genome,
                              const Genome &second_genome,
                              double disjoint_coefficient,
                              double excess_coefficient,
                              double weight_difference_coefficient,
                              double compatibility_threshold);

    // Species is one niche of similar genomes sharing fitness.
    // Citation: Stanley and Miikkulainen (2002), Section 3.3 on explicit fitness sharing.
    struct Species {
        // Stable species identifier for logging and parent selection.
        int species_id = 0;
        // Indices into the population vector of member genomes.
        std::vector<std::size_t> member_indices{};
        // Index of the representative genome used for compatibility tests.
        std::size_t representative_index = 0;
        // Mean adjusted fitness of members; drives broadcast and parent-species selection (Eq. 2).
        double average_adjusted_fitness = 0.0;
    };

    // Assigns each genome index to the first compatible species or creates a new species.
    std::vector<Species> speciatePopulation(const std::vector<Genome> &genomes,
                                            double disjoint_coefficient,
                                            double excess_coefficient,
                                            double weight_difference_coefficient,
                                            double compatibility_threshold);
    // Applies explicit fitness sharing: adjusted_fitness = fitness / species_size for every genome.
    void applyFitnessSharing(std::vector<Genome> &genomes, const std::vector<Species> &species_list);
    // Recomputes average_adjusted_fitness for every species from current adjusted fitness values.
    void refreshSpeciesFitness(const std::vector<Genome> &genomes, std::vector<Species> &species_list);

}  // namespace odneat

#endif  // ODNEAT_COMPATIBILITY_H_
