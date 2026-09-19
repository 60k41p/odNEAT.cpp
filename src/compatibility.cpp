#include "compatibility.h"

#include <algorithm>
#include <cmath>

#include "config.h"
#include "genome.h"

namespace odneat {

    CompatibilityBreakdown analyseCompatibility(
        const Genome &first_genome, const Genome &second_genome, double disjoint_coefficient, double excess_coefficient, double weight_difference_coefficient) {
        CompatibilityBreakdown breakdown{};
        const std::vector<ConnectionGene> &first_connections = first_genome.getConnectionGenes();
        const std::vector<ConnectionGene> &second_connections = second_genome.getConnectionGenes();
        if (first_connections.empty() && second_connections.empty()) {
            breakdown.normaliser = 1.0;
            breakdown.distance = 0.0;
            return breakdown;
        }

        std::vector<ConnectionGene> sorted_first = first_connections;
        std::vector<ConnectionGene> sorted_second = second_connections;
        std::sort(sorted_first.begin(), sorted_first.end(),
                  [](const ConnectionGene &left, const ConnectionGene &right) { return left.innovation_id < right.innovation_id; });
        std::sort(sorted_second.begin(), sorted_second.end(),
                  [](const ConnectionGene &left, const ConnectionGene &right) { return left.innovation_id < right.innovation_id; });
        const InnovationId maximum_first = sorted_first.empty() ? InnovationId{} : sorted_first.back().innovation_id;
        const InnovationId maximum_second = sorted_second.empty() ? InnovationId{} : sorted_second.back().innovation_id;
        std::size_t first_index = 0;
        std::size_t second_index = 0;
        double weight_difference_sum = 0.0;

        while (first_index < sorted_first.size() && second_index < sorted_second.size()) {
            const ConnectionGene &first_gene = sorted_first[first_index];
            const ConnectionGene &second_gene = sorted_second[second_index];
            if (first_gene.innovation_id == second_gene.innovation_id) {
                ++breakdown.matching_count;
                weight_difference_sum += std::fabs(first_gene.weight - second_gene.weight);
                ++first_index;
                ++second_index;
            } else if (first_gene.innovation_id < second_gene.innovation_id) {
                if (maximum_second < first_gene.innovation_id) {
                    ++breakdown.excess_count;
                } else {
                    ++breakdown.disjoint_count;
                }

                ++first_index;
            } else {
                if (maximum_first < second_gene.innovation_id) {
                    ++breakdown.excess_count;
                } else {
                    ++breakdown.disjoint_count;
                }

                ++second_index;
            }
        }

        for (std::size_t remaining_index = first_index; remaining_index < sorted_first.size(); ++remaining_index) {
            if (maximum_second < sorted_first[remaining_index].innovation_id) {
                ++breakdown.excess_count;
            } else {
                ++breakdown.disjoint_count;
            }
        }

        for (std::size_t remaining_index = second_index; remaining_index < sorted_second.size(); ++remaining_index) {
            if (maximum_first < sorted_second[remaining_index].innovation_id) {
                ++breakdown.excess_count;
            } else {
                ++breakdown.disjoint_count;
            }
        }

        if (breakdown.matching_count > 0) {
            breakdown.mean_weight_difference = weight_difference_sum / static_cast<double>(breakdown.matching_count);
        }

        const std::size_t larger_size = (sorted_first.size() > sorted_second.size()) ? sorted_first.size() : sorted_second.size();

        if (larger_size < static_cast<std::size_t>(DefaultConstants::kCompatibilitySmallGenomeSize)) {
            breakdown.normaliser = 1.0;
        } else {
            breakdown.normaliser = static_cast<double>(larger_size);
        }

        breakdown.distance = (disjoint_coefficient * static_cast<double>(breakdown.disjoint_count) / breakdown.normaliser) +
                             (excess_coefficient * static_cast<double>(breakdown.excess_count) / breakdown.normaliser) +
                             (weight_difference_coefficient * breakdown.mean_weight_difference);

        return breakdown;
    }

    double computeCompatibilityDistance(
        const Genome &first_genome, const Genome &second_genome, double disjoint_coefficient, double excess_coefficient, double weight_difference_coefficient) {
        const CompatibilityBreakdown breakdown =
            analyseCompatibility(first_genome, second_genome, disjoint_coefficient, excess_coefficient, weight_difference_coefficient);
        return breakdown.distance;
    }

    bool areGenomesCompatible(const Genome &first_genome,
                              const Genome &second_genome,
                              double disjoint_coefficient,
                              double excess_coefficient,
                              double weight_difference_coefficient,
                              double compatibility_threshold) {
        const double distance =
            computeCompatibilityDistance(first_genome, second_genome, disjoint_coefficient, excess_coefficient, weight_difference_coefficient);
        return distance < compatibility_threshold;
    }

    std::vector<Species> speciatePopulation(const std::vector<Genome> &genomes,
                                            double disjoint_coefficient,
                                            double excess_coefficient,
                                            double weight_difference_coefficient,
                                            double compatibility_threshold) {
        std::vector<Species> species_list{};
        species_list.reserve(genomes.size());
        int next_species_id = 0;

        for (std::size_t genome_index = 0; genome_index < genomes.size(); ++genome_index) {
            const Genome &candidate_genome = genomes[genome_index];
            bool placed_in_species = false;

            for (Species &species : species_list) {
                const Genome &representative_genome = genomes[species.representative_index];
                if (areGenomesCompatible(candidate_genome, representative_genome, disjoint_coefficient, excess_coefficient, weight_difference_coefficient,
                                         compatibility_threshold)) {
                    species.member_indices.push_back(genome_index);
                    placed_in_species = true;
                    break;
                }
            }

            if (!placed_in_species) {
                Species fresh_species{};
                fresh_species.species_id = next_species_id++;
                fresh_species.representative_index = genome_index;
                fresh_species.member_indices.push_back(genome_index);
                fresh_species.average_adjusted_fitness = 0.0;
                species_list.push_back(fresh_species);
            }
        }

        return species_list;
    }

    void applyFitnessSharing(std::vector<Genome> &genomes, const std::vector<Species> &species_list) {
        for (const Species &species : species_list) {
            const double species_size = static_cast<double>(species.member_indices.size());

            if (species_size <= 0.0) {
                continue;
            }

            for (const std::size_t member_index : species.member_indices) {
                const double shared_fitness = genomes[member_index].getFitness() / species_size;
                genomes[member_index].setAdjustedFitness(shared_fitness);
            }
        }
    }

    void refreshSpeciesFitness(const std::vector<Genome> &genomes, std::vector<Species> &species_list) {
        for (Species &species : species_list) {
            if (species.member_indices.empty()) {
                species.average_adjusted_fitness = 0.0;
                continue;
            }

            double fitness_sum = 0.0;

            for (const std::size_t member_index : species.member_indices) {
                fitness_sum += genomes[member_index].getAdjustedFitness();
            }

            species.average_adjusted_fitness = fitness_sum / static_cast<double>(species.member_indices.size());
        }
    }

}  // namespace odneat
