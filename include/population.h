/* Internal population of genomes maintained independently by each robot.
 *
 * This header stores the robot-local set of candidate solutions subject to speciation and fitness sharing. It rejects duplicate genomes (merging their fitness
 * instead), evicts the worst adjusted genome when full, and selects parent species proportionally to average adjusted fitness for both inter-robot broadcast
 * and intra-robot reproduction. Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers",
 * Sections 3.2, 3.3 and Algorithm 1.
 */

#ifndef ODNEAT_POPULATION_H_
#define ODNEAT_POPULATION_H_

#include <cstddef>
#include <random>
#include <vector>

#include "compatibility.h"
#include "genome.h"

namespace odneat {

    struct OdneatParams;

    // PopulationAcceptance explains why a received genome was accepted or rejected.
    enum class PopulationAcceptance : unsigned char {
        // Genome added as a novel entry.
        kAccepted = 0,
        // Genome already present; fitness merged instead of duplicating.
        kDuplicateMerged = 1,
        // Genome rejected (reserved for future capacity policies).
        kRejected = 2
    };

    // InternalPopulation stores up to max_population_size genomes with their species partition.
    class InternalPopulation {
       public:
        // Constructs an empty population honouring the given capacity and speciation settings.
        InternalPopulation(int maximum_population_size,
                           double disjoint_coefficient,
                           double excess_coefficient,
                           double weight_difference_coefficient,
                           double kCompatibilityThreshold);
        // Returns the stored genomes in insertion order.
        const std::vector<Genome> &getGenomes() const;
        // Returns the current species partition.
        const std::vector<Species> &getSpecies() const;
        // Returns the maximum capacity of the population.
        int getMaximumSize() const;
        // Returns the current number of stored genomes.
        std::size_t getCurrentSize() const;
        // Returns true when the population holds no genomes.
        bool isEmpty() const;
        // Returns true when the population reached its capacity.
        bool isFull() const;
        // Inserts a genome, merging fitness on duplicates and evicting the worst genome when full.
        PopulationAcceptance addGenome(const Genome &candidate_genome);
        // Inserts a genome and returns the evicted genome when capacity forced a removal; has_eviction reports it.
        PopulationAcceptance addGenomeWithEviction(const Genome &candidate_genome, Genome &evicted_genome, bool &has_eviction);
        // Merges a remote energy sample into the matching stored genome; returns true when a match existed.
        bool mergeDuplicateFitness(const Genome &received_genome, double remote_energy);
        // Returns true when an identical genome is already stored.
        bool containsIdenticalGenome(const Genome &candidate_genome) const;
        // Re-speciates all genomes and refreshes fitness sharing statistics.
        void respecialiseAndShareFitness();
        // Selects a parent species index proportionally to average adjusted fitness (Equation 2); -1 when empty.
        int selectParentSpecies(std::mt19937_64 &randomGenerator) const;
        // Runs tournament selection of size two inside one species and returns the winning genome index.
        std::size_t tournamentSelectInSpecies(int species_index, std::mt19937_64 &randomGenerator) const;
        // Returns the sum of average adjusted fitness values used as Equation 2 denominator.
        double computeTotalAverageFitness() const;
        // Returns the average adjusted fitness of the species owning the given genome index, or 0 when absent.
        double findSpeciesFitnessForGenome(std::size_t genome_index) const;
        // Removes the genome with the lowest adjusted fitness; returns false when empty.
        bool removeWorstGenome(Genome &removed_genome);
        // Clears all genomes and species.
        void clearPopulation();
        // Enables/disables the niching scheme (paper Sec. 6.2 "No niching" ablation):
        // disabled means one shared species with no fitness sharing (adjusted = raw fitness).
        void setNichingEnabled(bool niching_enabled);
        // Returns whether speciation with fitness sharing is active.
        bool isNichingEnabled() const;
        // Refreshes fitness sharing statistics without re-speciating (exact: the partition
        // depends on topology only, which this does not change).
        void refreshFitnessSharing();
        // Overwrites the stored copy matching the given genome's topology with fresh fitness
        // statistics; returns false when no identical genome is stored.
        bool syncStoredFitness(const Genome &genome, double fitness, int evaluation_count);

       private:
        // Maximum number of genomes retained locally (paper default 40).
        int maximum_population_size_;
        // NEAT disjoint coefficient c2.
        double disjoint_coefficient_;
        // NEAT excess coefficient c1.
        double excess_coefficient_;
        // NEAT weight difference coefficient c3.
        double weight_difference_coefficient_;
        // Speciation threshold delta.
        double compatibility_threshold_;
        // Stored genomes in insertion order.
        std::vector<Genome> genomes_;
        // Current species partition over genomes_.
        std::vector<Species> species_list_;
        // Whether speciation with fitness sharing applies (false = single shared species).
        bool niching_enabled_ = true;
        // Refreshes species_list_ and fitness sharing after every mutation of genomes_.
        void refreshSpeciation();
    };

}  // namespace odneat

#endif  // ODNEAT_POPULATION_H_
