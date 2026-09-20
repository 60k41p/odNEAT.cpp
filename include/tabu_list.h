/* Short-term tabu memory of recent poor solutions.
 *
 * This header filters received genomes that are topologically similar to controllers that failed recently, preventing flooding with poor solutions and cycling
 * in unfruitful neighbourhoods. Entries expire when no similar genome appears among the most recently received genomes. Citations: Silva et al. (2015) "odNEAT:
 * An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 3.2 and 4.1 (expiry over last 50 received genomes) and Algorithm 1.
 */

#ifndef ODNEAT_TABU_LIST_H_
#define ODNEAT_TABU_LIST_H_

#include <deque>
#include <vector>

#include "genome.h"

namespace odneat {

    // TabuList stores failed genomes plus a sliding window of recently received genomes for expiry.
    class TabuList {
       public:
        // Constructs a tabu list with speciation coefficients, threshold and recent-history window.
        TabuList(double disjoint_coefficient,
                 double excess_coefficient,
                 double weight_difference_coefficient,
                 double kCompatibilityThreshold,
                 int recent_history_size);
        // Returns true when the candidate is dissimilar to every tabu entry and may enter the population.
        bool approvesCandidate(const Genome &candidate_genome) const;
        // Records a failed or evicted genome in tabu memory.
        void addTabuGenome(const Genome &failed_genome);
        // Records a received genome in the recent history and expires stale tabu entries.
        void observeReceivedGenome(const Genome &received_genome);
        // Removes tabu entries with no similar genome in recent history.
        void expireStaleEntries();
        // Returns the stored tabu genomes.
        const std::vector<Genome> &getTabuGenomes() const;
        // Returns the sliding window of recently received genomes.
        const std::deque<Genome> &getRecentHistory() const;
        // Returns the number of tabu entries.
        std::size_t getTabuSize() const;
        // Clears tabu memory and recent history.
        void clearTabuList();
        // Restores exact persisted tabu and recent-history lists (checkpoint resume):
        // replaces contents verbatim without running expiry; recent history beyond
        // the window keeps only the most recent entries.
        void restoreState(const std::vector<Genome> &tabu_genomes, const std::vector<Genome> &recent_history);

       private:
        // NEAT disjoint coefficient c2 for similarity tests.
        double disjoint_coefficient_;
        // NEAT excess coefficient c1 for similarity tests.
        double excess_coefficient_;
        // NEAT weight difference coefficient c3 for similarity tests.
        double weight_difference_coefficient_;
        // Similarity threshold; candidates closer than this to any tabu entry are rejected.
        double compatibility_threshold_;
        // Maximum number of received genomes retained for expiry decisions (paper default 50).
        int recent_history_size_;
        // Failed genomes remembered as forbidden neighbourhoods.
        std::vector<Genome> tabu_genomes_;
        // Sliding window of most recently received genomes.
        std::deque<Genome> recent_history_;
    };

}  // namespace odneat

#endif  // ODNEAT_TABU_LIST_H_
