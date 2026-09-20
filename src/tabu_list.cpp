#include "tabu_list.h"

#include "compatibility.h"

namespace odneat {

    TabuList::TabuList(
        double disjoint_coefficient, double excess_coefficient, double weight_difference_coefficient, double kCompatibilityThreshold, int recent_history_size)
        : disjoint_coefficient_(disjoint_coefficient),
          excess_coefficient_(excess_coefficient),
          weight_difference_coefficient_(weight_difference_coefficient),
          compatibility_threshold_(kCompatibilityThreshold),
          recent_history_size_(recent_history_size) {}

    bool TabuList::approvesCandidate(const Genome &candidate_genome) const {
        for (const Genome &tabu_genome : tabu_genomes_) {
            if (areGenomesCompatible(candidate_genome, tabu_genome, disjoint_coefficient_, excess_coefficient_, weight_difference_coefficient_,
                                     compatibility_threshold_)) {
                return false;
            }
        }

        return true;
    }

    void TabuList::addTabuGenome(const Genome &failed_genome) { tabu_genomes_.push_back(failed_genome); }

    void TabuList::observeReceivedGenome(const Genome &received_genome) {
        recent_history_.push_back(received_genome);
        while (static_cast<int>(recent_history_.size()) > recent_history_size_) {
            recent_history_.pop_front();
        }

        expireStaleEntries();
    }

    void TabuList::expireStaleEntries() {
        std::vector<Genome> surviving_genomes{};
        surviving_genomes.reserve(tabu_genomes_.size());

        for (const Genome &tabu_genome : tabu_genomes_) {
            bool has_similar_recent = false;

            for (const Genome &recent_genome : recent_history_) {
                if (areGenomesCompatible(tabu_genome, recent_genome, disjoint_coefficient_, excess_coefficient_, weight_difference_coefficient_,
                                         compatibility_threshold_)) {
                    has_similar_recent = true;
                    break;
                }
            }

            if (has_similar_recent || recent_history_.empty()) {
                surviving_genomes.push_back(tabu_genome);
            }
        }

        tabu_genomes_ = surviving_genomes;
    }

    const std::vector<Genome> &TabuList::getTabuGenomes() const { return tabu_genomes_; }

    const std::deque<Genome> &TabuList::getRecentHistory() const { return recent_history_; }

    std::size_t TabuList::getTabuSize() const { return tabu_genomes_.size(); }

    void TabuList::clearTabuList() {
        tabu_genomes_.clear();
        recent_history_.clear();
    }

    void TabuList::restoreState(const std::vector<Genome> &tabu_genomes, const std::vector<Genome> &recent_history) {
        tabu_genomes_ = tabu_genomes;
        recent_history_.clear();
        std::size_t keep_from = std::size_t{0};
        if (recent_history.size() > static_cast<std::size_t>(recent_history_size_)) {
            keep_from = recent_history.size() - static_cast<std::size_t>(recent_history_size_);
        }
        for (std::size_t genome_index = keep_from; genome_index < recent_history.size(); ++genome_index) {
            recent_history_.push_back(recent_history[genome_index]);
        }
    }

}  // namespace odneat
