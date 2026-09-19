/* Innovation identifiers for odNEAT genomes.
 *
 * This header defines the decentralised replacement for NEAT's global sequential innovation numbers. Each robot assigns local high-resolution timestamps to new
 * genes, which practically guarantees uniqueness while retaining chronology for matching disjoint versus excess genes during crossover and speciation.
 * Citations: Silva et al. (2015) "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Section 3 (bulleted differences) and
 * Section 3.2; Stanley and Miikkulainen (2002) "Evolving Neural Networks through Augmenting Topologies", Section 3 on historical markings.
 */

#ifndef ODNEAT_INNOVATION_CLOCK_H_
#define ODNEAT_INNOVATION_CLOCK_H_

#include <chrono>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

namespace odneat {

    // InnovationId uniquely labels a neuron gene or connection gene introduced during evolution.
    struct InnovationId {
        // Identifier of the robot that created the gene; breaks timestamp ties across robots.
        std::uint32_t robot_identifier = 0;
        // Nanoseconds since Unix epoch (system clock) at creation time; preserves chronological order.
        // Minimal-genome genes use robot 0 with small timestamps; evolved genes use system time (~1.7e18 ns).
        std::uint64_t timestamp_nanoseconds = 0;
        // Monotonic per-robot counter; guarantees uniqueness when several genes share one timestamp.
        std::uint32_t local_counter = 0;

        // Equality is field-wise.
        bool operator==(const InnovationId &other) const = default;
        // Strict timestamp-first ordering used to distinguish disjoint from excess genes.
        // Timestamp dominates so cross-robot comparison stays chronological; robot/counter break ties.
        auto operator<=>(const InnovationId &other) const noexcept {
            if (timestamp_nanoseconds != other.timestamp_nanoseconds) {
                return timestamp_nanoseconds <=> other.timestamp_nanoseconds;
            }

            if (robot_identifier != other.robot_identifier) {
                return robot_identifier <=> other.robot_identifier;
            }

            return local_counter <=> other.local_counter;
        }
    };

    // Hash for InnovationId so genome maps key on the full 96-bit identity (no truncation collisions).
    struct InnovationIdHash {
        std::size_t operator()(const InnovationId &innovation_id) const noexcept {
            std::size_t seed = std::hash<std::uint32_t>{}(innovation_id.robot_identifier);
            seed ^= std::hash<std::uint64_t>{}(innovation_id.timestamp_nanoseconds) + 0x9E3779B97F4A7C15ULL + (seed << 6U) + (seed >> 2U);
            seed ^= std::hash<std::uint32_t>{}(innovation_id.local_counter) + 0x9E3779B97F4A7C15ULL + (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };

    // InnovationClock mints fresh InnovationId values for one robot.
    // Citation: Silva et al. (2015), Section 3: local high-resolution timestamps assigned by each robot.
    class InnovationClock {
       public:
        // Constructs a clock for the given robot; counter starts at zero, timestamps use system clock.
        explicit InnovationClock(std::uint32_t robot_identifier);
        // Restores a clock with persisted counter/timestamp to avoid collisions across restarts.
        InnovationClock(std::uint32_t robot_identifier, std::uint32_t initial_counter, std::uint64_t initial_last_timestamp);

        // Returns a new unique identifier using system-clock time (monotonic) plus the counter.
        InnovationId nextInnovationId();
        // Returns a deterministic identifier for a fixed timestamp; used by tests and seeded replays.
        InnovationId innovationIdAt(std::uint64_t timestamp_nanoseconds);
        // Returns the robot identifier owned by this clock.
        std::uint32_t getRobotIdentifier() const;
        // Returns how many identifiers this clock has minted so far.
        std::uint32_t getMintedCount() const;
        // Returns the last timestamp minted (for persistence).
        std::uint64_t getLastTimestamp() const;
        // Restores persisted state so restarts never re-mint colliding identifiers.
        void restoreState(std::uint32_t minted_count, std::uint64_t last_timestamp);
        // Exports (minted_count, last_timestamp) for persistence.
        std::pair<std::uint32_t, std::uint64_t> exportState() const;

       private:
        // Robot identifier stamped into every minted InnovationId.
        std::uint32_t robot_identifier_;
        // Monotonic counter ensuring uniqueness within a single timestamp tick.
        std::uint32_t local_counter_;
        // Last timestamp minted; next ids are forced strictly greater to stay monotonic.
        std::uint64_t last_timestamp_nanoseconds_;
        // Current system time in nanos since Unix epoch.
        static std::uint64_t currentSystemNanos();
    };

}  // namespace odneat

#endif  // ODNEAT_INNOVATION_CLOCK_H_
