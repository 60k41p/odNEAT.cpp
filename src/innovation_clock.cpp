#include "innovation_clock.h"

namespace odneat {

    InnovationClock::InnovationClock(std::uint32_t robot_identifier) : robot_identifier_(robot_identifier), local_counter_(0), last_timestamp_nanoseconds_(0) {}

    InnovationClock::InnovationClock(std::uint32_t robot_identifier, std::uint32_t initial_counter, std::uint64_t initial_last_timestamp)
        : robot_identifier_(robot_identifier), local_counter_(initial_counter), last_timestamp_nanoseconds_(initial_last_timestamp) {}

    std::uint64_t InnovationClock::currentSystemNanos() {
        const auto now = std::chrono::system_clock::now();
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    }

    InnovationId InnovationClock::nextInnovationId() {
        std::uint64_t timestamp_nanoseconds = currentSystemNanos();
        if (timestamp_nanoseconds <= last_timestamp_nanoseconds_) {
            timestamp_nanoseconds = last_timestamp_nanoseconds_ + 1;
        }

        last_timestamp_nanoseconds_ = timestamp_nanoseconds;
        InnovationId fresh_id{};
        fresh_id.robot_identifier = robot_identifier_;
        fresh_id.timestamp_nanoseconds = timestamp_nanoseconds;
        fresh_id.local_counter = local_counter_;
        ++local_counter_;

        return fresh_id;
    }

    InnovationId InnovationClock::innovationIdAt(std::uint64_t timestamp_nanoseconds) {
        if (timestamp_nanoseconds <= last_timestamp_nanoseconds_) {
            timestamp_nanoseconds = last_timestamp_nanoseconds_ + 1;
        }

        last_timestamp_nanoseconds_ = timestamp_nanoseconds;
        InnovationId deterministic_id{};
        deterministic_id.robot_identifier = robot_identifier_;
        deterministic_id.timestamp_nanoseconds = timestamp_nanoseconds;
        deterministic_id.local_counter = local_counter_;
        ++local_counter_;

        return deterministic_id;
    }

    std::uint32_t InnovationClock::getRobotIdentifier() const { return robot_identifier_; }

    std::uint32_t InnovationClock::getMintedCount() const { return local_counter_; }

    std::uint64_t InnovationClock::getLastTimestamp() const { return last_timestamp_nanoseconds_; }

    void InnovationClock::restoreState(std::uint32_t minted_count, std::uint64_t last_timestamp) {
        local_counter_ = minted_count;
        if (last_timestamp > last_timestamp_nanoseconds_) {
            last_timestamp_nanoseconds_ = last_timestamp;
        }
    }

    std::pair<std::uint32_t, std::uint64_t> InnovationClock::exportState() const { return {local_counter_, last_timestamp_nanoseconds_}; }

}  // namespace odneat
