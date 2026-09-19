/* Per-robot odNEAT agent executing Algorithm 1 independently on every robot.
 *
 * This header owns the active genome and decoded controller, virtual energy and fitness averaging, internal population, tabu list, maturation guard and
 * broadcast sampling. Each control cycle follows Algorithm 1 order: maybe broadcast, incorporate received genomes through tabu and population filters, operate
 * in the environment and update energy, then replace the controller only on failure outside maturation. Citations: Silva et al. (2015) "odNEAT: An Algorithm
 * for Decentralised Online Evolution of Robotic Controllers", Sections 3.1, 3.2, 3.3 and Algorithm 1 (pseudocode running independently on every robot).
 */

#ifndef ODNEAT_AGENT_H_
#define ODNEAT_AGENT_H_

#include <cstddef>
#include <random>
#include <vector>

#include "broadcast.h"
#include "config.h"
#include "energy.h"
#include "genome.h"
#include "innovation_clock.h"
#include "network.h"
#include "population.h"
#include "tabu_list.h"

namespace odneat {

    // ReceivedGenome pairs an incoming genome copy with the sender-observed energy used for incremental averaging.
    struct ReceivedGenome {
        // Copy of the sender's active genome.
        Genome genome{};
        // Sender's current virtual energy level attached to the broadcast.
        double sender_energy = 0.0;
    };

    // AgentStepResult reports what the agent did during one control cycle for logging and simulation wiring.
    struct AgentStepResult {
        // Whether the agent chose to broadcast its active genome this cycle.
        bool did_broadcast = false;
        // Number of received genomes accepted into the internal population.
        int accepted_genome_count = 0;
        // Number of received genomes merged as duplicates.
        int merged_duplicate_count = 0;
        // Number of received genomes rejected by the tabu list.
        int tabu_rejected_count = 0;
        // Whether the active controller was replaced after energy depletion.
        bool did_replace_controller = false;
        // Energy delta applied this cycle.
        double applied_energy_delta = 0.0;
        // Signed wheel speeds in [-1, 1] produced by the controller.
        double left_wheel_speed = 0.0;
        double right_wheel_speed = 0.0;
    };

    // OdneatAgent runs the full online evolution loop for a single robot.
    class OdneatAgent {
       public:
        // Constructs an agent with robot identity, controller dimensions, energy bounds and evolutionary parameters.
        OdneatAgent(std::uint32_t robot_identifier,
                    int input_count,
                    int output_count,
                    double minimum_energy,
                    double maximum_energy,
                    double default_energy,
                    double minimum_threshold,
                    const OdneatParams &parameters,
                    std::uint64_t random_seed);
        // Returns the robot identifier.
        std::uint32_t getRobotIdentifier() const;
        // Returns the active genome currently decoded into the controller.
        const Genome &getActiveGenome() const;
        // Returns the current virtual energy level.
        double getEnergy() const;
        // Returns the current fitness estimate (mean sampled energy).
        double getFitness() const;
        // Returns the internal population of candidate solutions.
        const InternalPopulation &getPopulation() const;
        // Returns the tabu memory of recent poor solutions.
        const TabuList &getTabuList() const;
        // Returns how many control cycles remain in the maturation protection period.
        int getMaturationCyclesRemaining() const;
        // Returns how many controllers this robot has evaluated (including the initial one).
        int getEvaluationCount() const;
        // Returns the mutable random generator for task and channel code sharing the stream.
        std::mt19937_64 &accessRandomGenerator();
        // Executes one Algorithm 1 control cycle given normalised sensor inputs and an energy delta from the task.
        // Legacy single-phase entry (broadcast+incorporate+step+energy); driver uses split API below for exact timing.
        AgentStepResult executeControlCycle(const std::vector<double> &sensor_inputs, double energy_delta, const std::vector<ReceivedGenome> &received_genomes);
        // Decides broadcast, packages the active genome with current energy when chosen (exactly one draw per cycle).
        bool prepareBroadcast(Genome &broadcast_genome, double &broadcast_energy);
        // Split-phase API for exact paper timing (avoids double broadcast draw and actuation lag):
        // incorporate -> stepController -> driver computes energy from fresh wheels -> updateEnergy.
        void incorporateExternal(const std::vector<ReceivedGenome> &received_genomes, int &accepted_count, int &merged_count, int &rejected_count);
        // Steps the active controller only; returns raw network outputs in [0,1], no energy update.
        std::vector<double> stepController(const std::vector<double> &sensor_inputs);
        // Applies task energy, samples fitness, ticks maturation and replaces on depletion; returns replacement.
        bool updateEnergyAndMaybeReplace(double energy_delta);
        // Forces controller replacement regardless of energy (used by tests and fault injection).
        void forceControllerReplacement();
        // Enables or disables genome exchange (ablation: exchange off discards received genomes after counting).
        void setExchangeEnabled(bool exchange_enabled);
        // Enables or disables the tabu filter (ablation: tabu off accepts everything population-acceptable).
        void setTabuEnabled(bool tabu_enabled);
        // Enables or disables the maturation guard (ablation: no maturation allows immediate replacement).
        void setMaturationEnabled(bool maturation_enabled);
        // Enables or disables speciation (ablation: single species, no fitness sharing adjustment).
        void setSpeciationEnabled(bool speciation_enabled);

       private:
        // Unique robot identity stamped into innovations and logs.
        std::uint32_t robot_identifier_;
        // Number of sensor inputs expected by the controller.
        int input_count_;
        // Number of motor outputs produced by the controller.
        int output_count_;
        // Evolutionary rates and population limits.
        OdneatParams parameters_;
        // Deterministic random stream for broadcast, selection, crossover and mutation.
        std::mt19937_64 random_generator_;
        // Local timestamp clock minting innovation identifiers.
        InnovationClock innovation_clock_;
        // Active genotype decoded into the running controller.
        Genome active_genome_;
        // Executable phenotype of the active genome.
        RecurrentNetwork active_controller_;
        // Virtual energy level reflecting recent task performance.
        EnergyTracker energy_tracker_;
        // Running mean of energy samples forming the fitness score.
        FitnessAverager fitness_averager_;
        // Robot-local candidate solutions under speciation and fitness sharing.
        InternalPopulation population_;
        // Short-term memory of failed solutions filtering received genomes.
        TabuList tabu_list_;
        // Unilateral broadcast sampler implementing Equation 2.
        BroadcastPolicy broadcast_policy_;
        // Remaining protected cycles before the controller may be replaced.
        int maturation_cycles_remaining_;
        // Number of controllers evaluated so far.
        int evaluation_count_;
        // Ablation switches.
        bool exchange_enabled_;
        bool tabu_enabled_;
        bool maturation_enabled_;
        bool speciation_enabled_;
        // Incorporates received genomes through tabu and population filters, returning accepted and rejected counts.
        void incorporateReceivedGenomes(const std::vector<ReceivedGenome> &received_genomes, int &accepted_count, int &merged_count, int &rejected_count);
        // Creates an offspring from the population, installs it and refills energy and maturation.
        void replaceControllerWithOffspring();
        // Re-decodes the active genome after replacement and resets runtime state.
        void reinstallActiveController();
    };

}  // namespace odneat

#endif  // ODNEAT_AGENT_H_
