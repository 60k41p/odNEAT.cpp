/* Mutation operators for weights and topology augmentation (Stanley bias-neuron model).
 *
 * This header implements parsimonious complexification: Gaussian perturbation of weights plus rare addition of one connection or one neuron by
 * splitting an existing connection. New genes receive decentralised timestamp innovation identifiers so chronology is retained. Citations: Silva et al. (2015)
 * "odNEAT: An Algorithm for Decentralised Online Evolution of Robotic Controllers", Sections 2.2, 3 and 4.1 (rates: mutation 0.1, add neuron 0.03, add
 * connection 0.05, magnitude 0.5); Stanley and Miikkulainen (2002) "Evolving Neural Networks through Augmenting Topologies", Section 3.4 on structural
 * mutation.
 */

#ifndef ODNEAT_MUTATION_H_
#define ODNEAT_MUTATION_H_

#include <random>

namespace odneat {

    class Genome;
    class InnovationClock;
    struct OdneatParams;

    // MutationOutcome counts which mutation operators fired for one offspring, useful for tests and logging.
    struct MutationOutcome {
        // Number of connection weights perturbed.
        int perturbed_weights = 0;
        // Deprecated (Stanley bias-neuron model has no per-neuron bias); always 0 for API compat.
        int perturbed_biases = 0;
        // Whether a new connection gene was added.
        bool added_connection = false;
        // Whether a new neuron gene was added by splitting a connection.
        bool added_neuron = false;
        // Whether any enabled flag was toggled.
        bool toggled_enabled = false;
    };

    // Perturbs weights, toggles enabled flags and augments topology according to the given rates.
    void mutateGenome(
        Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator, const OdneatParams &parameters, MutationOutcome *outcome = nullptr);
    // Adds one connection between two previously unconnected neurons with a random weight.
    bool addRandomConnection(Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator, double weight_minimum, double weight_maximum);
    // Adds one hidden neuron by splitting a random enabled connection (old edge disabled, two new edges added).
    bool addRandomNeuron(Genome &genome, InnovationClock &innovationClock, std::mt19937_64 &randomGenerator);
    // Generates one offspring from one or two parents following odNEAT rates: crossover with probability crossover_rate, then mutation.
    Genome generateOffspring(const Genome &first_parent,
                             const Genome *second_parent,
                             InnovationClock &innovationClock,
                             std::mt19937_64 &randomGenerator,
                             const OdneatParams &parameters);

}  // namespace odneat

#endif  // ODNEAT_MUTATION_H_
