#pragma once
#include "Genome.h"
#include <vector>

// ============================================================
// Brain  –  NEAT feedforward neural network
// ============================================================

struct NeuronState {
	int node_id = 0;
	NodeType type = NodeType::HIDDEN;
	ActivationType activation = ActivationType::TANH;
	float value = 0.0f;
	int topo_order = 0;
};

struct SynapseState {
	int from_idx = 0; // index into m_neurons (after topo sort)
	int to_idx = 0;
	float weight = 0.0f;
};

class Brain {
public:
	Brain() = default;

	// Build the evaluation structure from a genome.
	// Must be called whenever topology changes (once per generation rebuild).
	void build(const Genome &genome);

	// Evaluate the network given an input vector.
	// inputs.size() must equal input_count().
	// Returns output vector of size output_count().
	// Returns all-zero outputs if !is_valid() or size mismatch.
	std::vector<float> evaluate(const std::vector<float> &inputs) const;

	int input_count() const { return m_input_count; }
	int output_count() const { return m_output_count; }
	int neuron_count() const { return static_cast<int>(m_neurons.size()); }
	int synapse_count() const { return static_cast<int>(m_synapses.size()); }
	bool is_valid() const { return m_valid; }

private:
	bool topological_sort(); // Kahn's algorithm; returns false if cycle detected
	float activate(float x, ActivationType a) const;

	std::vector<NeuronState> m_neurons; // topo-sorted: inputs first, then hidden, then outputs
	std::vector<SynapseState> m_synapses;
	int m_input_count = 0;
	int m_output_count = 0;
	int m_bias_idx = -1; // index of BIAS neuron in m_neurons (-1 if absent)
	bool m_valid = false;
};
