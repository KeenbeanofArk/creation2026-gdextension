#include "Brain.h"
#include <cassert>
#include <cmath>
#include <deque>
#include <unordered_map>

// ============================================================
// build
// ============================================================
void Brain::build(const Genome &genome) {
	m_neurons.clear();
	m_synapses.clear();
	m_input_count = 0;
	m_output_count = 0;
	m_bias_idx = -1;
	m_valid = false;

	if (genome.nodes.empty())
		return;

	// --- Phase 1: collect all enabled nodes ---
	// Map node_id -> index in m_neurons
	std::unordered_map<int, int> id_to_idx;

	// Add in order: BIAS, INPUT, OUTPUT, HIDDEN
	// We'll fill topo_order during Kahn's pass
	auto add_neurons_of_type = [&](NodeType nt) {
		for (const auto &ng : genome.nodes) {
			if (!ng.enabled || ng.type != nt)
				continue;
			NeuronState ns;
			ns.node_id = ng.id;
			ns.type = ng.type;
			ns.activation = ng.activation;
			ns.value = 0.0f;
			ns.topo_order = -1;
			id_to_idx[ng.id] = static_cast<int>(m_neurons.size());
			m_neurons.push_back(ns);
		}
	};

	add_neurons_of_type(NodeType::BIAS);
	add_neurons_of_type(NodeType::INPUT);
	add_neurons_of_type(NodeType::HIDDEN);
	add_neurons_of_type(NodeType::OUTPUT);

	if (m_neurons.empty())
		return;

	// Count types
	for (int i = 0; i < static_cast<int>(m_neurons.size()); ++i) {
		switch (m_neurons[i].type) {
			case NodeType::INPUT:
				++m_input_count;
				break;
			case NodeType::OUTPUT:
				++m_output_count;
				break;
			case NodeType::BIAS:
				m_bias_idx = i;
				break;
			default:
				break;
		}
	}

	// --- Phase 2: collect enabled synapses, check endpoints exist ---
	for (const auto &cg : genome.connections) {
		if (!cg.enabled)
			continue;
		auto it_in = id_to_idx.find(cg.in_node_id);
		auto it_out = id_to_idx.find(cg.out_node_id);
		if (it_in == id_to_idx.end() || it_out == id_to_idx.end())
			continue;

		SynapseState ss;
		ss.from_idx = it_in->second;
		ss.to_idx = it_out->second;
		ss.weight = cg.weight;
		m_synapses.push_back(ss);
	}

	// --- Phase 3: topological sort (Kahn's algorithm) ---
	m_valid = topological_sort();
}

// ============================================================
// topological_sort (Kahn's algorithm)
// Re-orders m_neurons in place and returns false if cycle detected.
// ============================================================
bool Brain::topological_sort() {
	int N = static_cast<int>(m_neurons.size());

	// Build adjacency and in-degree over m_neurons indices
	// We treat BIAS and INPUT nodes as having no incoming edges
	std::vector<int> in_degree(N, 0);
	std::vector<std::vector<int>> adj(N); // adj[from] = list of to indices

	for (const auto &ss : m_synapses) {
		adj[ss.from_idx].push_back(ss.to_idx);
		++in_degree[ss.to_idx];
	}

	// Force BIAS/INPUT in-degree to 0 (may have been incremented erroneously)
	for (int i = 0; i < N; ++i) {
		if (m_neurons[i].type == NodeType::INPUT ||
				m_neurons[i].type == NodeType::BIAS)
			in_degree[i] = 0;
	}

	std::deque<int> ready;
	for (int i = 0; i < N; ++i)
		if (in_degree[i] == 0)
			ready.push_back(i);

	std::vector<NeuronState> sorted_neurons;
	std::vector<int> old_to_new(N, -1);
	sorted_neurons.reserve(N);

	while (!ready.empty()) {
		int cur = ready.front();
		ready.pop_front();
		old_to_new[cur] = static_cast<int>(sorted_neurons.size());
		sorted_neurons.push_back(m_neurons[cur]);

		for (int nb : adj[cur]) {
			--in_degree[nb];
			if (in_degree[nb] == 0)
				ready.push_back(nb);
		}
	}

	if (static_cast<int>(sorted_neurons.size()) != N) {
		// Cycle detected — mark invalid, zero outputs
		return false;
	}

	// Remap synapse indices to sorted order
	for (auto &ss : m_synapses) {
		ss.from_idx = old_to_new[ss.from_idx];
		ss.to_idx = old_to_new[ss.to_idx];
	}

	// Remap m_bias_idx
	if (m_bias_idx >= 0)
		m_bias_idx = old_to_new[m_bias_idx];

	m_neurons = std::move(sorted_neurons);
	return true;
}

// ============================================================
// evaluate
// ============================================================
std::vector<float> Brain::evaluate(const std::vector<float> &inputs) const {
	if (!m_valid)
		return std::vector<float>(m_output_count, 0.0f);
	if (static_cast<int>(inputs.size()) != m_input_count)
		return std::vector<float>(m_output_count, 0.0f);

	// Copy neuron states for this evaluation
	std::vector<float> values(m_neurons.size(), 0.0f);

	// Set input values
	int input_cursor = 0;
	for (int i = 0; i < static_cast<int>(m_neurons.size()); ++i) {
		if (m_neurons[i].type == NodeType::BIAS) {
			values[i] = 1.0f;
		} else if (m_neurons[i].type == NodeType::INPUT) {
			values[i] = inputs[input_cursor++];
		}
	}

	// Propagate in topological order
	for (const auto &ss : m_synapses)
		values[ss.to_idx] += values[ss.from_idx] * ss.weight;

	// Apply activation to non-input/bias neurons
	for (int i = 0; i < static_cast<int>(m_neurons.size()); ++i) {
		if (m_neurons[i].type == NodeType::HIDDEN ||
				m_neurons[i].type == NodeType::OUTPUT)
			values[i] = activate(values[i], m_neurons[i].activation);
	}

	// Collect output values (in order they appear in sorted array)
	std::vector<float> outputs;
	outputs.reserve(m_output_count);
	for (int i = 0; i < static_cast<int>(m_neurons.size()); ++i)
		if (m_neurons[i].type == NodeType::OUTPUT)
			outputs.push_back(values[i]);

	return outputs;
}

// ============================================================
// activate
// ============================================================
float Brain::activate(float x, ActivationType a) const {
	switch (a) {
		case ActivationType::TANH:
			return std::tanh(x);
		case ActivationType::SIGMOID:
			return 1.0f / (1.0f + std::exp(-x));
		case ActivationType::LINEAR:
			return x;
	}
	return x;
}
