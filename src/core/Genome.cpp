#include "Genome.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <unordered_set>
#include <cstdio>

// ============================================================
// Helpers
// ============================================================
static uint32_t xorshift32(uint32_t &s) {
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;
	return s;
}

float Genome::rand_float(uint32_t &rng) {
	return static_cast<float>(xorshift32(rng)) / 4294967295.0f;
}

float Genome::rand_range(uint32_t &rng, float lo, float hi) {
	return lo + rand_float(rng) * (hi - lo);
}

int Genome::rand_int(uint32_t &rng, int lo, int hi_inclusive) {
	if (lo >= hi_inclusive)
		return lo;
	return lo + static_cast<int>(xorshift32(rng) % static_cast<uint32_t>(hi_inclusive - lo + 1));
}

// ============================================================
// InnovationTracker
// ============================================================
int InnovationTracker::get_or_create(int in_node, int out_node) {
	for (auto &[a, b, innov] : m_this_gen) {
		if (a == in_node && b == out_node)
			return innov;
	}
	int n = m_next_innovation++;
	m_this_gen.emplace_back(in_node, out_node, n);
	return n;
}

void InnovationTracker::new_generation() {
	m_this_gen.clear();
}

int InnovationTracker::next_node_id() {
	return m_next_node_id++;
}

// ============================================================
// Genome helpers
// ============================================================
NodeGene *Genome::find_node(int id) {
	for (auto &n : nodes)
		if (n.id == id)
			return &n;
	return nullptr;
}

const NodeGene *Genome::find_node(int id) const {
	for (const auto &n : nodes)
		if (n.id == id)
			return &n;
	return nullptr;
}

bool Genome::connection_exists(int in_id, int out_id) const {
	for (const auto &c : connections)
		if (c.in_node_id == in_id && c.out_node_id == out_id)
			return true;
	return false;
}

bool Genome::is_leaf_segment(int seg_id) const {
	for (const auto &s : segments)
		if (s.enabled && s.parent_id == seg_id)
			return false;
	return true;
}

int Genome::active_segment_count() const {
	int count = 0;
	for (const auto &s : segments)
		if (s.enabled)
			++count;
	return count;
}

int Genome::neural_input_count() const {
	return active_segment_count() * 9 + 5;
}

int Genome::neural_output_count() const {
	// One output per non-root active joint
	int count = 0;
	for (const auto &s : segments)
		if (s.enabled && s.parent_id >= 0)
			++count;
	return count;
}

// DFS reachability check for DAG validation
bool Genome::can_reach(int from_node_id, int to_node_id) const {
	if (from_node_id == to_node_id)
		return true;
	// Iterative DFS with visited set — guards against cycles in the connection graph
	std::unordered_set<int> visited;
	std::vector<int> stack;
	stack.push_back(from_node_id);
	while (!stack.empty()) {
		int cur = stack.back();
		stack.pop_back();
		if (!visited.insert(cur).second)
			continue; // already visited — skip
		for (const auto &c : connections) {
			if (!c.enabled)
				continue;
			if (c.in_node_id == cur) {
				if (c.out_node_id == to_node_id)
					return true;
				if (!visited.count(c.out_node_id))
					stack.push_back(c.out_node_id);
			}
		}
	}
	return false;
}

// ============================================================
// Neural I/O node rebuild
// ============================================================
void Genome::rebuild_neural_io_nodes(InnovationTracker &innov) {
	// Remove all INPUT and OUTPUT nodes, then re-add to match current segments.
	// HIDDEN and BIAS nodes are kept.
	nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
						[](const NodeGene &n) {
							return n.type == NodeType::INPUT || n.type == NodeType::OUTPUT;
						}),
			nodes.end());

	// Ensure BIAS exists
	bool has_bias = false;
	for (const auto &n : nodes)
		if (n.type == NodeType::BIAS) {
			has_bias = true;
			break;
		}
	if (!has_bias) {
		NodeGene bias;
		bias.id = innov.next_node_id();
		bias.type = NodeType::BIAS;
		nodes.push_back(bias);
	}

	// Add INPUT nodes: active_segments * 9 + 5 inputs
	int n_seg = active_segment_count();
	int n_inputs = n_seg * 9 + 5;
	for (int i = 0; i < n_inputs; ++i) {
		NodeGene ng;
		ng.id = innov.next_node_id();
		ng.type = NodeType::INPUT;
		ng.activation = ActivationType::LINEAR;
		nodes.push_back(ng);
	}

	// Add OUTPUT nodes: one per non-root active joint
	for (const auto &s : segments) {
		if (!s.enabled || s.parent_id < 0)
			continue;
		NodeGene ng;
		ng.id = innov.next_node_id();
		ng.type = NodeType::OUTPUT;
		ng.activation = ActivationType::TANH;
		nodes.push_back(ng);
	}

	// Disable any connections referencing removed node ids
	// (they are kept but disabled to preserve innovation history if ever re-added)
	// In practice for a fresh create_minimal, all connections are regenerated.
}

// ============================================================
// create_minimal
// ============================================================
Genome Genome::create_minimal(int new_id, const SimConfig &cfg,
		InnovationTracker &innov, uint32_t &rng) {
	Genome g;
	g.id = new_id;
	g.m_next_segment_id = 0;

	// Root segment
	BodySegmentGene root;
	root.segment_id = g.m_next_segment_id++;
	root.parent_id = -1;
	root.shape = ShapeType::CAPSULE;
	root.sx = 0.3f;
	root.sy = 0.3f;
	root.sz = 0.5f;
	root.r = rand_range(rng, 0.2f, 1.0f);
	root.g = rand_range(rng, 0.2f, 1.0f);
	root.b = rand_range(rng, 0.2f, 1.0f);
	root.enabled = true;
	g.segments.push_back(root);

	// One child segment
	BodySegmentGene child;
	child.segment_id = g.m_next_segment_id++;
	child.parent_id = root.segment_id;
	child.shape = ShapeType::CAPSULE;
	child.sx = 0.2f;
	child.sy = 0.2f;
	child.sz = 0.4f;
	child.attach_x = 0.0f;
	child.attach_y = 0.0f;
	child.attach_z = 0.6f;
	child.joint_type = JointType::HINGE;
	child.joint_min = -1.0f;
	child.joint_max = 1.0f;
	child.r = rand_range(rng, 0.2f, 1.0f);
	child.g = rand_range(rng, 0.2f, 1.0f);
	child.b = rand_range(rng, 0.2f, 1.0f);
	child.enabled = true;
	g.segments.push_back(child);

	// Build neural I/O from segment tree
	g.rebuild_neural_io_nodes(innov);

	// Create fully-connected input->output connections (minimal)
	// Collect input and output node ids
	std::vector<int> input_ids, output_ids;
	int bias_id = -1;
	for (const auto &n : g.nodes) {
		if (n.type == NodeType::INPUT)
			input_ids.push_back(n.id);
		if (n.type == NodeType::OUTPUT)
			output_ids.push_back(n.id);
		if (n.type == NodeType::BIAS)
			bias_id = n.id;
	}

	// Connect each output to bias (and skip full dense to keep initial nets sparse)
	// Only wire bias -> each output
	for (int out_id : output_ids) {
		if (bias_id >= 0) {
			ConnectionGene cg;
			cg.in_node_id = bias_id;
			cg.out_node_id = out_id;
			cg.weight = rand_range(rng, -1.0f, 1.0f);
			cg.enabled = true;
			cg.innovation_number = innov.get_or_create(bias_id, out_id);
			g.connections.push_back(cg);
		}
		// Wire a random subset of inputs to each output
		int n_wire = std::max(1, static_cast<int>(input_ids.size()) / 3);
		for (int k = 0; k < n_wire && k < static_cast<int>(input_ids.size()); ++k) {
			int in_id = input_ids[rand_int(rng, 0, static_cast<int>(input_ids.size()) - 1)];
			if (!g.connection_exists(in_id, out_id)) {
				ConnectionGene cg;
				cg.in_node_id = in_id;
				cg.out_node_id = out_id;
				cg.weight = rand_range(rng, -1.0f, 1.0f);
				cg.enabled = true;
				cg.innovation_number = innov.get_or_create(in_id, out_id);
				g.connections.push_back(cg);
			}
		}
	}

	return g;
}

// ============================================================
// mutate_weights
// ============================================================
void Genome::mutate_weights(const SimConfig &cfg, uint32_t &rng) {
	for (auto &c : connections) {
		if (rand_float(rng) < cfg.weight_mutation_rate) {
			if (rand_float(rng) < cfg.weight_perturb_rate) {
				c.weight += rand_range(rng, -1.0f, 1.0f) * cfg.weight_perturb_power;
			} else {
				c.weight = rand_range(rng, -2.0f, 2.0f);
			}
			// Clamp weight
			if (c.weight > 4.0f)
				c.weight = 4.0f;
			if (c.weight < -4.0f)
				c.weight = -4.0f;
		}
	}
}

// ============================================================
// mutate_add_node
// ============================================================
void Genome::mutate_add_node(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng) {
	// Collect enabled connections eligible for splitting
	std::vector<int> candidates;
	for (int i = 0; i < static_cast<int>(connections.size()); ++i)
		if (connections[i].enabled)
			candidates.push_back(i);
	if (candidates.empty())
		return;

	int idx = candidates[rand_int(rng, 0, static_cast<int>(candidates.size()) - 1)];

	// Copy the values we need before any push_back that could reallocate connections
	int old_in_id = connections[idx].in_node_id;
	int old_out_id = connections[idx].out_node_id;
	float old_weight = connections[idx].weight;
	connections[idx].enabled = false;

	// New hidden node
	int new_node_id = innov.next_node_id();
	NodeGene ng;
	ng.id = new_node_id;
	ng.type = NodeType::HIDDEN;
	ng.activation = ActivationType::TANH;
	nodes.push_back(ng);

	// in -> new_node with weight 1
	ConnectionGene c1;
	c1.in_node_id = old_in_id;
	c1.out_node_id = new_node_id;
	c1.weight = 1.0f;
	c1.enabled = true;
	c1.innovation_number = innov.get_or_create(old_in_id, new_node_id);
	connections.push_back(c1);

	// new_node -> out with old weight
	ConnectionGene c2;
	c2.in_node_id = new_node_id;
	c2.out_node_id = old_out_id;
	c2.weight = old_weight;
	c2.enabled = true;
	c2.innovation_number = innov.get_or_create(new_node_id, old_out_id);
	connections.push_back(c2);
}

// ============================================================
// mutate_add_connection
// ============================================================
void Genome::mutate_add_connection(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng) {
	// Collect all non-output and non-input node ids for source
	// (inputs, hidden, bias can be sources; hidden, output can be targets)
	std::vector<int> sources, targets;
	for (const auto &n : nodes) {
		if (n.type != NodeType::OUTPUT)
			sources.push_back(n.id);
		if (n.type != NodeType::INPUT && n.type != NodeType::BIAS)
			targets.push_back(n.id);
	}
	if (sources.empty() || targets.empty())
		return;

	// Try up to 20 random pairs to find a valid new connection
	for (int attempt = 0; attempt < 20; ++attempt) {
		int in_id = sources[rand_int(rng, 0, static_cast<int>(sources.size()) - 1)];
		int out_id = targets[rand_int(rng, 0, static_cast<int>(targets.size()) - 1)];

		if (in_id == out_id)
			continue;
		if (connection_exists(in_id, out_id))
			continue;
		// DAG guard: adding in->out must not create a cycle
		// i.e. out must not already reach in
		if (can_reach(out_id, in_id))
			continue;

		ConnectionGene cg;
		cg.in_node_id = in_id;
		cg.out_node_id = out_id;
		cg.weight = rand_range(rng, -2.0f, 2.0f);
		cg.enabled = true;
		cg.innovation_number = innov.get_or_create(in_id, out_id);
		connections.push_back(cg);
		return;
	}
}

// ============================================================
// mutate_add_segment
// ============================================================
void Genome::mutate_add_segment(const SimConfig &cfg, uint32_t &rng) {
	if (active_segment_count() >= cfg.max_segments)
		return;

	// Choose a random active segment as parent
	std::vector<int> parent_candidates;
	for (const auto &s : segments)
		if (s.enabled)
			parent_candidates.push_back(s.segment_id);
	if (parent_candidates.empty())
		return;

	int parent_id = parent_candidates[rand_int(rng, 0, static_cast<int>(parent_candidates.size()) - 1)];

	BodySegmentGene seg;
	seg.segment_id = m_next_segment_id++;
	seg.parent_id = parent_id;
	seg.shape = static_cast<ShapeType>(rand_int(rng, 0, 2));
	seg.sx = rand_range(rng, 0.15f, 0.5f);
	seg.sy = rand_range(rng, 0.15f, 0.5f);
	seg.sz = rand_range(rng, 0.2f, 0.6f);
	seg.attach_x = rand_range(rng, -0.3f, 0.3f);
	seg.attach_y = rand_range(rng, -0.3f, 0.3f);
	seg.attach_z = rand_range(rng, 0.4f, 0.8f);
	seg.joint_type = (rand_float(rng) < 0.7f) ? JointType::HINGE : JointType::BALL;
	float limit = rand_range(rng, 0.4f, 1.5f);
	seg.joint_min = -limit;
	seg.joint_max = limit;
	seg.r = rand_range(rng, 0.2f, 1.0f);
	seg.g = rand_range(rng, 0.2f, 1.0f);
	seg.b = rand_range(rng, 0.2f, 1.0f);
	seg.density = 1.0f;
	seg.enabled = true;
	segments.push_back(seg);

	// DO NOT call rebuild_neural_io_nodes here - caller (mutate_all) does it once
}

// ============================================================
// mutate_remove_segment
// ============================================================
void Genome::mutate_remove_segment(const SimConfig &cfg, uint32_t &rng) {
	if (active_segment_count() <= cfg.min_segments)
		return;

	// Only remove leaf segments (no enabled children)
	std::vector<int> leaf_candidates;
	for (const auto &s : segments) {
		if (!s.enabled || s.parent_id < 0)
			continue; // skip root
		if (is_leaf_segment(s.segment_id))
			leaf_candidates.push_back(s.segment_id);
	}
	if (leaf_candidates.empty())
		return;

	int remove_id = leaf_candidates[rand_int(rng, 0, static_cast<int>(leaf_candidates.size()) - 1)];
	for (auto &s : segments)
		if (s.segment_id == remove_id) {
			s.enabled = false;
			break;
		}

	// DO NOT call rebuild_neural_io_nodes here - caller (mutate_all) does it once
}

// ============================================================
// mutate_all
// ============================================================
void Genome::mutate_all(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng) {
	bool structural_body_change = false;

	if (rand_float(rng) < cfg.add_segment_rate) {
		mutate_add_segment(cfg, rng);
		structural_body_change = true;
	}
	if (rand_float(rng) < cfg.remove_segment_rate) {
		mutate_remove_segment(cfg, rng);
		structural_body_change = true;
	}

	if (structural_body_change)
		rebuild_neural_io_nodes(innov);

	mutate_weights(cfg, rng);

	if (rand_float(rng) < cfg.add_node_rate)
		mutate_add_node(cfg, innov, rng);

	if (rand_float(rng) < cfg.add_connection_rate)
		mutate_add_connection(cfg, innov, rng);
}

// ============================================================
// crossover
// ============================================================
Genome Genome::crossover(int new_id, const Genome &fit, const Genome &less, uint32_t &rng) {
	Genome child;
	child.id = new_id;

	// Segment tree from more-fit parent; try to copy colours from less-fit where ids match
	child.segments = fit.segments;
	child.m_next_segment_id = fit.m_next_segment_id;
	for (auto &cs : child.segments) {
		for (const auto &ls : less.segments) {
			if (ls.segment_id == cs.segment_id) {
				// Blend colours
				float t = static_cast<float>(xorshift32(const_cast<uint32_t &>(rng))) / 4294967295.0f;
				cs.r = t * cs.r + (1.0f - t) * ls.r;
				cs.g = t * cs.g + (1.0f - t) * ls.g;
				cs.b = t * cs.b + (1.0f - t) * ls.b;
				break;
			}
		}
	}

	// Connection genes: align by innovation number
	// More-fit genes always included; matching genes random 50/50
	const std::vector<ConnectionGene> &fit_conns = fit.connections;
	const std::vector<ConnectionGene> &les_conns = less.connections;

	for (const auto &fc : fit_conns) {
		const ConnectionGene *match = nullptr;
		for (const auto &lc : les_conns)
			if (lc.innovation_number == fc.innovation_number) {
				match = &lc;
				break;
			}

		ConnectionGene chosen = fc;
		if (match) {
			// Randomly pick from either parent
			if (rand_float(const_cast<uint32_t &>(rng)) < 0.5f)
				chosen = *match;
			// If either parent had it disabled, 75% chance offspring also disabled
			if (!fc.enabled || !match->enabled)
				chosen.enabled = (rand_float(const_cast<uint32_t &>(rng)) > 0.75f);
		}
		child.connections.push_back(chosen);
	}

	// Node genes: take all from more-fit parent; add hidden nodes only if
	// they are referenced by the chosen connections and not already present
	child.nodes = fit.nodes;
	// Incorporate hidden nodes from less-fit that appear in child connections
	for (const auto &cc : child.connections) {
		auto add_if_missing = [&](int nid) {
			if (!child.find_node(nid)) {
				const NodeGene *n = less.find_node(nid);
				if (n)
					child.nodes.push_back(*n);
			}
		};
		add_if_missing(cc.in_node_id);
		add_if_missing(cc.out_node_id);
	}

	return child;
}

// ============================================================
// compatibility_distance
// ============================================================
float Genome::compatibility_distance(const Genome &other, const SimConfig &cfg) const {
	if (connections.empty() && other.connections.empty())
		return 0.0f;

	int n = std::max(static_cast<int>(connections.size()),
			static_cast<int>(other.connections.size()));
	if (n == 0)
		return 0.0f;

	int max_innov_a = 0, max_innov_b = 0;
	for (const auto &c : connections)
		if (c.innovation_number > max_innov_a)
			max_innov_a = c.innovation_number;
	for (const auto &c : other.connections)
		if (c.innovation_number > max_innov_b)
			max_innov_b = c.innovation_number;
	int disjoint_threshold = std::min(max_innov_a, max_innov_b);

	int excess_count = 0, disjoint_count = 0;
	float weight_diff_sum = 0.0f;
	int matching = 0;

	for (const auto &ca : connections) {
		const ConnectionGene *match = nullptr;
		for (const auto &cb : other.connections)
			if (cb.innovation_number == ca.innovation_number) {
				match = &cb;
				break;
			}
		if (match) {
			weight_diff_sum += std::abs(ca.weight - match->weight);
			++matching;
		} else {
			if (ca.innovation_number > disjoint_threshold)
				++excess_count;
			else
				++disjoint_count;
		}
	}
	for (const auto &cb : other.connections) {
		bool found = false;
		for (const auto &ca : connections)
			if (ca.innovation_number == cb.innovation_number) {
				found = true;
				break;
			}
		if (!found) {
			if (cb.innovation_number > disjoint_threshold)
				++excess_count;
			else
				++disjoint_count;
		}
	}

	float avg_w = (matching > 0) ? (weight_diff_sum / static_cast<float>(matching)) : 0.0f;
	float N = static_cast<float>(n < 20 ? 1 : n);
	return cfg.c1_excess * static_cast<float>(excess_count) / N + cfg.c2_disjoint * static_cast<float>(disjoint_count) / N + cfg.c3_weight_diff * avg_w;
}
