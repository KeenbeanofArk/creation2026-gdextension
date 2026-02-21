#pragma once
#include "SimConfig.h"
#include <cstdint>
#include <tuple>
#include <vector>

// ============================================================
// Enumerations
// ============================================================
enum class NodeType : uint8_t { INPUT,
	HIDDEN,
	OUTPUT,
	BIAS };
enum class ActivationType : uint8_t { TANH,
	SIGMOID,
	LINEAR };
enum class JointType : uint8_t { HINGE,
	BALL };
enum class ShapeType : uint8_t { SPHERE,
	BOX,
	CAPSULE };

// ============================================================
// Gene structs
// ============================================================
struct NodeGene {
	int id = 0;
	NodeType type = NodeType::HIDDEN;
	ActivationType activation = ActivationType::TANH;
	bool enabled = true;
};

struct ConnectionGene {
	int in_node_id = 0;
	int out_node_id = 0;
	float weight = 0.0f;
	bool enabled = true;
	int innovation_number = 0;
};

struct BodySegmentGene {
	int segment_id = 0;
	int parent_id = -1; // -1 = root
	ShapeType shape = ShapeType::BOX;
	float sx = 0.3f, sy = 0.3f, sz = 0.5f; // half-extents / radius
	float attach_x = 0.0f, attach_y = 0.0f, attach_z = 0.6f; // offset from parent
	JointType joint_type = JointType::HINGE;
	float joint_min = -1.0f; // radians
	float joint_max = 1.0f;
	float r = 0.5f, g = 0.5f, b = 0.8f;
	float density = 1.0f;
	bool enabled = true;
};

// ============================================================
// Innovation tracker
// ============================================================
class InnovationTracker {
public:
	InnovationTracker() = default;

	// Returns existing innovation number if (in,out) already registered this
	// generation, otherwise allocates a new one.
	int get_or_create(int in_node, int out_node);

	// Must be called at the start of each generation's mutation pass.
	void new_generation();

	// Allocate a fresh node id (used when splitting a connection).
	int next_node_id();

	int current_innovation() const { return m_next_innovation - 1; }

private:
	int m_next_innovation = 1;
	int m_next_node_id = 1;
	// (in_node, out_node, innovation_number) cache for current generation
	std::vector<std::tuple<int, int, int>> m_this_gen;
};

// ============================================================
// Genome
// ============================================================
class Genome {
public:
	int id = 0;
	int species_id = -1;
	float fitness = 0.0f;
	float adj_fitness = 0.0f;

	std::vector<NodeGene> nodes;
	std::vector<ConnectionGene> connections;
	std::vector<BodySegmentGene> segments;

	// Factory: build a valid minimal genome (1 root segment, matching I/O nodes)
	static Genome create_minimal(int id, const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng);

	// ---- Mutation operators (mutate in-place) ----
	void mutate_weights(const SimConfig &cfg, uint32_t &rng);
	void mutate_add_node(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng);
	void mutate_add_connection(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng);
	void mutate_add_segment(const SimConfig &cfg, uint32_t &rng);
	void mutate_remove_segment(const SimConfig &cfg, uint32_t &rng);

	// Apply all mutation operators based on rates in cfg
	void mutate_all(const SimConfig &cfg, InnovationTracker &innov, uint32_t &rng);

	// ---- Crossover ----
	// more_fit parent provides structure for disjoint/excess genes
	static Genome crossover(int new_id, const Genome &more_fit,
			const Genome &less_fit, uint32_t &rng);

	// ---- Speciation ----
	float compatibility_distance(const Genome &other, const SimConfig &cfg) const;

	// ---- Derived counts ----
	// Input: per segment: local_vel(3) + contact_normal(3) + euler_angles(3) = 9 per seg
	//        global: energy(1) + food_dir(3) + food_dist(1) = 5
	// Total = active_segments * 9 + 5
	int neural_input_count() const;
	int neural_output_count() const; // one torque per non-root joint
	int active_segment_count() const;

	// ---- Helpers ----
	NodeGene *find_node(int id);
	const NodeGene *find_node(int id) const;
	bool connection_exists(int in_id, int out_id) const;
	bool is_leaf_segment(int seg_id) const;

	// Rebuild INPUT and OUTPUT nodes to match current segment tree.
	// Must be called after any structural body mutation.
	void rebuild_neural_io_nodes(InnovationTracker &innov);

private:
	int m_next_segment_id = 0;

	// Topological reachability check for connection validity (DAG guard)
	bool can_reach(int from_node_id, int to_node_id) const;

	// Random helpers (xorshift32)
	static float rand_float(uint32_t &rng);
	static float rand_range(uint32_t &rng, float lo, float hi);
	static int rand_int(uint32_t &rng, int lo, int hi_inclusive);
};
