#pragma once
#include "Brain.h"
#include "Genome.h"
#include "Math3D.h"
#include "SimTypes.h"
#include <vector>

class PhysicsWorld;

// ============================================================
// SegmentInstance  –  maps Genome segment to physics objects
// ============================================================
struct SegmentInstance {
	int physics_body_idx = -1;
	int physics_joint_idx = -1; // -1 for root
	int parent_seg_inst = -1; // index in creature's segments list
	BodySegmentGene gene; // cached gene data
};

// ============================================================
// Creature
// ============================================================
class Creature {
public:
	int id = 0;
	bool alive = true;
	float energy = 0.0f;
	float fitness = 0.0f;
	int food_eaten = 0;

	Genome genome;
	Brain brain;

	std::vector<SegmentInstance> segments;

	// Build physics representation from genome.
	// spawn_pos is where the root segment is placed.
	void build_body(PhysicsWorld &world, Vec3 spawn_pos, const SimConfig &cfg);

	// Deactivate all physics bodies (on death or generation reset)
	void deactivate_body(PhysicsWorld &world);

	// One simulation step: sense -> think -> act -> metabolise -> eat
	void step(PhysicsWorld &world,
			const std::vector<FoodItem> &food,
			std::vector<bool> &food_collected,
			const SimConfig &cfg);

	// Root segment position (for stats, respawn checks)
	Vec3 get_root_position(const PhysicsWorld &world) const;
	Vec3 get_center_of_mass(const PhysicsWorld &world) const;

private:
	// --- Sensor building (length = genome.neural_input_count()) ---
	std::vector<float> build_inputs(const PhysicsWorld &world,
			const std::vector<FoodItem> &food) const;

	// --- Motor application ---
	void apply_outputs(PhysicsWorld &world, const std::vector<float> &outputs);

	// Recursive body builder
	void build_segment_recursive(int gene_idx,
			int parent_seg_inst,
			int parent_body_idx,
			Vec3 parent_world_pos,
			Quat parent_world_rot,
			PhysicsWorld &world,
			const SimConfig &cfg);
};
