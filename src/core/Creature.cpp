#include "Creature.h"
#include "Physics.h"
#include <algorithm>
#include <cmath>

// ============================================================
// compute_body_mass  –  from shape and density
// ============================================================
static float compute_body_mass(const BodySegmentGene &gene, float density) {
	const float pi = 3.14159265f;
	switch (gene.shape) {
		case ShapeType::SPHERE:
			return density * (4.0f / 3.0f) * pi * gene.sx * gene.sx * gene.sx;
		case ShapeType::BOX:
			return density * (2.0f * gene.sx) * (2.0f * gene.sy) * (2.0f * gene.sz);
		case ShapeType::CAPSULE: {
			float r = gene.sx;
			float h = gene.sz * 2.0f;
			float sphere_vol = (4.0f / 3.0f) * pi * r * r * r;
			float cyl_vol = pi * r * r * h;
			return density * (sphere_vol + cyl_vol);
		}
	}
	return 1.0f;
}

static Mat3 compute_inertia(const BodySegmentGene &gene, float mass) {
	switch (gene.shape) {
		case ShapeType::SPHERE:
			return Mat3::inertia_sphere(mass, gene.sx);
		case ShapeType::BOX:
			return Mat3::inertia_box(mass, gene.sx, gene.sy, gene.sz);
		case ShapeType::CAPSULE:
			return Mat3::inertia_capsule(mass, gene.sx, gene.sz);
	}
	return Mat3::identity();
}

static float compute_sphere_radius(const BodySegmentGene &gene) {
	// Conservative bounding sphere radius for collision
	switch (gene.shape) {
		case ShapeType::SPHERE:
			return gene.sx;
		case ShapeType::BOX:
			return std::sqrt(gene.sx * gene.sx + gene.sy * gene.sy + gene.sz * gene.sz);
		case ShapeType::CAPSULE:
			return gene.sx + gene.sz; // radius + half-height
	}
	return gene.sx;
}

// ============================================================
// build_segment_recursive
// ============================================================
void Creature::build_segment_recursive(int gene_idx,
		int parent_seg_inst,
		int parent_body_idx,
		Vec3 parent_world_pos,
		Quat parent_world_rot,
		PhysicsWorld &world,
		const SimConfig &cfg) {
	const BodySegmentGene &gene = genome.segments[gene_idx];
	if (!gene.enabled)
		return;

	// World position of this segment
	Vec3 local_offset{ gene.attach_x, gene.attach_y, gene.attach_z };
	Vec3 world_offset = parent_world_rot.rotate(local_offset);
	Vec3 world_pos = parent_world_pos + world_offset;

	// Clamp initial position above ground (PhysicsWorld handles this, but start safe)
	float safe_spawn_y = world_pos.y;

	RigidBody body;
	body.active = true;
	float mass = compute_body_mass(gene, gene.density * cfg.body_density);
	body.inv_mass = (mass > 1e-6f) ? 1.0f / mass : 0.0f;
	body.position = { world_pos.x, safe_spawn_y, world_pos.z };
	body.rotation = parent_world_rot;
	body.sphere_radius = compute_sphere_radius(gene);
	body.linear_damping = cfg.linear_damping;
	body.angular_damping = cfg.angular_damping;

	Mat3 inertia = compute_inertia(gene, mass);
	body.inv_inertia_local = inertia.inverse();

	int body_idx = world.add_body(body);

	// Create joint (if not root)
	int joint_idx = -1;
	if (parent_body_idx >= 0) {
		Joint j;
		j.body_a_idx = parent_body_idx;
		j.body_b_idx = body_idx;
		j.type = gene.joint_type;
		j.angle_min = gene.joint_min;
		j.angle_max = gene.joint_max;
		j.swing_limit = std::max(0.1f, std::abs(gene.joint_max));

		// Anchor in parent body's local space = the attach offset
		j.anchor_a_local = local_offset;
		j.anchor_b_local = { 0.0f, 0.0f, 0.0f };
		j.axis_a_local = { 1.0f, 0.0f, 0.0f }; // hinge axis = X by default
		joint_idx = world.add_joint(j);
	}

	// Store instance
	SegmentInstance inst;
	inst.physics_body_idx = body_idx;
	inst.physics_joint_idx = joint_idx;
	inst.parent_seg_inst = parent_seg_inst;
	inst.gene = gene;
	int my_seg_inst_idx = static_cast<int>(segments.size());
	segments.push_back(inst);

	// Recurse: find children (segments whose parent_id == this gene's segment_id)
	for (int i = 0; i < static_cast<int>(genome.segments.size()); ++i) {
		if (!genome.segments[i].enabled)
			continue;
		if (genome.segments[i].parent_id == gene.segment_id)
			build_segment_recursive(i, my_seg_inst_idx, body_idx,
					world_pos, parent_world_rot, world, cfg);
	}
}

// ============================================================
// build_body
// ============================================================
void Creature::build_body(PhysicsWorld &world, Vec3 spawn_pos, const SimConfig &cfg) {
	segments.clear();

	// Find root segment (parent_id == -1)
	int root_idx = -1;
	for (int i = 0; i < static_cast<int>(genome.segments.size()); ++i) {
		if (genome.segments[i].enabled && genome.segments[i].parent_id < 0) {
			root_idx = i;
			break;
		}
	}
	if (root_idx < 0)
		return; // No valid root

	build_segment_recursive(root_idx, -1, -1, spawn_pos, Quat::identity(), world, cfg);

	// Build brain from genome
	brain.build(genome);

	energy = cfg.initial_energy;
	alive = true;
	fitness = 0.0f;
	food_eaten = 0;
}

// ============================================================
// deactivate_body
// ============================================================
void Creature::deactivate_body(PhysicsWorld &world) {
	for (const auto &seg : segments)
		if (seg.physics_body_idx >= 0)
			world.deactivate_body(seg.physics_body_idx);
}

// ============================================================
// get_root_position / center_of_mass
// ============================================================
Vec3 Creature::get_root_position(const PhysicsWorld &world) const {
	if (segments.empty())
		return Vec3::zero();
	return world.body(segments[0].physics_body_idx).position;
}

Vec3 Creature::get_center_of_mass(const PhysicsWorld &world) const {
	if (segments.empty())
		return Vec3::zero();
	Vec3 sum = Vec3::zero();
	float total_mass = 0.0f;
	for (const auto &s : segments) {
		const RigidBody &b = world.body(s.physics_body_idx);
		float m = (b.inv_mass > 0.0f) ? 1.0f / b.inv_mass : 0.0f;
		sum += b.position * m;
		total_mass += m;
	}
	if (total_mass < 1e-6f)
		return segments.empty() ? Vec3::zero() : world.body(segments[0].physics_body_idx).position;
	return sum * (1.0f / total_mass);
}

// ============================================================
// build_inputs
// Sensor layout: per active segment (in order): local_vel(3) + contact_normal(3) + euler(3) = 9
//                global: energy_norm(1) + food_dir(3) + food_dist(1) = 5
// ============================================================
std::vector<float> Creature::build_inputs(const PhysicsWorld &world,
		const std::vector<FoodItem> &food) const {
	std::vector<float> inputs;
	inputs.reserve(genome.neural_input_count());

	for (const auto &seg : segments) {
		if (!seg.gene.enabled)
			continue;
		const RigidBody &b = world.body(seg.physics_body_idx);

		// Local linear velocity (body space)
		Vec3 vel_world = b.velocity;
		Vec3 vel_local = b.rotation.conjugate().rotate(vel_world);
		inputs.push_back(vel_local.x * 0.1f); // scale down
		inputs.push_back(vel_local.y * 0.1f);
		inputs.push_back(vel_local.z * 0.1f);

		// Contact normal (ground normal or zero)
		if (b.on_ground) {
			inputs.push_back(b.ground_normal.x);
			inputs.push_back(b.ground_normal.y);
			inputs.push_back(b.ground_normal.z);
		} else {
			inputs.push_back(0.0f);
			inputs.push_back(0.0f);
			inputs.push_back(0.0f);
		}

		// Euler angles (YXZ order, normalised to [-1,1] by dividing by pi)
		Vec3 euler = b.rotation.to_euler_yxz();
		const float pi = 3.14159265f;
		inputs.push_back(euler.x / pi);
		inputs.push_back(euler.y / pi);
		inputs.push_back(euler.z / pi);
	}

	// Global: energy normalised
	float energy_max = 200.0f; // generous cap
	inputs.push_back(energy / energy_max);

	// Direction and distance to nearest active food
	Vec3 root_pos = get_root_position(world);
	float nearest_dist = 1e9f;
	Vec3 nearest_dir = Vec3::zero();
	for (const auto &fi : food) {
		if (!fi.active)
			continue;
		Vec3 delta = fi.position - root_pos;
		float dist = delta.length();
		if (dist < nearest_dist) {
			nearest_dist = dist;
			nearest_dir = (dist > 1e-4f) ? delta / dist : Vec3::zero();
		}
	}
	const float max_food_dist = 100.0f;
	inputs.push_back(nearest_dir.x);
	inputs.push_back(nearest_dir.y);
	inputs.push_back(nearest_dir.z);
	inputs.push_back(std::min(1.0f, nearest_dist / max_food_dist));

	// Pad to expected size if needed (safety)
	int expected = genome.neural_input_count();
	while (static_cast<int>(inputs.size()) < expected)
		inputs.push_back(0.0f);
	if (static_cast<int>(inputs.size()) > expected)
		inputs.resize(expected);

	return inputs;
}

// ============================================================
// apply_outputs
// ============================================================
void Creature::apply_outputs(PhysicsWorld &world, const std::vector<float> &outputs) {
	// One output per non-root joint, in order of non-root segments
	int out_idx = 0;
	for (const auto &seg : segments) {
		if (seg.physics_joint_idx < 0)
			continue; // skip root
		if (out_idx >= static_cast<int>(outputs.size()))
			break;

		world.joint(seg.physics_joint_idx).motor_target = outputs[out_idx];
		++out_idx;
	}
}

// ============================================================
// step
// ============================================================
void Creature::step(PhysicsWorld &world,
		const std::vector<FoodItem> &food,
		std::vector<bool> &food_collected,
		const SimConfig &cfg) {
	if (!alive)
		return;

	// 1. Build neural inputs
	auto inputs = build_inputs(world, food);

	// 2. Run brain
	std::vector<float> outputs;
	if (brain.is_valid() && brain.input_count() > 0)
		outputs = brain.evaluate(inputs);
	else
		outputs.assign(genome.neural_output_count(), 0.0f);

	// 3. Apply motor torques
	apply_outputs(world, outputs);

	// 4. Energy drain
	energy -= cfg.energy_drain_per_step;

	// 5. Food pickup
	Vec3 root_pos = get_root_position(world);
	float pickup_r2 = cfg.food_pickup_radius * cfg.food_pickup_radius;
	for (int fi = 0; fi < static_cast<int>(food.size()); ++fi) {
		if (!food[fi].active || food_collected[fi])
			continue;
		Vec3 delta = food[fi].position - root_pos;
		if (delta.length_sq() <= pickup_r2) {
			energy += cfg.energy_from_food;
			++food_eaten;
			fitness += food[fi].energy_value;
			food_collected[fi] = true;
		}
	}

	// 6. Fitness bonus for survival time
	fitness += 0.001f;

	// 7. Death check
	if (energy <= 0.0f) {
		alive = false;
		energy = 0.0f;
		deactivate_body(world);
	}
}
