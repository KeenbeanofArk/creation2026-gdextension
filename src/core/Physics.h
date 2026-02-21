#pragma once
#include "Genome.h" // for JointType enum
#include "Math3D.h"
#include "SimConfig.h"
#include <cassert>
#include <vector>

// ============================================================
// RigidBody
// ============================================================
struct RigidBody {
	int id = -1;
	bool active = true; // false = static / skip dynamics

	Vec3 position = { 0.0f, 0.0f, 0.0f };
	Vec3 velocity = { 0.0f, 0.0f, 0.0f };
	Quat rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
	Vec3 angular_vel = { 0.0f, 0.0f, 0.0f };

	float inv_mass = 1.0f; // 0.0f = infinite mass (static)
	Mat3 inv_inertia_local; // In body space, updated once at creation

	Vec3 force_accum = { 0.0f, 0.0f, 0.0f };
	Vec3 torque_accum = { 0.0f, 0.0f, 0.0f };

	float sphere_radius = 0.5f; // collision proxy radius
	Vec3 sphere_offset_local = { 0.0f, 0.0f, 0.0f }; // offset in body space

	float linear_damping = 0.30f;
	float angular_damping = 0.50f;

	bool on_ground = false;
	Vec3 ground_normal = { 0.0f, 1.0f, 0.0f };

	// Helpers
	void apply_force(Vec3 f) { force_accum += f; }
	void apply_torque(Vec3 t) { torque_accum += t; }
	void apply_force_at_world_point(Vec3 f, Vec3 world_pt);
	void clear_forces() {
		force_accum = Vec3::zero();
		torque_accum = Vec3::zero();
	}

	Vec3 world_velocity_at_point(Vec3 world_pt) const;
	Mat3 world_inertia_inv() const;
	Vec3 sphere_world_center() const;
};

// ============================================================
// Joint
// ============================================================
struct Joint {
	int body_a_idx = -1;
	int body_b_idx = -1;
	JointType type = JointType::HINGE;

	Vec3 anchor_a_local = { 0.0f, 0.0f, 0.0f };
	Vec3 anchor_b_local = { 0.0f, 0.0f, 0.0f };
	Vec3 axis_a_local = { 1.0f, 0.0f, 0.0f }; // hinge axis in body A space

	float angle_min = -1.5708f;
	float angle_max = 1.5708f;

	float current_angle = 0.0f;
	float prev_angle = 0.0f;

	// Brain sets this [-1, 1]; scaled to angle range for PD target
	float motor_target = 0.0f;

	// Ball joint swing limit (cone half-angle radians)
	float swing_limit = 1.0f;

	bool active = true;
};

// ============================================================
// PhysicsWorld
// ============================================================
class PhysicsWorld {
public:
	explicit PhysicsWorld(const SimConfig &cfg);

	// Returns stable index (never changes while world is alive)
	int add_body(const RigidBody &body);
	int add_joint(const Joint &joint);

	// Deactivate: zero mass, freeze motion — safe to call mid-step
	void deactivate_body(int idx);

	// Clear all bodies and joints (called at generation boundary)
	void clear_all();

	// Provide the terrain heightmap
	void set_heightmap(const std::vector<float> &h, int size,
			float cell_scale, float height_scale);

	float sample_height(float x, float z) const;
	Vec3 sample_normal(float x, float z) const;

	// Advance simulation by one fixed timestep (SimConfig::physics_dt)
	void step();

	// Accessors
	RigidBody &body(int idx) { assert(idx >= 0 && idx < (int)m_bodies.size()); return m_bodies[idx]; }
	const RigidBody &body(int idx) const { assert(idx >= 0 && idx < (int)m_bodies.size()); return m_bodies[idx]; }
	RigidBody *body_safe(int idx) { if (idx < 0 || idx >= (int)m_bodies.size()) return nullptr; return &m_bodies[idx]; }
	const RigidBody *body_safe(int idx) const { if (idx < 0 || idx >= (int)m_bodies.size()) return nullptr; return &m_bodies[idx]; }
	Joint &joint(int idx) { return m_joints[idx]; }
	const Joint &joint(int idx) const { return m_joints[idx]; }
	int body_count() const { return static_cast<int>(m_bodies.size()); }
	int joint_count() const { return static_cast<int>(m_joints.size()); }

private:
	void apply_gravity();
	void integrate_forces(); // semi-implicit Euler velocity step
	void apply_damping();
	void integrate_positions();
	void resolve_ground_contacts(); // sphere vs heightmap
	void resolve_body_contacts(); // sphere-sphere, broadphase + narrow
	void solve_joints(); // PD motor + anchor position correction
	void update_joint_angles();
	void clamp_velocities();
	void sanitize_bodies(); // NaN guard

	float hinge_angle(const Joint &j) const;

	std::vector<RigidBody> m_bodies;
	std::vector<Joint> m_joints;
	std::vector<float> m_heights;
	int m_terrain_size = 0;
	float m_cell_scale = 1.0f;
	float m_height_scale = 1.0f;
	int m_next_body_id = 0;

	const SimConfig &m_cfg;
};
