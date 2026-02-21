#include "Physics.h"
#include <algorithm>
#include <cassert>
#include <cmath>

// ============================================================
// RigidBody helpers
// ============================================================
void RigidBody::apply_force_at_world_point(Vec3 f, Vec3 world_pt) {
	force_accum += f;
	Vec3 r = world_pt - position;
	torque_accum += Vec3::cross(r, f);
}

Vec3 RigidBody::world_velocity_at_point(Vec3 world_pt) const {
	Vec3 r = world_pt - position;
	return velocity + Vec3::cross(angular_vel, r);
}

Mat3 RigidBody::world_inertia_inv() const {
	return inv_inertia_local.rotated_by(rotation);
}

Vec3 RigidBody::sphere_world_center() const {
	return position + rotation.rotate(sphere_offset_local);
}

// ============================================================
// PhysicsWorld
// ============================================================
PhysicsWorld::PhysicsWorld(const SimConfig &cfg) : m_cfg(cfg) {}

int PhysicsWorld::add_body(const RigidBody &body) {
	int idx = static_cast<int>(m_bodies.size());
	RigidBody b = body;
	b.id = m_next_body_id++;
	m_bodies.push_back(b);
	return idx;
}

int PhysicsWorld::add_joint(const Joint &joint) {
	int idx = static_cast<int>(m_joints.size());
	m_joints.push_back(joint);
	return idx;
}

void PhysicsWorld::deactivate_body(int idx) {
	if (idx < 0 || idx >= static_cast<int>(m_bodies.size()))
		return;
	m_bodies[idx].active = false;
	m_bodies[idx].inv_mass = 0.0f;
	m_bodies[idx].velocity = Vec3::zero();
	m_bodies[idx].angular_vel = Vec3::zero();
	m_bodies[idx].force_accum = Vec3::zero();
	m_bodies[idx].torque_accum = Vec3::zero();
}

void PhysicsWorld::clear_all() {
	m_bodies.clear();
	m_joints.clear();
	m_next_body_id = 0;
}

void PhysicsWorld::set_heightmap(const std::vector<float> &h, int size,
		float cell_scale, float height_scale) {
	m_heights = h;
	m_terrain_size = size;
	m_cell_scale = cell_scale;
	m_height_scale = height_scale;
}

float PhysicsWorld::sample_height(float x, float z) const {
	if (m_heights.empty())
		return 0.0f;
	// Map world coords to heightmap grid (centred at origin)
	float half = m_terrain_size * m_cell_scale * 0.5f;
	float gx = (x + half) / m_cell_scale;
	float gz = (z + half) / m_cell_scale;
	int ix = static_cast<int>(gx);
	int iz = static_cast<int>(gz);
	// Clamp
	ix = std::max(0, std::min(m_terrain_size - 2, ix));
	iz = std::max(0, std::min(m_terrain_size - 2, iz));
	float fx = gx - static_cast<float>(ix);
	float fz = gz - static_cast<float>(iz);
	fx = std::max(0.0f, std::min(1.0f, fx));
	fz = std::max(0.0f, std::min(1.0f, fz));

	// Bilinear interpolation
	float h00 = m_heights[iz * m_terrain_size + ix] * m_height_scale;
	float h10 = m_heights[iz * m_terrain_size + ix + 1] * m_height_scale;
	float h01 = m_heights[(iz + 1) * m_terrain_size + ix] * m_height_scale;
	float h11 = m_heights[(iz + 1) * m_terrain_size + ix + 1] * m_height_scale;
	return h00 * (1 - fx) * (1 - fz) + h10 * fx * (1 - fz) + h01 * (1 - fx) * fz + h11 * fx * fz;
}

Vec3 PhysicsWorld::sample_normal(float x, float z) const {
	float eps = m_cell_scale;
	float hL = sample_height(x - eps, z);
	float hR = sample_height(x + eps, z);
	float hD = sample_height(x, z - eps);
	float hU = sample_height(x, z + eps);
	Vec3 n{ hL - hR, 2.0f * eps, hD - hU };
	return n.normalized();
}

// ============================================================
// step
// ============================================================
void PhysicsWorld::step() {
	apply_gravity();
	integrate_forces();
	apply_damping();
	integrate_positions();
	resolve_ground_contacts();
	resolve_body_contacts();
	solve_joints();
	update_joint_angles();
	clamp_velocities();
	sanitize_bodies();
}

// ============================================================
void PhysicsWorld::apply_gravity() {
	Vec3 g{ 0.0f, m_cfg.gravity, 0.0f };
	for (auto &b : m_bodies) {
		if (!b.active || b.inv_mass == 0.0f)
			continue;
		b.apply_force(g * (1.0f / b.inv_mass));
	}
}

void PhysicsWorld::integrate_forces() {
	float dt = m_cfg.physics_dt;
	for (auto &b : m_bodies) {
		if (!b.active || b.inv_mass == 0.0f)
			continue;
		// Semi-implicit Euler: update velocity first
		b.velocity += b.force_accum * (b.inv_mass * dt);
		Vec3 alpha = b.world_inertia_inv() * b.torque_accum;
		b.angular_vel += alpha * dt;
		b.clear_forces();
	}
}

void PhysicsWorld::apply_damping() {
	for (auto &b : m_bodies) {
		if (!b.active)
			continue;
		b.velocity *= std::max(0.0f, 1.0f - b.linear_damping * m_cfg.physics_dt);
		b.angular_vel *= std::max(0.0f, 1.0f - b.angular_damping * m_cfg.physics_dt);
	}
}

void PhysicsWorld::integrate_positions() {
	float dt = m_cfg.physics_dt;
	for (auto &b : m_bodies) {
		if (!b.active || b.inv_mass == 0.0f)
			continue;
		b.position += b.velocity * dt;
		b.rotation = Quat::integrate(b.rotation, b.angular_vel, dt);
		b.on_ground = false;
	}
}

// ============================================================
// resolve_ground_contacts
// ============================================================
void PhysicsWorld::resolve_ground_contacts() {
	float restitution = m_cfg.ground_restitution;
	float friction = m_cfg.ground_friction;

	for (auto &b : m_bodies) {
		if (!b.active)
			continue;
		Vec3 center = b.sphere_world_center();
		float h = sample_height(center.x, center.z);
		float penetration = h + b.sphere_radius - center.y;
		if (penetration <= 0.0f)
			continue;

		Vec3 normal = sample_normal(center.x, center.z);
		b.on_ground = true;
		b.ground_normal = normal;

		// Push out of ground (position correction)
		b.position.y += penetration;

		// Velocity response
		Vec3 contact_vel = b.world_velocity_at_point(center);
		float vn = Vec3::dot(contact_vel, normal);
		if (vn >= 0.0f)
			continue; // Already separating

		// Normal impulse
		float j = 0.0f;
		if (b.inv_mass > 0.0f) {
			Vec3 r = center - b.position;
			Mat3 Iinv = b.world_inertia_inv();
			float denom = b.inv_mass + Vec3::dot(normal, Iinv * Vec3::cross(Vec3::cross(r, normal), r));
			if (denom > 1e-8f)
				j = -(1.0f + restitution) * vn / denom;
			else
				j = -(1.0f + restitution) * vn / b.inv_mass; // fallback: point mass, no rotation
		}

		Vec3 impulse = normal * j;
		b.velocity += impulse * b.inv_mass;
		if (b.inv_mass > 0.0f) {
			Vec3 r = center - b.position;
			b.angular_vel += b.world_inertia_inv() * Vec3::cross(r, impulse);
		}

		// Friction impulse
		Vec3 tangent_vel = contact_vel - normal * Vec3::dot(contact_vel, normal);
		if (!tangent_vel.is_near_zero()) {
			Vec3 t = tangent_vel.normalized();
			float jt_max = friction * std::abs(j);
			float vt = Vec3::dot(contact_vel, t);
			float jt = -vt;
			if (b.inv_mass > 0.0f) {
				Vec3 r = center - b.position;
				float denom_t = b.inv_mass + Vec3::dot(t, b.world_inertia_inv() * Vec3::cross(Vec3::cross(r, t), r));
				if (denom_t > 1e-8f)
					jt = -vt / denom_t;
			}
			jt = std::max(-jt_max, std::min(jt_max, jt));
			Vec3 friction_impulse = t * jt;
			b.velocity += friction_impulse * b.inv_mass;
			if (b.inv_mass > 0.0f) {
				Vec3 r = center - b.position;
				b.angular_vel += b.world_inertia_inv() * Vec3::cross(r, friction_impulse);
			}
		}
	}
}

// ============================================================
// resolve_body_contacts (sphere-sphere)
// ============================================================
void PhysicsWorld::resolve_body_contacts() {
	int N = static_cast<int>(m_bodies.size());
	for (int i = 0; i < N - 1; ++i) {
		RigidBody &a = m_bodies[i];
		if (!a.active)
			continue;

		for (int j = i + 1; j < N; ++j) {
			RigidBody &b = m_bodies[j];
			if (!b.active)
				continue;

			Vec3 ca = a.sphere_world_center();
			Vec3 cb = b.sphere_world_center();
			Vec3 delta = cb - ca;
			float dist2 = delta.length_sq();
			float min_dist = a.sphere_radius + b.sphere_radius;
			if (dist2 >= min_dist * min_dist)
				continue;

			float dist = std::sqrt(dist2);
			Vec3 normal = (dist > 1e-6f) ? delta / dist : Vec3{ 0.0f, 1.0f, 0.0f };
			float penetration = min_dist - dist;

			// Position correction
			float inv_total = a.inv_mass + b.inv_mass;
			if (inv_total > 0.0f) {
				float corr = penetration / inv_total * 1.0f; // baumgarte scalar
				a.position -= normal * (a.inv_mass * corr);
				b.position += normal * (b.inv_mass * corr);
			}

			// Velocity response
			Vec3 rel_vel = b.world_velocity_at_point(cb) - a.world_velocity_at_point(ca);
			float vn = Vec3::dot(rel_vel, normal);
			if (vn >= 0.0f)
				continue;

			float denom = a.inv_mass + b.inv_mass;
			if (denom < 1e-8f)
				continue;

			// Simplified point-mass impulse (no rotational transfer for simplicity)
			float impulse_mag = -(1.0f + 0.2f) * vn / denom;
			Vec3 impulse = normal * impulse_mag;
			a.velocity -= impulse * a.inv_mass;
			b.velocity += impulse * b.inv_mass;
		}
	}
}

// ============================================================
// hinge_angle  —  angle around axis_a_local
// ============================================================
float PhysicsWorld::hinge_angle(const Joint &j) const {
	const RigidBody &a = m_bodies[j.body_a_idx];
	const RigidBody &b = m_bodies[j.body_b_idx];

	// Axis in world space
	Vec3 axis = a.rotation.rotate(j.axis_a_local).normalized();

	// Reference vector perpendicular to axis (choose any perp in body-a)
	Vec3 ref_in_a = { 1.0f, 0.0f, 0.0f };
	if (std::abs(Vec3::dot(ref_in_a, j.axis_a_local)) > 0.9f)
		ref_in_a = { 0.0f, 1.0f, 0.0f };
	Vec3 ref_perp = Vec3::cross(j.axis_a_local, ref_in_a).normalized();
	Vec3 ref_world = a.rotation.rotate(ref_perp);

	// Corresponding vector in body b
	Vec3 b_vec = b.rotation.rotate(ref_perp);

	// Project onto plane perpendicular to axis
	auto project = [&](Vec3 v) -> Vec3 {
		return v - axis * Vec3::dot(v, axis);
	};
	Vec3 ra = project(ref_world);
	Vec3 rb = project(b_vec);
	if (ra.is_near_zero() || rb.is_near_zero())
		return 0.0f;
	ra = ra.normalized();
	rb = rb.normalized();

	float c = Vec3::dot(ra, rb);
	c = std::max(-1.0f, std::min(1.0f, c));
	float angle = std::acos(c);
	// Determine sign from cross product
	Vec3 cross = Vec3::cross(ra, rb);
	if (Vec3::dot(cross, axis) < 0.0f)
		angle = -angle;
	return angle;
}

// ============================================================
// update_joint_angles
// ============================================================
void PhysicsWorld::update_joint_angles() {
	for (auto &j : m_joints) {
		if (!j.active)
			continue;
		if (j.body_a_idx < 0 || j.body_a_idx >= static_cast<int>(m_bodies.size()))
			continue;
		if (j.body_b_idx < 0 || j.body_b_idx >= static_cast<int>(m_bodies.size()))
			continue;
		j.prev_angle = j.current_angle;
		j.current_angle = hinge_angle(j);
	}
}

// ============================================================
// solve_joints  (PD motor + anchor position correction)
// ============================================================
void PhysicsWorld::solve_joints() {
	float dt = m_cfg.physics_dt;
	float kp = m_cfg.joint_kp;
	float kd = m_cfg.joint_kd;

	for (auto &j : m_joints) {
		if (!j.active)
			continue;
		if (j.body_a_idx < 0 || j.body_a_idx >= static_cast<int>(m_bodies.size()))
			continue;
		if (j.body_b_idx < 0 || j.body_b_idx >= static_cast<int>(m_bodies.size()))
			continue;

		RigidBody &a = m_bodies[j.body_a_idx];
		RigidBody &b = m_bodies[j.body_b_idx];

		if (!a.active && !b.active)
			continue;

		// ---- Anchor correction (keep bodies joined) ----
		Vec3 anchor_a_world = a.position + a.rotation.rotate(j.anchor_a_local);
		Vec3 anchor_b_world = b.position + b.rotation.rotate(j.anchor_b_local);
		Vec3 err = anchor_b_world - anchor_a_world;

		const float baumgarte_beta = 0.1f;
		Vec3 corr_impulse = err * (baumgarte_beta / dt);
		// Clamp correction to avoid huge impulses from initially misaligned joints
		float corr_mag = corr_impulse.length();
		if (corr_mag > 10.0f)
			corr_impulse = corr_impulse * (10.0f / corr_mag);

		float inv_total = a.inv_mass + b.inv_mass;
		if (inv_total > 0.0f) {
			if (a.inv_mass > 0.0f) {
				a.velocity += corr_impulse * a.inv_mass;
				a.angular_vel += a.world_inertia_inv() * Vec3::cross(anchor_a_world - a.position, corr_impulse);
			}
			if (b.inv_mass > 0.0f) {
				b.velocity -= corr_impulse * b.inv_mass;
				b.angular_vel -= b.world_inertia_inv() * Vec3::cross(anchor_b_world - b.position, corr_impulse);
			}
		}

		// ---- Motor (PD controller for hinge joints) ----
		if (j.type == JointType::HINGE) {
			Vec3 axis = a.rotation.rotate(j.axis_a_local).normalized();

			// Target angle from motor_target in [-1,1]
			float target_angle;
			if (j.motor_target >= 0.0f)
				target_angle = j.motor_target * j.angle_max;
			else
				target_angle = j.motor_target * (-j.angle_min);

			// Clamp target to limits
			target_angle = std::max(j.angle_min, std::min(j.angle_max, target_angle));

			float angle_error = target_angle - j.current_angle;
			float angle_vel = (j.current_angle - j.prev_angle) / dt;

			float torque_mag = kp * angle_error - kd * angle_vel;
			torque_mag = std::max(-m_cfg.joint_max_torque,
					std::min(m_cfg.joint_max_torque, torque_mag));

			Vec3 torque = axis * torque_mag;
			a.apply_torque(-torque);
			b.apply_torque(torque);
		} else {
			// BALL joint: apply swing limit via penalty torque
			Vec3 axis_world = a.rotation.rotate(j.axis_a_local).normalized();
			Vec3 b_axis = b.rotation.rotate(j.axis_a_local).normalized();
			float swing_cos = Vec3::dot(axis_world, b_axis);
			if (swing_cos < std::cos(j.swing_limit)) {
				Vec3 corr_axis = Vec3::cross(axis_world, b_axis).normalized();
				float err_angle = std::acos(std::max(-1.0f, std::min(1.0f, swing_cos))) - j.swing_limit;
				float pen_torque = kp * err_angle;
				a.apply_torque(corr_axis * pen_torque);
				b.apply_torque(-corr_axis * pen_torque);
			}

			// Also apply the motor signal as a general torque
			Vec3 motor_torque = axis_world * (j.motor_target * m_cfg.joint_max_torque);
			a.apply_torque(-motor_torque);
			b.apply_torque(motor_torque);
		}
	}
}

// ============================================================
// clamp_velocities
// ============================================================
void PhysicsWorld::clamp_velocities() {
	float max_v = m_cfg.max_body_velocity;
	for (auto &b : m_bodies) {
		if (!b.active)
			continue;
		float v2 = b.velocity.length_sq();
		if (v2 > max_v * max_v)
			b.velocity = b.velocity * (max_v / std::sqrt(v2));
		float av2 = b.angular_vel.length_sq();
		if (av2 > max_v * max_v)
			b.angular_vel = b.angular_vel * (max_v / std::sqrt(av2));
	}
}

// ============================================================
// sanitize_bodies  (NaN guard — corrupted state quarantine)
// ============================================================
void PhysicsWorld::sanitize_bodies() {
	for (auto &b : m_bodies) {
		if (!b.active)
			continue;
		if (b.position.has_nan() || b.velocity.has_nan() ||
				b.angular_vel.has_nan() || b.rotation.has_nan()) {
			// Reset to safe state above terrain
			b.velocity = Vec3::zero();
			b.angular_vel = Vec3::zero();
			b.rotation = Quat::identity();
			b.position = Vec3{ 0.0f, 0.0f, 0.0f }; // reset to valid coords before height sample
			float h = sample_height(0.0f, 0.0f);
			b.position.y = h + b.sphere_radius + 1.0f;
		}
	}
}
