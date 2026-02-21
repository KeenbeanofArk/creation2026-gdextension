#include "CreatureBinding.h"
#include <cmath>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/color.hpp>

using namespace godot;

void CreatureBinding::_bind_methods() {
	// No exported methods needed — driven programmatically by WorldBinding
}

// ============================================================
// to_godot helpers
// ============================================================
Vector3 CreatureBinding::to_godot_v3(Vec3 v) {
	return Vector3(v.x, v.y, v.z);
}

Quaternion CreatureBinding::to_godot_quat(Quat q) {
	// Both Godot and our custom math use Y-up right-handed coords.
	// Godot Quaternion is (x, y, z, w) — same component order.
	return Quaternion(q.x, q.y, q.z, q.w);
}

Transform3D CreatureBinding::to_godot_transform(Vec3 pos, Quat rot) {
	Quaternion gq = to_godot_quat(rot.normalized());
	Basis basis(gq);
	return Transform3D(basis, to_godot_v3(pos));
}

// ============================================================
// clear_meshes
// ============================================================
void CreatureBinding::clear_meshes() {
	for (auto *mi : m_meshes) {
		if (mi) {
			remove_child(mi);
			mi->queue_free();
		}
	}
	m_meshes.clear();
	m_materials.clear();
	m_base_colors.clear();
}

// ============================================================
// create_segment_mesh
// ============================================================
void CreatureBinding::create_segment_mesh(int /*seg_local_idx*/, const BodySegmentGene &gene) {
	Ref<Mesh> mesh_res;
	switch (gene.shape) {
		case ShapeType::BOX: {
			Ref<BoxMesh> bm;
			bm.instantiate();
			bm->set_size(Vector3(gene.sx * 2.0f, gene.sy * 2.0f, gene.sz * 2.0f));
			mesh_res = bm;
			break;
		}
		case ShapeType::SPHERE: {
			Ref<SphereMesh> sm;
			sm.instantiate();
			sm->set_radius(gene.sx);
			sm->set_height(gene.sx * 2.0f);
			mesh_res = sm;
			break;
		}
		case ShapeType::CAPSULE: {
			Ref<CapsuleMesh> cm;
			cm.instantiate();
			cm->set_radius(gene.sx);
			cm->set_height(gene.sz * 2.0f + gene.sx * 2.0f);
			mesh_res = cm;
			break;
		}
	}

	Ref<StandardMaterial3D> mat;
	mat.instantiate();
	mat->set_albedo(Color(gene.r, gene.g, gene.b));
	mat->set_roughness(0.7f);
	mat->set_metallic(0.1f);

	MeshInstance3D *mi = memnew(MeshInstance3D);
	mi->set_mesh(mesh_res);
	mi->set_material_override(mat);
	add_child(mi);

	m_meshes.push_back(mi);
	m_materials.push_back(mat);
	m_base_colors.push_back({ gene.r, gene.g, gene.b });
}

// ============================================================
// build_from_creature
// ============================================================
void CreatureBinding::build_from_creature(const Creature &creature,
		const PhysicsWorld &physics) {
	clear_meshes();

	for (const auto &seg : creature.segments) {
		if (!seg.gene.enabled)
			continue;
		create_segment_mesh(static_cast<int>(m_meshes.size()), seg.gene);
	}

	// Initial sync
	sync_transforms(creature, physics);
}

// ============================================================
// sync_transforms
// ============================================================
void CreatureBinding::sync_transforms(const Creature &creature,
		const PhysicsWorld &physics) {
	int mesh_idx = 0;
	for (const auto &seg : creature.segments) {
		if (!seg.gene.enabled)
			continue;
		if (mesh_idx >= static_cast<int>(m_meshes.size()))
			break;
		if (!m_meshes[mesh_idx]) {
			++mesh_idx;
			continue;
		}

		const RigidBody &b = physics.body(seg.physics_body_idx);
		m_meshes[mesh_idx]->set_transform(to_godot_transform(b.position, b.rotation));
		++mesh_idx;
	}
}

// ============================================================
// set_energy_level
// ============================================================
void CreatureBinding::set_energy_level(float normalized) {
	normalized = std::max(0.0f, std::min(1.0f, normalized));
	for (int i = 0; i < static_cast<int>(m_materials.size()); ++i) {
		if (m_materials[i].is_null())
			continue;
		SegColor &bc = m_base_colors[i];
		// Fade to red as energy drops
		float r = bc.r * normalized + (1.0f - normalized);
		float g = bc.g * normalized;
		float b = bc.b * normalized;
		m_materials[i]->set_albedo(Color(r, g, b));
	}
}

// ============================================================
// set_dead
// ============================================================
void CreatureBinding::set_dead(bool dead) {
	for (auto &mat : m_materials) {
		if (mat.is_null())
			continue;
		if (dead) {
			mat->set_albedo(Color(0.35f, 0.35f, 0.35f));
		}
	}
}
