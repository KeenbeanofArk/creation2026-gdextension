#pragma once
#include "../core/Creature.h"
#include "../core/Physics.h"
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <vector>

// ============================================================
// CreatureBinding  –  visual representation of one Creature
// ============================================================
class CreatureBinding : public godot::Node3D {
	GDCLASS(CreatureBinding, godot::Node3D)

public:
	CreatureBinding() = default;
	~CreatureBinding() override = default;

	// Rebuild MeshInstance3D children from the creature's segment list.
	// Call once after the creature's body has been built.
	void build_from_creature(const Creature &creature, const PhysicsWorld &physics);

	// Sync all segment transforms from physics state — call every rendered frame.
	void sync_transforms(const Creature &creature, const PhysicsWorld &physics);

	// Visual feedback helpers
	void set_energy_level(float normalized); // 0=no energy (red tint), 1=full (normal)
	void set_dead(bool dead);

protected:
	static void _bind_methods();

private:
	std::vector<godot::MeshInstance3D *> m_meshes;
	std::vector<godot::Ref<godot::StandardMaterial3D>> m_materials;

	// Base colours per segment (from gene), cached for energy tinting
	struct SegColor {
		float r, g, b;
	};
	std::vector<SegColor> m_base_colors;

	void clear_meshes();
	void create_segment_mesh(int seg_local_idx, const BodySegmentGene &gene);

	static godot::Transform3D to_godot_transform(Vec3 pos, Quat rot);
	static godot::Vector3 to_godot_v3(Vec3 v);
	static godot::Quaternion to_godot_quat(Quat q);
};
