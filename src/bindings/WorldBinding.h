#pragma once
#include "../core/SimConfig.h"
#include "../core/World.h"
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <memory>
#include <vector>

class CreatureBinding;
class TerrainBinding;

// ============================================================
// WorldBinding  –  main simulation node
// Drives the C++ World simulation from _physics_process and
// manages all visual child nodes.
// ============================================================
class WorldBinding : public godot::Node3D {
	GDCLASS(WorldBinding, godot::Node3D)

public:
	WorldBinding();
	~WorldBinding() override;

	// ---- Godot virtuals ----
	void _ready() override;
	void _physics_process(double delta) override;

	// ---- GDScript callable methods ----
	void restart();
	void toggle_pause();
	bool is_paused() const;
	godot::String get_species_details() const;
	godot::String get_species_detailed_info(int species_index) const;
	godot::String get_creature_detail(int creature_index) const;
	int get_generation() const;
	float get_best_fitness() const;
	float get_avg_fitness() const;
	int get_alive_count() const;
	int get_species_count() const;
	int get_step_count() const;

	// Get visual binding for a creature (for selection/highlighting)
	CreatureBinding *get_creature_visual(int idx) const {
		if (idx >= 0 && idx < static_cast<int>(m_visuals.size()))
			return m_visuals[idx];
		return nullptr;
	}

	// ---- Exported SimConfig properties ----
	void set_population_size(int v);
	int get_population_size() const;
	void set_food_count(int v);
	int get_food_count() const;
	void set_terrain_size(int v);
	int get_terrain_size() const;
	void set_max_segments(int v);
	int get_max_segments() const;
	void set_max_steps_per_gen(int v);
	int get_max_steps_per_gen() const;
	void set_weight_mutation_rate(float v);
	float get_weight_mutation_rate() const;
	void set_add_node_rate(float v);
	float get_add_node_rate() const;
	void set_add_connection_rate(float v);
	float get_add_connection_rate() const;
	void set_add_segment_rate(float v);
	float get_add_segment_rate() const;
	void set_initial_energy(float v);
	float get_initial_energy() const;
	void set_energy_drain(float v);
	float get_energy_drain() const;
	void set_food_pickup_radius(float v);
	float get_food_pickup_radius() const;
	void set_terrain_height_scale(float v);
	float get_terrain_height_scale() const;
	void set_terrain_roughness(float v);
	float get_terrain_roughness() const;
	void set_joint_max_torque(float v);
	float get_joint_max_torque() const;
	void set_rng_seed(int v);
	int get_rng_seed() const;

protected:
	static void _bind_methods();

private:
	// ---- Simulation ----
	std::unique_ptr<World> m_world;
	SimConfig m_config;
	bool m_running = false;
	bool m_needs_restart = false;
	float m_accum_dt = 0.0f;

	// ---- Child node references ----
	TerrainBinding *m_terrain = nullptr;
	godot::Node3D *m_creatures_node = nullptr;
	godot::Node3D *m_food_node = nullptr;
	std::vector<CreatureBinding *> m_visuals;
	std::vector<godot::MeshInstance3D *> m_food_meshes;

	// ---- Visual management ----
	void rebuild_all_visuals();
	void create_creature_visual(int idx);
	void sync_visuals_to_simulation();
	void rebuild_food_visuals();
	void sync_food_visuals();
	void destroy_all_visuals();

	// ---- Helper methods for detail serialization ----
	int alive_count_in_species(const Species &sp) const;
	float get_speed(const Creature &creature) const;
	float get_mass(const Creature &creature) const;
	const char *shape_name(ShapeType type) const;
};
