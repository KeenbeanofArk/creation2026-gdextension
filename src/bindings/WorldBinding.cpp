#include "WorldBinding.h"
#include "CreatureBinding.h"
#include "TerrainBinding.h"
#include <cstdio>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// ============================================================
// Construction
// ============================================================
WorldBinding::WorldBinding() {
	m_config = SimConfig{};
}

WorldBinding::~WorldBinding() {
	destroy_all_visuals();
}

// ============================================================
// _bind_methods
// ============================================================
void WorldBinding::_bind_methods() {
	// ---- GDScript callable methods ----
	ClassDB::bind_method(D_METHOD("restart"), &WorldBinding::restart);
	ClassDB::bind_method(D_METHOD("toggle_pause"), &WorldBinding::toggle_pause);
	ClassDB::bind_method(D_METHOD("is_paused"), &WorldBinding::is_paused);
	ClassDB::bind_method(D_METHOD("get_species_details"), &WorldBinding::get_species_details);
	ClassDB::bind_method(D_METHOD("get_species_detailed_info", "species_index"), &WorldBinding::get_species_detailed_info);
	ClassDB::bind_method(D_METHOD("get_creature_detail", "creature_index"), &WorldBinding::get_creature_detail);
	ClassDB::bind_method(D_METHOD("get_generation"), &WorldBinding::get_generation);
	ClassDB::bind_method(D_METHOD("get_best_fitness"), &WorldBinding::get_best_fitness);
	ClassDB::bind_method(D_METHOD("get_avg_fitness"), &WorldBinding::get_avg_fitness);
	ClassDB::bind_method(D_METHOD("get_alive_count"), &WorldBinding::get_alive_count);
	ClassDB::bind_method(D_METHOD("get_species_count"), &WorldBinding::get_species_count);
	ClassDB::bind_method(D_METHOD("get_step_count"), &WorldBinding::get_step_count);
	ClassDB::bind_method(D_METHOD("get_creature_visual", "idx"), &WorldBinding::get_creature_visual);

	// ---- Signals ----
	ADD_SIGNAL(MethodInfo("generation_complete",
			PropertyInfo(Variant::INT, "generation"),
			PropertyInfo(Variant::FLOAT, "best_fitness"),
			PropertyInfo(Variant::FLOAT, "avg_fitness"),
			PropertyInfo(Variant::INT, "species_count")));
	ADD_SIGNAL(MethodInfo("creature_died",
			PropertyInfo(Variant::INT, "creature_id")));

	// ---- Properties (population / evolution) ----
#define BIND_PROP(name, type, getter, setter)                                \
	ClassDB::bind_method(D_METHOD(#getter), &WorldBinding::getter);          \
	ClassDB::bind_method(D_METHOD(#setter, "value"), &WorldBinding::setter); \
	ADD_PROPERTY(PropertyInfo(type, name), #setter, #getter);

	BIND_PROP("population_size", Variant::INT, get_population_size, set_population_size)
	BIND_PROP("food_count", Variant::INT, get_food_count, set_food_count)
	BIND_PROP("terrain_size", Variant::INT, get_terrain_size, set_terrain_size)
	BIND_PROP("max_segments", Variant::INT, get_max_segments, set_max_segments)
	BIND_PROP("max_steps_per_gen", Variant::INT, get_max_steps_per_gen, set_max_steps_per_gen)
	BIND_PROP("weight_mutation_rate", Variant::FLOAT, get_weight_mutation_rate, set_weight_mutation_rate)
	BIND_PROP("add_node_rate", Variant::FLOAT, get_add_node_rate, set_add_node_rate)
	BIND_PROP("add_connection_rate", Variant::FLOAT, get_add_connection_rate, set_add_connection_rate)
	BIND_PROP("add_segment_rate", Variant::FLOAT, get_add_segment_rate, set_add_segment_rate)
	BIND_PROP("initial_energy", Variant::FLOAT, get_initial_energy, set_initial_energy)
	BIND_PROP("energy_drain", Variant::FLOAT, get_energy_drain, set_energy_drain)
	BIND_PROP("food_pickup_radius", Variant::FLOAT, get_food_pickup_radius, set_food_pickup_radius)
	BIND_PROP("terrain_height_scale", Variant::FLOAT, get_terrain_height_scale, set_terrain_height_scale)
	BIND_PROP("terrain_roughness", Variant::FLOAT, get_terrain_roughness, set_terrain_roughness)
	BIND_PROP("joint_max_torque", Variant::FLOAT, get_joint_max_torque, set_joint_max_torque)
	BIND_PROP("rng_seed", Variant::INT, get_rng_seed, set_rng_seed)
#undef BIND_PROP
}

// ============================================================
// _ready
// ============================================================
void WorldBinding::_ready() {
	if (Engine::get_singleton()->is_editor_hint())
		return;

	m_terrain = get_node<TerrainBinding>("Terrain");
	m_creatures_node = get_node<Node3D>("Creatures");
	m_food_node = get_node<Node3D>("Food");

	restart();
}

// ============================================================
// restart
// ============================================================
void WorldBinding::restart() {
	destroy_all_visuals();

	m_world = std::make_unique<World>(m_config);
	m_world->initialize();

	// Build terrain mesh
	if (m_terrain) {
		m_terrain->build_from_heightmap(
				m_world->heightmap(),
				m_world->terrain_size(),
				m_config.terrain_cell_scale,
				m_config.terrain_height_scale);
	}

	rebuild_all_visuals();
	rebuild_food_visuals();

	m_running = false; // start paused so engine is ready
	m_accum_dt = 0.0f;
	m_running = true;
}

// ============================================================
// toggle_pause
// ============================================================
void WorldBinding::toggle_pause() {
	if (m_world) {
		m_running = !m_running;
	}
}

// ============================================================
// is_paused
// ============================================================
bool WorldBinding::is_paused() const {
	return !m_running;
}

// ============================================================
// get_species_details
// ============================================================
godot::String WorldBinding::get_species_details() const {
	if (!m_world)
		return "";

	// Build creature list since get_species_info needs it
	std::vector<Creature> creature_list;
	for (int i = 0; i < m_world->creature_count(); ++i) {
		creature_list.push_back(m_world->creature(i));
	}

	auto species_infos = m_world->evolution().get_species_info(creature_list);

	godot::String result;
	for (const auto &info : species_infos) {
		char buffer[256];
		snprintf(buffer, sizeof(buffer),
			"%s:  %d alive / %d total  |  Best: %.2f  |  Avg: %.2f\n",
			info.name.c_str(), info.alive_count, info.member_count,
			info.best_fitness, info.avg_fitness
		);
		result += buffer;
	}
	return result;
}

// ============================================================
// get_species_detailed_info
// ============================================================
godot::String WorldBinding::get_species_detailed_info(int species_index) const {
	if (!m_world) return "";

	const Evolution &evo = m_world->evolution();
	const auto &species = evo.get_species();

	if (species_index < 0 || species_index >= static_cast<int>(species.size())) {
		return "";
	}

	const Species &sp = species[species_index];

	godot::String result;
	char buffer[512];

	snprintf(buffer, sizeof(buffer),
		"=== %s (ID: %d) ===\n"
		"Members: %d  |  Alive: %d\n"
		"Generation: %d  |  Age: %d\n"
		"Best Fitness: %.2f  |  Avg Fitness: %.2f\n"
		"Stagnation: %d / 20\n\n"
		"MEMBER CREATURES:\n",
		sp.name.c_str(), sp.id,
		static_cast<int>(sp.member_indices.size()), alive_count_in_species(sp),
		evo.generation(), sp.age,
		sp.best_fitness, sp.avg_fitness,
		sp.stagnation_counter
	);
	result += buffer;

	// List details for each creature in species
	for (int idx : sp.member_indices) {
		if (idx >= 0 && idx < m_world->creature_count()) {
			const Creature &creature = m_world->creature(idx);
			snprintf(buffer, sizeof(buffer),
				"[%d] E:%.1f F:%.2f S:%.1f M:%.1f %s\n",
				idx, creature.energy, creature.fitness,
				get_speed(creature), get_mass(creature),
				creature.alive ? "ALIVE" : "DEAD"
			);
			result += buffer;
		}
	}

	return result;
}

// ============================================================
// get_creature_detail
// ============================================================
godot::String WorldBinding::get_creature_detail(int creature_index) const {
	if (!m_world || creature_index < 0 || creature_index >= m_world->creature_count()) {
		return "";
	}

	const Creature &creature = m_world->creature(creature_index);
	const Genome &genome = creature.genome;
	const Brain &brain = creature.brain;

	godot::String result;
	char buffer[512];

	// Get position and speed
	Vec3 root_pos = creature.get_root_position(m_world->physics());
	float speed = get_speed(creature);
	float mass = get_mass(creature);

	snprintf(buffer, sizeof(buffer),
		"=== Creature [%d] ===\n"
		"Status: %s | Energy: %.2f | Fitness: %.2f\n"
		"Position: (%.1f, %.1f, %.1f) | Speed: %.2f m/s\n"
		"Mass: %.2f kg\n\n"
		"BODY STRUCTURE:\n"
		"Segments: %d | Joints: %d\n",
		creature_index,
		creature.alive ? "Alive" : "Dead",
		creature.energy, creature.fitness,
		root_pos.x, root_pos.y, root_pos.z, speed,
		mass,
		static_cast<int>(creature.segments.size()),
		static_cast<int>(genome.segments.size()) - 1
	);
	result += buffer;

	// Add segment details
	for (int i = 0; i < static_cast<int>(creature.segments.size()); ++i) {
		const auto &seg = creature.segments[i];
		const auto &gene = seg.gene;
		snprintf(buffer, sizeof(buffer),
			"  [%d] %s (%.1f, %.1f, %.1f) rgb(%.2f, %.2f, %.2f) %s\n",
			i, shape_name(gene.shape), gene.sx, gene.sy, gene.sz,
			gene.r, gene.g, gene.b, gene.enabled ? "ON" : "OFF"
		);
		result += buffer;
	}

	// Add brain info
	int hidden_count = brain.neuron_count() - brain.input_count() - brain.output_count();
	if (hidden_count < 0) hidden_count = 0;

	snprintf(buffer, sizeof(buffer),
		"\nNEURAL NETWORK:\n"
		"Inputs: %d | Outputs: %d | Hidden: %d | Connections: %d\n"
		"Valid: %s\n",
		brain.input_count(), brain.output_count(),
		hidden_count,
		brain.synapse_count(),
		brain.is_valid() ? "Yes" : "No (cyclic)"
	);
	result += buffer;

	// Add genome info
	snprintf(buffer, sizeof(buffer),
		"\nGENETIC INFO:\n"
		"Nodes: %d | Connections: %d\n"
		"Genome ID: %d | Species ID: %d\n",
		static_cast<int>(genome.nodes.size()),
		static_cast<int>(genome.connections.size()),
		genome.id, genome.species_id
	);
	result += buffer;

	return result;
}

// ============================================================
// Helper methods for detail serialization
// ============================================================
int WorldBinding::alive_count_in_species(const Species &sp) const {
	int count = 0;
	for (int idx : sp.member_indices) {
		if (idx >= 0 && idx < m_world->creature_count()) {
			if (m_world->creature(idx).alive) count++;
		}
	}
	return count;
}

float WorldBinding::get_speed(const Creature &creature) const {
	if (creature.segments.empty()) return 0.0f;
	const RigidBody &root = m_world->physics().body(creature.segments[0].physics_body_idx);
	return root.velocity.length();
}

float WorldBinding::get_mass(const Creature &creature) const {
	float total = 0.0f;
	for (const auto &seg : creature.segments) {
		const RigidBody &body = m_world->physics().body(seg.physics_body_idx);
		if (body.inv_mass > 1e-6f) {
			total += 1.0f / body.inv_mass;
		}
	}
	return total;
}

const char *WorldBinding::shape_name(ShapeType type) const {
	switch (type) {
		case ShapeType::SPHERE: return "SPHERE";
		case ShapeType::BOX: return "BOX";
		case ShapeType::CAPSULE: return "CAPSULE";
		default: return "UNKNOWN";
	}
}


void WorldBinding::_physics_process(double delta) {
	if (!m_running || !m_world)
		return;
	if (Engine::get_singleton()->is_editor_hint())
		return;

	m_accum_dt += static_cast<float>(delta);
	float dt = m_config.physics_dt;

	// Step simulation in fixed increments
	int max_steps_per_frame = 4; // guard against spiral of death
	int steps = 0;
	while (m_accum_dt >= dt && steps < max_steps_per_frame) {
		m_world->step();
		m_accum_dt -= dt;
		++steps;
	}

	// Check generation done
	if (m_world->is_generation_done()) {
		m_world->next_generation();

		int gen = m_world->generation();
		float best = m_world->best_fitness();
		float avg = m_world->avg_fitness();
		int species = m_world->species_count();

		// Rebuild terrain (new generation may have same terrain; no-op if unchanged)
		if (m_terrain) {
			m_terrain->build_from_heightmap(
					m_world->heightmap(),
					m_world->terrain_size(),
					m_config.terrain_cell_scale,
					m_config.terrain_height_scale);
		}

		rebuild_all_visuals();
		rebuild_food_visuals();

		emit_signal("generation_complete",
				gen, static_cast<double>(best), static_cast<double>(avg), species);
	}

	sync_visuals_to_simulation();
	sync_food_visuals();
}

// ============================================================
// rebuild_all_visuals
// ============================================================
void WorldBinding::rebuild_all_visuals() {
	// Remove old creature visuals
	for (auto *cv : m_visuals) {
		if (cv && m_creatures_node) {
			m_creatures_node->remove_child(cv);
			cv->queue_free();
		}
	}
	m_visuals.clear();

	if (!m_world || !m_creatures_node)
		return;

	m_visuals.resize(m_world->creature_count(), nullptr);
	for (int i = 0; i < m_world->creature_count(); ++i)
		create_creature_visual(i);
}

void WorldBinding::create_creature_visual(int idx) {
	if (!m_creatures_node || !m_world)
		return;
	if (idx < 0 || idx >= m_world->creature_count())
		return;

	CreatureBinding *cv = memnew(CreatureBinding);
	m_creatures_node->add_child(cv);

	const Creature &c = m_world->creature(idx);
	cv->build_from_creature(c, m_world->physics());
	m_visuals[idx] = cv;
}

// ============================================================
// sync_visuals_to_simulation
// ============================================================
void WorldBinding::sync_visuals_to_simulation() {
	if (!m_world)
		return;
	for (int i = 0; i < static_cast<int>(m_visuals.size()); ++i) {
		if (!m_visuals[i])
			continue;
		const Creature &c = m_world->creature(i);
		m_visuals[i]->sync_transforms(c, m_world->physics());

		float energy_norm = c.energy / m_config.initial_energy;
		m_visuals[i]->set_energy_level(energy_norm);

		if (!c.alive)
			m_visuals[i]->set_dead(true);
	}
}

// ============================================================
// rebuild_food_visuals
// ============================================================
void WorldBinding::rebuild_food_visuals() {
	for (auto *mi : m_food_meshes) {
		if (mi && m_food_node) {
			m_food_node->remove_child(mi);
			mi->queue_free();
		}
	}
	m_food_meshes.clear();

	if (!m_world || !m_food_node)
		return;

	// Create a shared sphere mesh + material for all food items
	Ref<SphereMesh> sphere_mesh;
	sphere_mesh.instantiate();
	sphere_mesh->set_radius(0.4f);
	sphere_mesh->set_height(0.8f);

	Ref<StandardMaterial3D> food_mat;
	food_mat.instantiate();
	food_mat->set_albedo(Color(0.9f, 0.8f, 0.1f)); // yellow
	food_mat->set_roughness(0.5f);
	food_mat->set_feature(StandardMaterial3D::FEATURE_EMISSION, true);
	food_mat->set_emission(Color(0.4f, 0.3f, 0.0f));

	const auto &food = m_world->food();
	for (const auto &fi : food) {
		MeshInstance3D *mi = memnew(MeshInstance3D);
		mi->set_mesh(sphere_mesh);
		mi->set_material_override(food_mat);
		mi->set_position(Vector3(fi.position.x, fi.position.y, fi.position.z));
		m_food_node->add_child(mi);
		m_food_meshes.push_back(mi);
	}
}

// ============================================================
// sync_food_visuals
// ============================================================
void WorldBinding::sync_food_visuals() {
	if (!m_world)
		return;
	const auto &food = m_world->food();
	int count = std::min(static_cast<int>(food.size()),
			static_cast<int>(m_food_meshes.size()));
	for (int i = 0; i < count; ++i) {
		if (m_food_meshes[i])
			m_food_meshes[i]->set_visible(food[i].active);
	}
}

// ============================================================
// destroy_all_visuals
// ============================================================
void WorldBinding::destroy_all_visuals() {
	for (auto *cv : m_visuals) {
		if (cv && m_creatures_node) {
			m_creatures_node->remove_child(cv);
			cv->queue_free();
		}
	}
	m_visuals.clear();

	for (auto *mi : m_food_meshes) {
		if (mi && m_food_node) {
			m_food_node->remove_child(mi);
			mi->queue_free();
		}
	}
	m_food_meshes.clear();
}

// ============================================================
// Stat accessors
// ============================================================
int WorldBinding::get_generation() const { return m_world ? m_world->generation() : 0; }
float WorldBinding::get_best_fitness() const { return m_world ? m_world->best_fitness() : 0.0f; }
float WorldBinding::get_avg_fitness() const { return m_world ? m_world->avg_fitness() : 0.0f; }
int WorldBinding::get_alive_count() const { return m_world ? m_world->alive_count() : 0; }
int WorldBinding::get_species_count() const { return m_world ? m_world->species_count() : 0; }
int WorldBinding::get_step_count() const { return m_world ? m_world->step_count() : 0; }

// ============================================================
// Property getters / setters
// ============================================================
void WorldBinding::set_population_size(int v) { m_config.population_size = v; }
int WorldBinding::get_population_size() const { return m_config.population_size; }

void WorldBinding::set_food_count(int v) { m_config.food_count = v; }
int WorldBinding::get_food_count() const { return m_config.food_count; }

void WorldBinding::set_terrain_size(int v) { m_config.terrain_size = v; }
int WorldBinding::get_terrain_size() const { return m_config.terrain_size; }

void WorldBinding::set_max_segments(int v) { m_config.max_segments = v; }
int WorldBinding::get_max_segments() const { return m_config.max_segments; }

void WorldBinding::set_max_steps_per_gen(int v) { m_config.max_steps_per_generation = v; }
int WorldBinding::get_max_steps_per_gen() const { return m_config.max_steps_per_generation; }

void WorldBinding::set_weight_mutation_rate(float v) { m_config.weight_mutation_rate = v; }
float WorldBinding::get_weight_mutation_rate() const { return m_config.weight_mutation_rate; }

void WorldBinding::set_add_node_rate(float v) { m_config.add_node_rate = v; }
float WorldBinding::get_add_node_rate() const { return m_config.add_node_rate; }

void WorldBinding::set_add_connection_rate(float v) { m_config.add_connection_rate = v; }
float WorldBinding::get_add_connection_rate() const { return m_config.add_connection_rate; }

void WorldBinding::set_add_segment_rate(float v) { m_config.add_segment_rate = v; }
float WorldBinding::get_add_segment_rate() const { return m_config.add_segment_rate; }

void WorldBinding::set_initial_energy(float v) { m_config.initial_energy = v; }
float WorldBinding::get_initial_energy() const { return m_config.initial_energy; }

void WorldBinding::set_energy_drain(float v) { m_config.energy_drain_per_step = v; }
float WorldBinding::get_energy_drain() const { return m_config.energy_drain_per_step; }

void WorldBinding::set_food_pickup_radius(float v) { m_config.food_pickup_radius = v; }
float WorldBinding::get_food_pickup_radius() const { return m_config.food_pickup_radius; }

void WorldBinding::set_terrain_height_scale(float v) { m_config.terrain_height_scale = v; }
float WorldBinding::get_terrain_height_scale() const { return m_config.terrain_height_scale; }

void WorldBinding::set_terrain_roughness(float v) { m_config.terrain_roughness = v; }
float WorldBinding::get_terrain_roughness() const { return m_config.terrain_roughness; }

void WorldBinding::set_joint_max_torque(float v) { m_config.joint_max_torque = v; }
float WorldBinding::get_joint_max_torque() const { return m_config.joint_max_torque; }

void WorldBinding::set_rng_seed(int v) { m_config.rng_seed = static_cast<uint32_t>(v); }
int WorldBinding::get_rng_seed() const { return static_cast<int>(m_config.rng_seed); }
