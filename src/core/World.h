#pragma once
#include "Creature.h"
#include "Evolution.h"
#include "Physics.h"
#include "SimConfig.h"
#include "SimTypes.h"
#include <memory>
#include <vector>

// ============================================================
// World  –  top-level simulation orchestrator
// ============================================================
class World {
public:
	explicit World(SimConfig cfg);
	~World() = default;

	// One-time init: generate terrain, spawn food, seed population
	void initialize();

	// Advance simulation by one physics step
	void step();

	// Trigger evolution: evaluate, reproduce, respawn
	void next_generation();

	bool is_generation_done() const;

	// ---- Read accessors (for binding layer) ----
	int creature_count() const { return static_cast<int>(m_creatures.size()); }
	const Creature &creature(int idx) const { return m_creatures[idx]; }
	Creature &creature(int idx) { return m_creatures[idx]; }
	const PhysicsWorld &physics() const { return m_physics; }
	PhysicsWorld &physics() { return m_physics; }
	const Evolution &evolution() const { return m_evolution; }
	Evolution &evolution() { return m_evolution; }
	const std::vector<FoodItem> &food() const { return m_food; }
	const std::vector<float> &heightmap() const { return m_heights; }
	const SimConfig &config() const { return m_cfg; }

	// ---- Statistics ----
	int generation() const { return m_evolution.generation(); }
	float best_fitness() const { return m_evolution.best_fitness_ever(); }
	float avg_fitness() const { return m_evolution.last_avg_fitness(); }
	int alive_count() const { return m_alive_count; }
	int species_count() const { return m_evolution.species_count(); }
	int step_count() const { return m_step_count; }
	int terrain_size() const { return m_cfg.terrain_size; }

	// Update the simulation config (effective from next_generation())
	void set_config(const SimConfig &cfg);

private:
	void generate_terrain(); // diamond-square midpoint displacement
	void spawn_food();
	void respawn_population(const std::vector<Genome> &new_genomes);
	Vec3 random_ground_position();
	void check_alive_count();
	void respawn_out_of_bounds(); // teleport creatures that fall off world
	void remap_species_to_creatures(); // remap species member_indices after respawn

	SimConfig m_cfg;
	PhysicsWorld m_physics;
	Evolution m_evolution;
	std::vector<Creature> m_creatures;
	std::vector<FoodItem> m_food;
	std::vector<bool> m_food_collected;
	std::vector<float> m_heights;

	int m_step_count = 0;
	int m_alive_count = 0;
	uint32_t m_rng;

	float rand_float();
	float rand_range(float lo, float hi);
	static uint32_t xorshift(uint32_t &s);
};
