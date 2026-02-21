#pragma once
#include <cstdint>

struct SimConfig {
	// --- Population / Evolution ---
	int population_size = 20;
	int max_species = 10;
	float species_compat_threshold = 3.0f;
	float c1_excess = 1.0f;
	float c2_disjoint = 1.0f;
	float c3_weight_diff = 0.4f;
	float crossover_rate = 0.75f;
	int elites_per_species = 1;
	int stagnation_limit = 15;

	// --- Mutation rates ---
	float weight_mutation_rate = 0.80f;
	float weight_perturb_rate = 0.90f;
	float weight_perturb_power = 0.10f;
	float add_node_rate = 0.03f;
	float add_connection_rate = 0.05f;
	float add_segment_rate = 0.03f;
	float remove_segment_rate = 0.01f;

	// --- Body / Creature ---
	int min_segments = 2;
	int max_segments = 10;
	float initial_energy = 100.0f;
	float energy_drain_per_step = 0.01f;
	float energy_from_food = 50.0f;
	float food_pickup_radius = 2.0f;

	// --- Physics ---
	float physics_dt = 1.0f / 60.0f;
	float gravity = -9.8f;
	float ground_restitution = 0.1f;
	float ground_friction = 0.8f;
	float linear_damping = 0.30f;
	float angular_damping = 0.50f;
	float joint_max_torque = 50.0f;
	float joint_kp = 50.0f;
	float joint_kd = 5.0f;
	float body_density = 1.0f;
	float max_body_velocity = 30.0f;

	// --- World / Terrain ---
	int terrain_size = 64;
	float terrain_cell_scale = 2.0f;
	float terrain_height_scale = 5.0f;
	float terrain_roughness = 0.5f;
	int food_count = 50;
	int max_steps_per_generation = 800;
	float world_spawn_radius = 60.0f;

	// --- Neural ---
	int initial_hidden_nodes = 0;
	uint32_t rng_seed = 12345u;
};
