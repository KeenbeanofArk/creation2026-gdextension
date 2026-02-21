#include "World.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

// ============================================================
// RNG
// ============================================================
uint32_t World::xorshift(uint32_t &s) {
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;
	return s;
}
float World::rand_float() {
	return static_cast<float>(xorshift(m_rng)) / 4294967295.0f;
}
float World::rand_range(float lo, float hi) {
	return lo + rand_float() * (hi - lo);
}

// ============================================================
// Construction
// ============================================================
World::World(SimConfig cfg) : m_cfg(cfg), m_physics(m_cfg), m_evolution(m_cfg, m_cfg.rng_seed + 1), m_rng(m_cfg.rng_seed) {}

// ============================================================
// initialize
// ============================================================
void World::initialize() {
	m_step_count = 0;
	m_alive_count = 0;

	generate_terrain(); // must come before spawn_food (needs heights)
	spawn_food();

	std::vector<Genome> initial_pop;
	m_evolution.seed_population(initial_pop);
	respawn_population(initial_pop);
}

// ============================================================
// generate_terrain  (diamond-square algorithm)
// ============================================================
void World::generate_terrain() {
	int N = m_cfg.terrain_size;
	m_heights.assign(static_cast<size_t>(N * N), 0.0f);

	// Size must be 2^n + 1 for diamond-square; find nearest power
	// We just use the requested size and clamp access.
	// Initialise corners
	auto idx = [&](int x, int y) -> float & {
		x = std::max(0, std::min(N - 1, x));
		y = std::max(0, std::min(N - 1, y));
		return m_heights[y * N + x];
	};

	// Seed corners with uniform value
	idx(0, 0) = 0.5f;
	idx(N - 1, 0) = 0.5f;
	idx(0, N - 1) = 0.5f;
	idx(N - 1, N - 1) = 0.5f;

	float roughness = m_cfg.terrain_roughness;
	float scale = roughness;

	for (int step = N - 1; step > 1; step /= 2) {
		int half = step / 2;

		// Diamond step
		for (int y = 0; y < N - 1; y += step) {
			for (int x = 0; x < N - 1; x += step) {
				float avg = (idx(x, y) +
									idx(x + step, y) +
									idx(x, y + step) +
									idx(x + step, y + step)) *
						0.25f;
				idx(x + half, y + half) = avg + rand_range(-scale, scale);
			}
		}

		// Square step
		for (int y = 0; y < N; y += half) {
			for (int x = (y + half) % step; x < N; x += step) {
				std::vector<float> neighbours;
				if (x - half >= 0)
					neighbours.push_back(idx(x - half, y));
				if (x + half < N)
					neighbours.push_back(idx(x + half, y));
				if (y - half >= 0)
					neighbours.push_back(idx(x, y - half));
				if (y + half < N)
					neighbours.push_back(idx(x, y + half));
				if (neighbours.empty())
					continue;
				float avg = 0.0f;
				for (float v : neighbours)
					avg += v;
				avg /= static_cast<float>(neighbours.size());
				idx(x, y) = avg + rand_range(-scale, scale);
			}
		}

		scale *= roughness * 0.5f;
	}

	// Normalise to [0,1]
	float mn = m_heights[0], mx = m_heights[0];
	for (float v : m_heights) {
		if (v < mn)
			mn = v;
		if (v > mx)
			mx = v;
	}
	float range = mx - mn;
	if (range < 1e-6f)
		range = 1.0f;
	for (float &v : m_heights)
		v = (v - mn) / range;

	m_physics.set_heightmap(m_heights, N,
			m_cfg.terrain_cell_scale,
			m_cfg.terrain_height_scale);
}

// ============================================================
// spawn_food
// ============================================================
void World::spawn_food() {
	m_food.clear();
	m_food.reserve(m_cfg.food_count);
	m_food_collected.assign(m_cfg.food_count, false);

	for (int i = 0; i < m_cfg.food_count; ++i) {
		FoodItem fi;
		fi.active = true;
		fi.energy_value = m_cfg.energy_from_food;
		Vec3 pos = random_ground_position();
		float h = m_physics.sample_height(pos.x, pos.z);
		fi.position = { pos.x, h + 0.5f, pos.z };
		m_food.push_back(fi);
	}
}

// ============================================================
// respawn_population
// ============================================================
void World::respawn_population(const std::vector<Genome> &new_genomes) {
	m_physics.clear_all();
	m_creatures.clear();
	m_creatures.reserve(new_genomes.size());

	float world_r = m_cfg.world_spawn_radius;

	for (const auto &g : new_genomes) {
		Creature c;
		c.id = g.id;
		c.genome = g;

		Vec3 spawn = random_ground_position();
		float h = m_physics.sample_height(spawn.x, spawn.z);
		spawn.y = h + 2.0f; // a bit above ground

		c.build_body(m_physics, spawn, m_cfg);
		m_creatures.push_back(std::move(c));
	}

	check_alive_count();
}

// ============================================================
// random_ground_position
// ============================================================
Vec3 World::random_ground_position() {
	float half = m_cfg.world_spawn_radius;
	float x = rand_range(-half, half);
	float z = rand_range(-half, half);
	float y = m_physics.sample_height(x, z);
	return { x, y, z };
}

// ============================================================
// step
// ============================================================
void World::step() {
	// Advance physics
	m_physics.step();

	// Reset per-step food collection flags
	std::fill(m_food_collected.begin(), m_food_collected.end(), false);

	// Step each creature
	for (auto &c : m_creatures) {
		if (!c.alive)
			continue;
		c.step(m_physics, m_food, m_food_collected, m_cfg);
	}

	// Apply food collected flags back to food items
	for (int i = 0; i < static_cast<int>(m_food.size()); ++i) {
		if (m_food_collected[i])
			m_food[i].active = false;
	}

	// Sync fitness back to genome (for evolution)
	for (auto &c : m_creatures)
		c.genome.fitness = c.fitness;

	check_alive_count();
	respawn_out_of_bounds();
	++m_step_count;
}

// ============================================================
// is_generation_done
// ============================================================
bool World::is_generation_done() const {
	if (m_step_count >= m_cfg.max_steps_per_generation)
		return true;
	if (m_alive_count == 0 && !m_creatures.empty())
		return true;
	return false;
}

// ============================================================
// next_generation
// ============================================================
void World::next_generation() {
	// Collect genomes with accumulated fitness
	std::vector<Genome> current_pop;
	current_pop.reserve(m_creatures.size());
	for (auto &c : m_creatures) {
		c.genome.fitness = c.fitness;
		current_pop.push_back(c.genome);
	}

	std::vector<Genome> new_pop = m_evolution.evolve(current_pop);

	// Respawn food
	spawn_food();

	// Respawn creatures with new genomes
	respawn_population(new_pop);

	// Remap species member_indices to new creature indices
	remap_species_to_creatures();

	fprintf(stderr, "[GEN] gen=%d  alive=%d/%d  best=%.3f  avg=%.3f  species=%d\n",
		m_evolution.generation(),
		m_alive_count, (int)m_creatures.size(),
		m_evolution.best_fitness_ever(),
		m_evolution.last_avg_fitness(),
		m_evolution.species_count());
	fflush(stderr);

	m_step_count = 0;
}

// ============================================================
// check_alive_count
// ============================================================
void World::check_alive_count() {
	m_alive_count = 0;
	for (const auto &c : m_creatures)
		if (c.alive)
			++m_alive_count;
}

// ============================================================
// respawn_out_of_bounds
// ============================================================
void World::respawn_out_of_bounds() {
	float half = m_cfg.terrain_size * m_cfg.terrain_cell_scale * 0.5f + 10.0f;
	for (auto &c : m_creatures) {
		if (!c.alive || c.segments.empty())
			continue;
		Vec3 pos = c.get_root_position(m_physics);
		// If creature falls off the edge or below world, teleport back
		if (std::abs(pos.x) > half || std::abs(pos.z) > half || pos.y < -20.0f) {
			Vec3 new_pos = random_ground_position();
			new_pos.y += 3.0f;
			// Move root body
			int body_idx = c.segments[0].physics_body_idx;
			if (body_idx >= 0) {
				Vec3 delta = new_pos - m_physics.body(body_idx).position;
				// Shift all bodies
				for (const auto &seg : c.segments) {
					if (seg.physics_body_idx >= 0) {
						m_physics.body(seg.physics_body_idx).position += delta;
						m_physics.body(seg.physics_body_idx).velocity = Vec3::zero();
						m_physics.body(seg.physics_body_idx).angular_vel = Vec3::zero();
					}
				}
			}
		}
	}
}

// ============================================================
// remap_species_to_creatures
// ============================================================
void World::remap_species_to_creatures() {
	// Clear all species member lists
	for (auto &sp : m_evolution.get_species_mut()) {
		sp.member_indices.clear();
	}

	// Rebuild member_indices by iterating through creatures
	// Each creature's genome has a species_id that was set during evolution
	for (int i = 0; i < static_cast<int>(m_creatures.size()); ++i) {
		int species_id = m_creatures[i].genome.species_id;
		if (species_id < 0) {
			// Should not happen, but skip if no species assigned
			continue;
		}

		// Find the species with this ID and add the creature index
		for (auto &sp : m_evolution.get_species_mut()) {
			if (sp.id == species_id) {
				sp.member_indices.push_back(i);
				break;
			}
		}
	}
}

// ============================================================
// set_config
// ============================================================
void World::set_config(const SimConfig &cfg) {
	m_cfg = cfg;
}
