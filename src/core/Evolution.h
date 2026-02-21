#pragma once
#include "Creature.h"
#include "Genome.h"
#include "SimConfig.h"
#include <map>
#include <string>
#include <vector>

// ============================================================
// Species
// ============================================================
struct Species {
	int id = 0;
	std::string name = "";  // "Gen 2-A", "Gen 3-B", etc.
	Genome representative;
	std::vector<int> member_indices; // indices into the population vector
	std::vector<int> member_genome_ids; // genome.id of each member (for remapping to creature indices)
	float best_fitness = 0.0f;
	float avg_fitness = 0.0f;
	float adj_fitness_sum = 0.0f;
	int stagnation_counter = 0;
	int age = 0;
};

// ============================================================
// SpeciesInfo  –  per-species display info
// ============================================================
struct SpeciesInfo {
	std::string name;
	int member_count = 0;
	int alive_count = 0;
	float best_fitness = 0.0f;
	float avg_fitness = 0.0f;
};

// ============================================================
// Evolution  –  NEAT speciation, selection, and reproduction
// ============================================================
class Evolution {
public:
	explicit Evolution(const SimConfig &cfg, uint32_t rng_seed);

	// Seed the initial population (call once before first generation)
	void seed_population(std::vector<Genome> &pop);

	// Run a full evolution cycle: speciate -> compute fitness -> produce offspring
	// Returns a new population of exactly cfg.population_size genomes
	std::vector<Genome> evolve(const std::vector<Genome> &population);

	// ---- Accessors for stats ----
	int generation() const { return m_generation; }
	float best_fitness_ever() const { return m_best_fitness_ever; }
	int species_count() const { return static_cast<int>(m_species.size()); }
	float last_avg_fitness() const { return m_last_avg_fitness; }

	// Get detailed per-species info (requires creature list for alive count)
	std::vector<SpeciesInfo> get_species_info(const std::vector<struct Creature> &creatures) const;

	// Access species list for remapping (World::remap_species_to_creatures)
	std::vector<Species> &get_species_mut() { return m_species; }
	const std::vector<Species> &get_species() const { return m_species; }

	// Shared innovation tracker — used by all mutations in a generation
	InnovationTracker innov;

private:
	void speciate(const std::vector<Genome> &pop);
	void compute_adjusted_fitness(std::vector<Genome> &pop);
	void eliminate_stagnant_species(const std::vector<Genome> &pop);
	void cull_species_bottom_half(const std::vector<Genome> &pop_ref,
			std::vector<Genome> &pop);
	std::vector<int> compute_offspring_counts(float total_adj_sum) const;
	const Genome &tournament_select(const std::vector<int> &members,
			const std::vector<Genome> &pop) const;
	void update_representatives(const std::vector<Genome> &pop);

	// Helper for generating species names
	std::string create_species_name(int generation);

	std::vector<Species> m_species;
	int m_next_species_id = 0;
	int m_next_genome_id = 0;
	int m_generation = 0;
	float m_best_fitness_ever = 0.0f;
	float m_last_avg_fitness = 0.0f;
	std::map<int, int> m_species_created_per_gen;  // generation -> count created that gen

	const SimConfig &m_cfg;
	mutable uint32_t m_rng;

	float rand_float() const;
	float rand_range(float lo, float hi) const;
	int rand_int(int lo, int hi_inclusive) const;
	static uint32_t xorshift(uint32_t &s);
};
