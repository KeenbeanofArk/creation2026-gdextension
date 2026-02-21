#include "Evolution.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>
#include <cstdio>

// ============================================================
// RNG helpers
// ============================================================
uint32_t Evolution::xorshift(uint32_t &s) {
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;
	return s;
}
float Evolution::rand_float() const {
	return static_cast<float>(xorshift(m_rng)) / 4294967295.0f;
}
float Evolution::rand_range(float lo, float hi) const {
	return lo + rand_float() * (hi - lo);
}
int Evolution::rand_int(int lo, int hi_inclusive) const {
	if (lo >= hi_inclusive)
		return lo;
	return lo + static_cast<int>(xorshift(m_rng) % static_cast<uint32_t>(hi_inclusive - lo + 1));
}

// ============================================================
// Construction
// ============================================================
Evolution::Evolution(const SimConfig &cfg, uint32_t rng_seed) : m_cfg(cfg), m_rng(rng_seed) {}

// ============================================================
// seed_population
// ============================================================
void Evolution::seed_population(std::vector<Genome> &pop) {
	innov.new_generation();
	pop.clear();
	pop.reserve(m_cfg.population_size);
	uint32_t rng = m_rng;
	for (int i = 0; i < m_cfg.population_size; ++i) {
		Genome g = Genome::create_minimal(m_next_genome_id++, m_cfg, innov, rng);
		pop.push_back(std::move(g));
	}
	m_rng = rng;
}

// ============================================================
// evolve  (main function called each generation)
// ============================================================
std::vector<Genome> Evolution::evolve(const std::vector<Genome> &population) {
	++m_generation;
	innov.new_generation();

	// Work on a mutable copy so we can compute adj fitness in-place
	std::vector<Genome> pop = population;

	// Update stats
	float total_fit = 0.0f;
	float best_fit = 0.0f;
	for (const auto &g : pop) {
		total_fit += g.fitness;
		if (g.fitness > best_fit)
			best_fit = g.fitness;
	}
	m_last_avg_fitness = pop.empty() ? 0.0f : total_fit / static_cast<float>(pop.size());
	if (best_fit > m_best_fitness_ever)
		m_best_fitness_ever = best_fit;

	// --- Speciation ---
	speciate(pop);
	update_representatives(pop);
	compute_adjusted_fitness(pop);
	eliminate_stagnant_species(pop);

	// --- Compute offspring allocation per species ---
	float total_adj = 0.0f;
	for (const auto &sp : m_species)
		total_adj += sp.adj_fitness_sum;

	std::vector<int> offspring_counts = compute_offspring_counts(total_adj);

	// --- Cull each species to top 50% before producing offspring ---
	cull_species_bottom_half(population, pop);

	// --- Produce next generation ---
	std::vector<Genome> next_gen;
	next_gen.reserve(m_cfg.population_size);

	for (int s_idx = 0; s_idx < static_cast<int>(m_species.size()); ++s_idx) {
		Species &sp = m_species[s_idx];
		if (sp.member_indices.empty())
			continue;

		int n_offspring = offspring_counts[s_idx];
		if (n_offspring <= 0)
			continue;

		// Elite: keep best member unchanged
		int elite_count = std::min(m_cfg.elites_per_species, static_cast<int>(sp.member_indices.size()));
		// Sort members by fitness (descending)
		std::sort(sp.member_indices.begin(), sp.member_indices.end(),
				[&pop](int a, int b) { return pop[a].fitness > pop[b].fitness; });

		for (int e = 0; e < elite_count && e < n_offspring; ++e) {
			Genome elite = pop[sp.member_indices[e]];
			elite.id = m_next_genome_id++;
			elite.species_id = sp.id;  // Preserve species assignment
			next_gen.push_back(std::move(elite));
		}

		// Offspring via crossover + mutation
		for (int k = elite_count; k < n_offspring; ++k) {
			Genome offspring;
			if (sp.member_indices.size() > 1 && rand_float() < m_cfg.crossover_rate) {
				const Genome &p1 = tournament_select(sp.member_indices, pop);
				const Genome &p2 = tournament_select(sp.member_indices, pop);
				const Genome &more_fit = (p1.fitness >= p2.fitness) ? p1 : p2;
				const Genome &less_fit = (p1.fitness >= p2.fitness) ? p2 : p1;
				offspring = Genome::crossover(m_next_genome_id++, more_fit, less_fit, m_rng);
			} else {
				offspring = tournament_select(sp.member_indices, pop);
				offspring.id = m_next_genome_id++;
			}
			offspring.species_id = sp.id;  // Assign to parent species
			offspring.mutate_all(m_cfg, innov, m_rng);
			next_gen.push_back(std::move(offspring));
		}
	}

	// If we're short (due to rounding), fill with mutations of random survivors
	while (static_cast<int>(next_gen.size()) < m_cfg.population_size) {
		if (pop.empty())
			break;
		int idx = rand_int(0, static_cast<int>(pop.size()) - 1);
		Genome g = pop[idx];
		g.id = m_next_genome_id++;
		g.mutate_all(m_cfg, innov, m_rng);
		next_gen.push_back(std::move(g));
	}

	// Trim if over-target
	if (static_cast<int>(next_gen.size()) > m_cfg.population_size)
		next_gen.resize(m_cfg.population_size);

	// Reset fitness for new generation (but preserve species_id for remapping)
	for (auto &g : next_gen) {
		g.fitness = 0.0f;
		g.adj_fitness = 0.0f;
		// NOTE: Do NOT reset species_id here - it's needed for remap_species_to_creatures()
	}

	fprintf(stderr, "[GEN-EVOLVE] gen=%d  best=%.3f  avg=%.3f  next_gen=%d  species=%d\n",
		m_generation, m_best_fitness_ever, m_last_avg_fitness,
		(int)next_gen.size(), (int)m_species.size());
	fflush(stderr);
	return next_gen;
}

// ============================================================
// speciate
// ============================================================
void Evolution::speciate(const std::vector<Genome> &pop) {
	// Clear member lists
	for (auto &sp : m_species) {
		sp.member_indices.clear();
		sp.member_genome_ids.clear();
	}

	for (int i = 0; i < static_cast<int>(pop.size()); ++i) {
		bool assigned = false;
		for (auto &sp : m_species) {
			float dist = pop[i].compatibility_distance(sp.representative, m_cfg);
			if (dist < m_cfg.species_compat_threshold) {
				sp.member_indices.push_back(i);
				sp.member_genome_ids.push_back(pop[i].id);
				const_cast<Genome &>(pop[i]).species_id = sp.id;
				assigned = true;
				break;
			}
		}
		if (!assigned) {
			Species new_sp;
			new_sp.id = m_next_species_id++;
			new_sp.name = create_species_name(m_generation);
			new_sp.representative = pop[i];
			new_sp.member_indices.push_back(i);
			new_sp.member_genome_ids.push_back(pop[i].id);
			const_cast<Genome &>(pop[i]).species_id = new_sp.id;
			m_species.push_back(std::move(new_sp));
		}
	}

	// Remove extinct species
	m_species.erase(std::remove_if(m_species.begin(), m_species.end(),
							[](const Species &sp) { return sp.member_indices.empty(); }),
			m_species.end());
}

// ============================================================
// compute_adjusted_fitness
// ============================================================
void Evolution::compute_adjusted_fitness(std::vector<Genome> &pop) {
	for (auto &sp : m_species) {
		float sum = 0.0f;
		float best = 0.0f;
		int n = static_cast<int>(sp.member_indices.size());
		for (int idx : sp.member_indices) {
			float af = (n > 0) ? pop[idx].fitness / static_cast<float>(n) : 0.0f;
			pop[idx].adj_fitness = af;
			sum += af;
			if (pop[idx].fitness > best)
				best = pop[idx].fitness;
		}
		sp.adj_fitness_sum = sum;
		sp.avg_fitness = (n > 0) ? sum : 0.0f;

		// Stagnation tracking
		if (best > sp.best_fitness) {
			sp.best_fitness = best;
			sp.stagnation_counter = 0;
		} else {
			++sp.stagnation_counter;
		}
		++sp.age;
	}
}

// ============================================================
// eliminate_stagnant_species
// ============================================================
void Evolution::eliminate_stagnant_species(const std::vector<Genome> &pop) {
	if (m_species.empty())
		return;

	// Find the best species ID before the lambda — accessing m_species[idx]
	// inside remove_if is UB because elements are moved during the algorithm.
	int best_sp_id = m_species[0].id;
	float best_sp_fit = -1.0f;
	for (const auto &sp : m_species) {
		if (sp.best_fitness > best_sp_fit) {
			best_sp_fit = sp.best_fitness;
			best_sp_id = sp.id;
		}
	}

	m_species.erase(std::remove_if(m_species.begin(), m_species.end(),
							[&](const Species &sp) {
								if (sp.id == best_sp_id)
									return false;
								return sp.stagnation_counter >= m_cfg.stagnation_limit;
							}),
			m_species.end());
}

// ============================================================
// cull_species_bottom_half
// ============================================================
void Evolution::cull_species_bottom_half(const std::vector<Genome> & /*pop_ref*/,
		std::vector<Genome> &pop) {
	for (auto &sp : m_species) {
		if (sp.member_indices.size() <= 2)
			continue;
		// Sort descending by fitness
		std::sort(sp.member_indices.begin(), sp.member_indices.end(),
				[&pop](int a, int b) { return pop[a].fitness > pop[b].fitness; });
		// Keep top half (at least 1)
		int keep = std::max(1, static_cast<int>(sp.member_indices.size()) / 2);
		sp.member_indices.resize(keep);
	}
}

// ============================================================
// compute_offspring_counts
// ============================================================
std::vector<int> Evolution::compute_offspring_counts(float total_adj_sum) const {
	int n_sp = static_cast<int>(m_species.size());
	std::vector<int> counts(n_sp, 0);
	if (total_adj_sum <= 0.0f || n_sp == 0) {
		// Distribute evenly
		for (int i = 0; i < n_sp; ++i)
			counts[i] = m_cfg.population_size / n_sp;
		counts[0] += m_cfg.population_size - (m_cfg.population_size / n_sp) * n_sp;
		return counts;
	}

	int total = 0;
	for (int i = 0; i < n_sp; ++i) {
		float ratio = m_species[i].adj_fitness_sum / total_adj_sum;
		counts[i] = static_cast<int>(std::floor(ratio * m_cfg.population_size));
		// Guarantee at least 1 offspring per species
		if (m_species[i].member_indices.size() > 0)
			counts[i] = std::max(1, counts[i]);
		total += counts[i];
	}
	// Distribute remainder to highest adj_fitness species
	int remainder = m_cfg.population_size - total;
	if (remainder > 0) {
		// Find species with highest fractional part
		std::vector<int> order(n_sp);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) {
			float fa = m_species[a].adj_fitness_sum / total_adj_sum;
			float fb = m_species[b].adj_fitness_sum / total_adj_sum;
			float ra = fa * m_cfg.population_size - std::floor(fa * m_cfg.population_size);
			float rb = fb * m_cfg.population_size - std::floor(fb * m_cfg.population_size);
			return ra > rb;
		});
		for (int k = 0; k < remainder && k < n_sp; ++k)
			++counts[order[k]];
	}
	return counts;
}

// ============================================================
// tournament_select
// ============================================================
const Genome &Evolution::tournament_select(const std::vector<int> &members,
		const std::vector<Genome> &pop) const {
	int k = std::min(3, static_cast<int>(members.size()));
	int best_local = members[rand_int(0, static_cast<int>(members.size()) - 1)];
	for (int i = 1; i < k; ++i) {
		int candidate = members[rand_int(0, static_cast<int>(members.size()) - 1)];
		if (pop[candidate].fitness > pop[best_local].fitness)
			best_local = candidate;
	}
	return pop[best_local];
}

// ============================================================
// update_representatives
// ============================================================
void Evolution::update_representatives(const std::vector<Genome> &pop) {
	for (auto &sp : m_species) {
		if (sp.member_indices.empty())
			continue;
		// Choose a random member as the new representative
		int idx = sp.member_indices[rand_int(0, static_cast<int>(sp.member_indices.size()) - 1)];
		sp.representative = pop[idx];
	}
}

// ============================================================
// create_species_name
// ============================================================
std::string Evolution::create_species_name(int generation) {
	int count = m_species_created_per_gen[generation]++;
	char letter = 'A' + (count % 26);  // A-Z cycles
	return "Gen " + std::to_string(generation) + "-" + letter;
}

// ============================================================
// get_species_info
// ============================================================
std::vector<SpeciesInfo> Evolution::get_species_info(const std::vector<Creature> &creatures) const {
	std::vector<SpeciesInfo> result;
	int total_alive_counted = 0;

	for (const auto &sp : m_species) {
		SpeciesInfo info;
		info.name = sp.name;
		info.member_count = static_cast<int>(sp.member_indices.size());
		info.alive_count = 0;
		info.best_fitness = sp.best_fitness;
		info.avg_fitness = sp.avg_fitness;

		for (int idx : sp.member_indices) {
			if (idx >= 0 && idx < static_cast<int>(creatures.size())) {
				if (creatures[idx].alive) {
					info.alive_count++;
					total_alive_counted++;
				}
			}
		}
		result.push_back(info);
	}

	// Debug: Log if counts don't match
	int total_alive = 0;
	for (const auto &c : creatures) {
		if (c.alive) total_alive++;
	}
	if (total_alive_counted != total_alive) {
		fprintf(stderr, "[SPECIES-COUNT-MISMATCH] reported=%d actual=%d species_count=%zu\n",
			total_alive_counted, total_alive, m_species.size());
		for (size_t i = 0; i < m_species.size(); ++i) {
			fprintf(stderr, "  [%zu] %s: member_count=%zu indices=[",
				i, m_species[i].name.c_str(), m_species[i].member_indices.size());
			for (int idx : m_species[i].member_indices) {
				fprintf(stderr, "%d ", idx);
			}
			fprintf(stderr, "]\n");
		}
	}

	return result;
}
