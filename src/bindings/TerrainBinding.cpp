#include "TerrainBinding.h"
#include <algorithm>
#include <cmath>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>

using namespace godot;

void TerrainBinding::_bind_methods() {
	// No exported properties needed — built programmatically
}

void TerrainBinding::build_from_heightmap(const std::vector<float> &heights,
		int size,
		float cell_scale,
		float height_scale) {
	if (heights.empty() || size < 2)
		return;

	// Build material once
	if (m_material.is_null()) {
		m_material.instantiate();
		m_material->set_albedo(Color(0.24f, 0.55f, 0.22f)); // grass green
		m_material->set_roughness(0.9f);
		m_material->set_metallic(0.0f);
	}

	build_mesh(heights, size, cell_scale, height_scale);
	set_material_override(m_material);
}

void TerrainBinding::build_mesh(const std::vector<float> &heights,
		int size, float cell_scale, float height_scale) {
	Ref<SurfaceTool> st;
	st.instantiate();
	st->begin(Mesh::PRIMITIVE_TRIANGLES);

	float half = (size - 1) * cell_scale * 0.5f; // centre at origin

	auto get_h = [&](int x, int z) -> float {
		x = std::max(0, std::min(size - 1, x));
		z = std::max(0, std::min(size - 1, z));
		return heights[z * size + x] * height_scale;
	};

	auto get_pos = [&](int x, int z) -> Vector3 {
		return Vector3(x * cell_scale - half,
				get_h(x, z),
				z * cell_scale - half);
	};

	// Normal from finite differences
	auto get_normal = [&](int x, int z) -> Vector3 {
		float hL = get_h(x - 1, z);
		float hR = get_h(x + 1, z);
		float hD = get_h(x, z - 1);
		float hU = get_h(x, z + 1);
		Vector3 n(hL - hR, 2.0f * cell_scale, hD - hU);
		return n.normalized();
	};

	for (int z = 0; z < size - 1; ++z) {
		for (int x = 0; x < size - 1; ++x) {
			// Two triangles per quad: (x,z) (x+1,z) (x,z+1) and (x+1,z) (x+1,z+1) (x,z+1)
			auto emit = [&](int xi, int zi) {
				st->set_normal(get_normal(xi, zi));
				st->set_uv(Vector2(static_cast<float>(xi) / (size - 1),
						static_cast<float>(zi) / (size - 1)));
				st->add_vertex(get_pos(xi, zi));
			};

			// Triangle 1
			emit(x, z);
			emit(x + 1, z);
			emit(x, z + 1);
			// Triangle 2
			emit(x + 1, z);
			emit(x + 1, z + 1);
			emit(x, z + 1);
		}
	}

	m_mesh.instantiate();
	st->commit(m_mesh);
	set_mesh(m_mesh);
}
