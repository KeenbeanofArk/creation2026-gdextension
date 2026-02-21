#pragma once
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <vector>

// ============================================================
// TerrainBinding  –  builds a visual terrain mesh from a heightmap
// ============================================================
class TerrainBinding : public godot::MeshInstance3D {
	GDCLASS(TerrainBinding, godot::MeshInstance3D)

public:
	TerrainBinding() = default;
	~TerrainBinding() override = default;

	// Called once (or any time terrain changes) to (re)build the mesh.
	// heights: row-major float grid of size*size normalised values in [0,1]
	// cell_scale: world units per grid cell
	// height_scale: world units for max height
	void build_from_heightmap(const std::vector<float> &heights,
			int size,
			float cell_scale,
			float height_scale);

protected:
	static void _bind_methods();

private:
	void build_mesh(const std::vector<float> &heights,
			int size, float cell_scale, float height_scale);

	godot::Ref<godot::ArrayMesh> m_mesh;
	godot::Ref<godot::StandardMaterial3D> m_material;
};
