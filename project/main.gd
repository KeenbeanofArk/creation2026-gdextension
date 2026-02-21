extends Node3D

@onready var world_binding: WorldBinding = $WorldBinding
@onready var sim_ui = $CanvasLayer/SimUI

var _camera: Camera3D = null
var _selected_creature_index: int = -1

func _ready() -> void:
	_camera = $Camera3D
	world_binding.generation_complete.connect(_on_generation_complete)

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_R:
			world_binding.restart()
			sim_ui.update_stats(0, 0.0, 0.0, 0)
		elif event.keycode == KEY_SPACE:
			world_binding.toggle_pause()

	# NEW: Handle creature selection via LEFT mouse click
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		_select_creature_from_click(event.position)

func _on_generation_complete(gen: int, best: float, avg: float, species: int) -> void:
	sim_ui.update_stats(gen, best, avg, species)
	sim_ui.on_generation_complete()

func _select_creature_from_click(mouse_position: Vector2) -> void:
	if not _camera or not world_binding:
		return

	# Ray-cast from camera through mouse position
	var ray_origin = _camera.project_ray_origin(mouse_position)
	var ray_normal = _camera.project_ray_normal(mouse_position)

	# Find closest creature mesh intersecting with ray
	var closest_creature_idx = -1
	var closest_distance = INF

	# Check each creature's segment meshes
	# Loop through creatures until we get a null binding
	var creature_idx = 0
	while true:
		var creature_binding = world_binding.get_creature_visual(creature_idx)
		if not creature_binding:
			break

		# Check each segment mesh
		for mesh_instance in creature_binding.get_children():
			if mesh_instance is MeshInstance3D:
				var hit_distance = _ray_mesh_distance(ray_origin, ray_normal, mesh_instance)
				if hit_distance >= 0 and hit_distance < closest_distance:
					closest_distance = hit_distance
					closest_creature_idx = creature_idx

		creature_idx += 1

	# Update selection
	if closest_creature_idx >= 0:
		_selected_creature_index = closest_creature_idx
		var detail_text = world_binding.get_creature_detail(closest_creature_idx)
		sim_ui.show_creature_detail(detail_text)

func _ray_mesh_distance(ray_origin: Vector3, ray_normal: Vector3, mesh_instance: MeshInstance3D) -> float:
	# Ray-sphere intersection using mesh AABB
	var mesh = mesh_instance.get_mesh()
	if not mesh:
		return -1.0

	var aabb = mesh.get_aabb()
	var global_xform = mesh_instance.get_global_transform()

	# World-space sphere parameters
	var sphere_center = global_xform * aabb.get_center()
	var radius = aabb.get_longest_axis_size() / 2.0

	# Ray: P(t) = ray_origin + t * ray_normal
	# Sphere: |P - C|² = r²
	# Solve: |ray_origin + t*ray_normal - sphere_center|² = r²

	var oc = ray_origin - sphere_center
	var b = ray_normal.dot(oc)
	var c = oc.dot(oc) - radius * radius
	var discriminant = b * b - c

	if discriminant < 0:
		return -1.0  # No intersection

	var sqrt_disc = sqrt(discriminant)
	var t1 = -b - sqrt_disc
	var t2 = -b + sqrt_disc

	# Return closest positive intersection
	if t1 >= 0:
		return t1
	elif t2 >= 0:
		return t2
	else:
		return -1.0
