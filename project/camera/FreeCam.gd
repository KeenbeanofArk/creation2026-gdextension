extends Camera3D

const SPEED := 20.0
const SENSITIVITY := 0.003

var _captured := false

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_RIGHT:
		_captured = event.pressed
		Input.set_mouse_mode(
			Input.MOUSE_MODE_CAPTURED if _captured else Input.MOUSE_MODE_VISIBLE
		)

	if _captured and event is InputEventMouseMotion:
		rotate_y(-event.relative.x * SENSITIVITY)
		rotate_object_local(Vector3.RIGHT, -event.relative.y * SENSITIVITY)

func _process(delta: float) -> void:
	if not _captured:
		return
	var dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_W): dir -= basis.z
	if Input.is_key_pressed(KEY_S): dir += basis.z
	if Input.is_key_pressed(KEY_A): dir -= basis.x
	if Input.is_key_pressed(KEY_D): dir += basis.x
	if Input.is_key_pressed(KEY_Q): dir -= basis.y
	if Input.is_key_pressed(KEY_E): dir += basis.y
	if not dir.is_zero_approx():
		position += dir.normalized() * SPEED * delta
