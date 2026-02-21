extends Control

@onready var stats_label: Label = $Panel/StatsVBox/StatsLabel
@onready var alive_label: Label = $Panel/StatsVBox/AliveLabel
@onready var help_label: Label = $HelpLabel
@onready var species_panel: PanelContainer = $SpeciesPanel
@onready var species_vbox: VBoxContainer = $SpeciesPanel/VBox/SpeciesScrollContainer/SpeciesVBox
@onready var detail_panel: PanelContainer = $SpeciesPanel/VBox/DetailPanel
@onready var detail_label: Label = $SpeciesPanel/VBox/DetailPanel/DetailScroll/DetailLabel

var _world_binding: WorldBinding = null
var _selected_species_index: int = -1
var _species_buttons: Array = []
var _should_update_species_panel := true

func _ready() -> void:
	help_label.text = "Right-click + drag: rotate  |  WASDQE: move  |  Space: pause/resume  |  R: restart  |  Click creature for Info"
	species_panel.visible = true
	detail_panel.visible = true

func _process(_delta: float) -> void:
	if _world_binding == null:
		_world_binding = _find_world()
	if _world_binding == null:
		return

	# Update species list on first load
	if _should_update_species_panel:
		update_species_panel()
		_should_update_species_panel = false

	alive_label.text = "Alive: %d   Step: %d" % [
		_world_binding.get_alive_count(),
		_world_binding.get_step_count()
	]
	stats_label.text = (
		"Generation: %d\nBest Fitness: %.2f\nAvg Fitness: %.2f\nSpecies: %d"
		% [
			_world_binding.get_generation(),
			_world_binding.get_best_fitness(),
			_world_binding.get_avg_fitness(),
			_world_binding.get_species_count()
		]
	)

func update_species_panel() -> void:
	if not _world_binding:
		return

	# Clear existing buttons
	for btn in _species_buttons:
		btn.queue_free()
	_species_buttons.clear()
	_selected_species_index = -1
	detail_panel.visible = false

	var species_info = _world_binding.get_species_details()
	var lines = species_info.split("\n")

	# Create clickable button for each species
	var button_index = 0
	for line in lines:
		if line.is_empty():
			continue

		var btn = Button.new()
		btn.text = line
		btn.alignment = HORIZONTAL_ALIGNMENT_LEFT
		btn.clip_text = true
		btn.pressed.connect(_on_species_clicked.bind(button_index))

		species_vbox.add_child(btn)
		_species_buttons.append(btn)
		button_index += 1

func _on_species_clicked(species_index: int) -> void:
	_selected_species_index = species_index

	var detail_text = _world_binding.get_species_detailed_info(species_index)
	detail_label.text = detail_text

	# Show detail panel
	detail_panel.visible = true

func update_stats(gen: int, best: float, avg: float, species: int) -> void:
	stats_label.text = (
		"Generation: %d\nBest Fitness: %.2f\nAvg Fitness: %.2f\nSpecies: %d"
		% [gen, best, avg, species]
	)

func on_generation_complete() -> void:
	_should_update_species_panel = true

# Called from main.gd when a creature is clicked in the 3D world
func show_creature_detail(detail_text: String) -> void:
	if not detail_text.is_empty():
		detail_label.text = detail_text
		detail_panel.visible = true

func _find_world() -> WorldBinding:
	var root = get_tree().get_root()
	for child in root.get_children():
		var wb = _search_for_world(child)
		if wb:
			return wb
	return null

func _search_for_world(node: Node) -> WorldBinding:
	if node is WorldBinding:
		return node
	for child in node.get_children():
		var result = _search_for_world(child)
		if result:
			return result
	return null
