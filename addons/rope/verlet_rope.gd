@tool
extends MeshInstance3D
class_name Rope3D

## A fast implementation of verlet integration based rope physics, similar to the one seen in Half-Life 2.
##
## Creating ropes in code:
## [codeblock]
### instance and add rope to scene
##var rope = load('<path to GDVerletRope.gd>').instance()
##add_child(rope)
##
### set its params
##rope.preprocess_iterations = 0
##rope.stiffness = 1.0
##rope.rope_width = 0.02
##rope.transform.origin = Vector3.ZERO
##rope.attach_end_to = end_node.get_path()
##rope.rope_length = 6.0
##rope.simulation_particles = 7
##var wind_noise = FastNoiseLite.new()
##wind_noise.fractal_octaves = 1
##wind_noise.frequency = 0.04
##wind_noise.fractal_gain = 1.0
##rope.wind_noise = wind_noise
##rope.wind = Vector3.RIGHT
##rope.wind_scale = 5.0
## [/codeblock]
## References: [br]
## [kbd]https://docs.unrealengine.com/4.26/en-US/Basics/Components/Rendering/CableComponent/[/kbd][br]
## [kbd]https://owlree.blog/posts/simulating-a-rope.html[/kbd][br]
## [kbd]https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html[/kbd][br]
## [kbd]https://toqoz.fyi/game-rope.html[/kbd][br]

func _enter_tree():
	if not get_mesh():
		set_mesh(ImmediateMesh.new())

class RopeParticleData:
	var pos_curr: PackedVector3Array
	var pos_prev: PackedVector3Array
	var accel: PackedVector3Array
	var is_attached: Array
	
	var tangents: PackedVector3Array
	var normals: PackedVector3Array
	var binormals: PackedVector3Array

	func is_empty() -> bool:
		return len(pos_curr) == 0

	func resize(idx: int) -> void:
		pos_curr.resize(idx)
		pos_prev.resize(idx)
		accel.resize(idx)
		is_attached.resize(idx)
		
		tangents.resize(idx)
		normals.resize(idx)
		binormals.resize(idx)

	func _init() -> void:
		resize(3)

# t = 0.0, 1.0
static func catmull_interpolate_in_step_one(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PackedVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)
	# order point, tangent ...
	return PackedVector3Array([
		p1, m1.normalized(), # t = 0.0
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.25, 0.5, 0.75, 1.0
static func catmull_interpolate_in_step_fourths(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PackedVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent ... 
	return PackedVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.84375 * p1 + 0.15625 * p2 + 0.140625 * m1 - 0.046875 * m2, (-1.125 * p1 + 1.125 * p2 + 0.1875 * m1 - 0.3125 * m2).normalized(), # t = 0.25
		0.5 * p1 + 0.5 * p2 + 0.125 * m1 - 0.125 * m2, (-1.5 * p1 + 1.5 * p2 - 0.25 * m1 - 0.25 * m2).normalized(), # t = 0.5
		0.15625 * p1 + 0.84375 * p2 + 0.046875 * m1 - 0.140625 * m2, (-1.125 * p1 + 1.125 * p2 - 0.3125 * m1 + 0.1875 * m2).normalized(), # t = 0.75
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.333, 0.6666, 1.0
static func catmull_interpolate_in_step_thirds(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PackedVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent ...
	return PackedVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.7407407407407407 * p1 + 0.25925925925925924 * p2 + 0.14814814814814814 * m1 - 0.07407407407407407 * m2, (-1.3333333333333335 * p1 + 1.3333333333333335 * p2 - 0.3333333333333333 * m2).normalized(), # t = 0.33 
		0.2592592592592593 * p1 + 0.7407407407407407 * p2 + 0.07407407407407407 * m1 - 0.14814814814814814 * m2, (-1.3333333333333335 * p1 + 1.3333333333333335 * p2 - 0.33333333333333326 * m1).normalized(), # t = 0.66
		p2, m2.normalized() # t = 1.0
	])

# t = 0.0, 0.5, 1.0
static func catmull_interpolate_in_step_halfs(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3) -> PackedVector3Array:
	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	# order point, tangent
	return PackedVector3Array([
		p1, m1.normalized(), # t = 0.0
		0.5 * p1 + 0.5 * p2 + 0.125 * m1 - 0.125 * m2, (-1.5 * p1 + 1.5 * p2 - 0.25 * m1 - 0.25 * m2).normalized(), # t = 0.5
		p2, m2.normalized() # t = 1.0
	])

# fast? catmull spline
static func catmull_interpolate(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, t: float) -> PackedVector3Array:
	var t_sqr: float = t * t
	var t_cube: float = t_sqr * t

	var m1: Vector3 = 0.5 * (p2 - p0)
	var m2: Vector3 = 0.5 * (p3 - p1)

	var a: Vector3 = 2.0 * (p1 - p2) + m1 + m2
	var b: Vector3 = -3.0 * (p1 - p2) - 2.0 * m1 - m2;
	var c: Vector3 = m1;
	var d: Vector3 = p1;
	# order point, tangent
	return PackedVector3Array([a * t_cube + b * t_sqr + c * t + d, (3.0 * a * t_sqr + 2.0 * b * t + c).normalized()])


# references: 
# https://docs.unrealengine.com/4.26/en-US/Basics/Components/Rendering/CableComponent/
# https://owlree.blog/posts/simulating-a-rope.html
# https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
# https://toqoz.fyi/game-rope.html

const INV_SQRT_2: float = 1.0 / sqrt(2.0)
const COS_5_DEG: float = cos(deg_to_rad(5))
const COS_10_DEG: float = cos(deg_to_rad(10))
const COS_15_DEG: float = cos(deg_to_rad(15))
const COS_20_DEG: float = cos(deg_to_rad(20))
const COS_25_DEG: float = cos(deg_to_rad(25))
const COS_30_DEG: float = cos(deg_to_rad(30))

## Attach/detach the start point.
@export var attach_start: bool = true:
	set = set_attach_start
		
func set_attach_start(value: bool) -> void:
	attach_start = value
	if particle_data:
		particle_data.is_attached[0] = value

## Attach end to any another [Node3D] by [NodePath].
@export_node_path var attach_end_to: NodePath:
	set = set_attach_end_to
		
func set_attach_end_to(val: NodePath) -> void:
	attach_end_to = val 
	if particle_data != null:
		particle_data.is_attached[-1] = is_attached_end()
 
func get_end_location() -> Vector3:
	return particle_data.pos_curr[-1]

func is_attached_end() -> bool:
	return not attach_end_to.is_empty()

func is_attached_start() -> bool:
	return attach_start

## Length of the rope in meters.
@export_range(0.0, 40.0, 0.001, "or_greater", "suffix:m") var rope_length: float = 5.0
## Width of the rope in meters.
@export_range(0.0, 4.0, 0.001, "or_greater", "suffix:m") var rope_width : float = 0.07

## Rope material.
@export var default_material : BaseMaterial3D = preload("res://addons/rope/rope_material.tres") 

@export_group("Simulation")
## Number of particles to simulate the rope. [br]
## Odd number (greater than 3) is recommended for ropes attached on both sides
## for a smoother rope at its lowest point.
@export_range(3, 200) var simulation_particles: int = 9:
	set(value):
		set_simulation_particles(value)
		
func set_simulation_particles(val: int) -> void:
	simulation_particles = val
	if particle_data:
		particle_data.resize(simulation_particles)
		_create_rope()

## Number of verlet constraint iterations per frame. [br]
## Higher value gives accurate rope simulation for lengthy and ropes with many simulation particles.
## Increase if you find the rope is sagging or stretching too much.
@export var iterations: int = 2 # low value = more sag, high value = less sag
## Number of iterations to be precalculated in _ready() to set the rope in a rest position.
## Value of 20-30 should be enough.
@export_range(0, 120) var preprocess_iterations: int = 5
## Rate of simulation. Lower the value if the rope is not going to move a lot or it is far away.
@export_range(10, 60) var simulation_rate: int = 60
## Should be named elasticity.
## It is a fraction that controls how much the verlet constraint corrects the rope.
## Value from 0.1 to 1.0 is recommended.
@export_range(0.01, 1.5) var stiffness = 0.9 # low value = elastic, high value = taut rope

## On/off the simulation.
@export var simulate: bool = true
## On/off the drawing. You will still see the rope because [method ImmediateMesh.clear_surfaces] wasnt called,
## but the rope isn't being drawn every frame.
@export var draw: bool = true
## Does Catmull–Rom spline smoothing (for required segments) at distances less than this.
@export var subdiv_lod_distance = 15.0 # switches to only drawing quads between particles at this point

@export_group("Gravity")
## On/off gravity.
@export var apply_gravity: bool = true
## Gravity vector.
@export var gravity: Vector3 = ProjectSettings.get_setting("physics/3d/default_gravity", 9.8) * \
	ProjectSettings.get_setting("physics/3d/default_gravity_vector", Vector3.DOWN)
## A factor to scale the gravity vector.
@export var gravity_scale: float = 1.0

@export_group("Wind")
## On/off wind.
@export var apply_wind: bool = false
## [Noise] resource for the wind. Noise frequency controls the turbulence (kinda).
## Save resource to disk and share across ropes for a global wind setting.
@export var wind_noise: Noise
## The wind vector. 
@export var wind: Vector3 = Vector3(1.0, 0.0, 0.0)
## A factor to scale the wind vector.
@export var wind_scale: float = 10.0

@export_group("Damping")
## On/off air drag/damping. Sometimes helps when rope bugs out to bring it back to rest.
@export var apply_damping: bool = true
## Amount of damping.
@export var damping_factor: float = 100.0

@export_group("Collisions")
## On/off collision with bodies. Collisions work best on smooth surfaces without sharp edges.
## Collisions are checked only when a body enters the ropes [AABB].
@export var apply_collision: bool = false
## The collision mask to be used for collisions.
@export_flags_3d_physics var collision_mask: int = 1

@onready var space_state = get_world_3d().direct_space_state

var time: float = 0.0
var particle_data: RopeParticleData
var collision_check_param: PhysicsShapeQueryParameters3D
var collision_check_box: BoxShape3D

func get_segment_length() -> float:
	return rope_length / (simulation_particles - 1)

# unused func, maybe useful in code?
func add_particle_at_end(adjust_length: bool) -> void:
	var _pos_prev: Vector3 = particle_data.pos_prev[-1] + Vector3.BACK * 0.01;
	var _pos_curr: Vector3 = particle_data.pos_curr[-1] + Vector3.BACK * 0.01;
	var _is_attached: bool = particle_data.is_attached[-1]
	var _accel: Vector3 = particle_data.accel[-1]
	
	var _tangent: Vector3 = particle_data.tangents[-1]
	var _normal: Vector3 = particle_data.normals[-1]
	var _binormal: Vector3 = particle_data.binormals[-1]
	
	particle_data.is_attached[-1] = false
	
	particle_data.pos_curr.append(_pos_curr)
	particle_data.pos_prev.append(_pos_prev)
	particle_data.accel.append(_accel)
	particle_data.is_attached.append(_is_attached)
	particle_data.tangents.append(_tangent)
	particle_data.normals.append(_normal)
	particle_data.binormals.append(_binormal)
	
	simulation_particles += 1
	if adjust_length:
		rope_length += get_segment_length()

# unused func draws simple lines between particles
func _draw_linear_curve():
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, default_material)
	for i in range(simulation_particles - 1):
		var curr_pos: Vector3 = particle_data.pos_curr[i] - global_transform.origin
		var curr_binorm: Vector3 = particle_data.binormals[i]
		
		var next_pos: Vector3 = particle_data.pos_curr[i + 1] - global_transform.origin
		var next_binorm: Vector3 = particle_data.binormals[i + 1]
		
		_draw_quad([
			curr_pos - curr_binorm * rope_width, 
			next_pos - next_binorm * rope_width, 
			next_pos + next_binorm * rope_width, 
			curr_pos + curr_binorm * rope_width], 
			-curr_binorm, particle_data.tangents[i], 
			0.0, 1.0,
			Color.BLACK)
	mesh.suface_end()

func _draw_interval(data: PackedVector3Array, camera_position: Vector3, step: float, tangent: Vector3) -> void:
	var t: float = 0.0
	for i in range(0, data.size() - 2, 2):
		var curr_pos: Vector3 = data[i] - global_transform.origin
		var curr_tangent: Vector3 = data[i + 1]
		var curr_normal: Vector3 = (data[i] - camera_position).normalized()
		var curr_binorm: Vector3 = curr_normal.cross(curr_tangent).normalized()

		var next_pos: Vector3 = data[i + 2] - global_transform.origin
		var next_tangent: Vector3 = data[i + 3]
		var next_normal: Vector3 = (data[i + 2] - camera_position).normalized()
		var next_binorm: Vector3 = next_normal.cross(next_tangent).normalized()
	
		_draw_quad([
			curr_pos - curr_binorm * rope_width, 
			next_pos - next_binorm * rope_width, 
			next_pos + next_binorm * rope_width, 
			curr_pos + curr_binorm * rope_width], 
			-curr_binorm, tangent, 
			t, t + step,
			Color.BLACK)
		t += step
	pass

func _draw_catmull_curve_baked() -> void:
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, default_material)
	
	# do drawing
	var camera = get_viewport().get_camera_3d()
	var camera_position: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	
	for i in range(0, simulation_particles - 1):
		var p0: Vector3 = particle_data.pos_curr[i] - particle_data.tangents[i] * get_segment_length() if i == 0 else particle_data.pos_curr[i - 1]
		var p1: Vector3 = particle_data.pos_curr[i]
		var p2: Vector3 = particle_data.pos_curr[i + 1]
		var p3: Vector3 = particle_data.pos_curr[i + 1] + particle_data.tangents[i + 1] * get_segment_length() if i == simulation_particles - 2 else particle_data.pos_curr[i + 2]
		
		var cam_dist_particle: Vector3 = camera_position - p1
		var interval_data: PackedVector3Array
		var rope_draw_subdivs: float = 1.0
		# dont subdivide if farther than subdiv_lod_distance units from camera
		if cam_dist_particle.length_squared() <= subdiv_lod_distance * subdiv_lod_distance:
			# subdivision based on dot product of segments
			var tangent_dots: float = particle_data.tangents[i].dot(particle_data.tangents[i + 1])
			if tangent_dots >= COS_5_DEG:
				# 0 subdivisions
				interval_data = catmull_interpolate_in_step_one(p0, p1, p2, p3)
				rope_draw_subdivs = 1.0
			elif tangent_dots >= COS_15_DEG:
				# 2 subdivisions
				interval_data = catmull_interpolate_in_step_halfs(p0, p1, p2, p3)
				rope_draw_subdivs = 0.5
			elif tangent_dots >= COS_30_DEG:
				# 3 subdivisions
				interval_data = catmull_interpolate_in_step_thirds(p0, p1, p2, p3)
				rope_draw_subdivs = 0.333333
			else:
				# 4 subdivisions
				interval_data = catmull_interpolate_in_step_fourths(p0, p1, p2, p3)
				rope_draw_subdivs = 0.25
		else:
			interval_data = catmull_interpolate_in_step_one(p0, p1, p2, p3)
		
		_draw_interval(interval_data, camera_position, rope_draw_subdivs, particle_data.tangents[i])
	mesh.surface_end()

# unused func use catmull_curve_baked instead, it is faster
func _draw_catmull_curve() -> void:
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, default_material)
	
	# do drawing
	var camera = get_viewport().get_camera_3d()
	var camera_position: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	
	for i in range(0, simulation_particles - 1):
		var p0: Vector3 = particle_data.pos_curr[i] - particle_data.tangents[i] * get_segment_length() if i == 0 else particle_data.pos_curr[i - 1]
		var p1: Vector3 = particle_data.pos_curr[i]
		var p2: Vector3 = particle_data.pos_curr[i + 1]
		var p3: Vector3 = particle_data.pos_curr[i + 1] + particle_data.tangents[i + 1] * get_segment_length() if i == simulation_particles - 2 else particle_data.pos_curr[i + 2]
		
		var rope_draw_subdivs: float = 1.0
		var cam_dist_particle: Vector3 = camera_position - p1
		# dont subdivide if farther than subdiv_lod_distance units from camera
		if cam_dist_particle.length_squared() <= subdiv_lod_distance * subdiv_lod_distance:
			var tangent_dots: float = particle_data.tangents[i].dot(particle_data.tangents[i + 1])
			if tangent_dots >= COS_5_DEG:
				rope_draw_subdivs = 1.0 # 0 subdivisions
			elif tangent_dots >= COS_15_DEG:
				rope_draw_subdivs = 0.5 # 2 subdivisions
			elif tangent_dots >= COS_30_DEG:
				rope_draw_subdivs = 0.33333 # 3 subdivisions
			else:
				rope_draw_subdivs = 0.25 # 4 subdivisions
		
		var t = 0.0
		var step = rope_draw_subdivs
		while t <= 1.0:
			var point1_data: PackedVector3Array = catmull_interpolate(p0, p1, p2, p3, t)
			var point2_data: PackedVector3Array = catmull_interpolate(p0, p1, p2, p3, min(t + step, 1.0))
			
			var curr_pos: Vector3 = point1_data[0] - global_transform.origin
			var curr_tangent: Vector3 = point1_data[1]
			var curr_normal: Vector3 = (point1_data[0] - camera_position).normalized()
			var curr_binorm: Vector3 = curr_normal.cross(curr_tangent).normalized()

			var next_pos: Vector3 = point2_data[0] - global_transform.origin
			var next_tangent: Vector3 = point2_data[1]
			var next_normal: Vector3 = (point2_data[0] - camera_position).normalized()
			var next_binorm: Vector3 = next_normal.cross(next_tangent).normalized()

			_draw_quad([
				curr_pos - curr_binorm * rope_width, 
				next_pos - next_binorm * rope_width, 
				next_pos + next_binorm * rope_width, 
				curr_pos + curr_binorm * rope_width], 
				-curr_binorm, particle_data.tangents[i], 
				t, t + step,
				Color.BLACK)
			t += step
	mesh.surface_end()

func _calculate_rope_orientation_with_camera() -> void:
	var camera: Camera3D = get_viewport().get_camera_3d()
	var camera_pos: Vector3 = camera.global_transform.origin if camera else Vector3.ZERO
	particle_data.tangents[0] = (particle_data.pos_curr[1] - particle_data.pos_curr[0]).normalized()
	particle_data.normals[0] = (particle_data.pos_curr[0] - camera_pos).normalized()
	particle_data.binormals[0] = particle_data.normals[0].cross(particle_data.tangents[0]).normalized()

	particle_data.tangents[-1] = (particle_data.pos_curr[-1] - particle_data.pos_curr[-2]).normalized()
	particle_data.normals[-1] = (particle_data.pos_curr[-1] - camera_pos).normalized()
	particle_data.binormals[-1] = particle_data.normals[-1].cross(particle_data.tangents[-1]).normalized()

	for i in range(1, simulation_particles - 1):
		particle_data.tangents[i] = (particle_data.pos_curr[i + 1] - particle_data.pos_curr[i - 1]).normalized()
		particle_data.normals[i] = (particle_data.pos_curr[i] - camera_pos).normalized()
		particle_data.binormals[i] = particle_data.normals[i].cross(particle_data.tangents[i]).normalized()

func _create_rope() -> void:
	var end_location: Vector3
	if not attach_end_to.is_empty():
		end_location = get_node(attach_end_to).global_transform.origin
	else:
		end_location = global_transform.origin + Vector3.DOWN * rope_length
	
	var direction: Vector3 = (end_location - global_transform.origin).normalized()
	var gap = get_segment_length()
	
	if particle_data == null:
		particle_data = RopeParticleData.new()
	
	particle_data.resize(simulation_particles)
	# place particle_data initially in a line from start to end
	for i in range(simulation_particles):
		particle_data.pos_prev[i] = global_transform.origin + direction * gap * i
		particle_data.pos_curr[i] = particle_data.pos_prev[i]
		particle_data.accel[i] = gravity * gravity_scale
		particle_data.is_attached[i] = false
	
	particle_data.is_attached[0] = attach_start
	particle_data.is_attached[simulation_particles - 1] = not attach_end_to.is_empty()
	particle_data.pos_prev[simulation_particles - 1] = end_location
	particle_data.pos_curr[simulation_particles - 1] = end_location
	
	for _iter in range(preprocess_iterations):
		_verlet_process(1.0 / 60.0)
		# commenting this shaves quite a bit more startup time
		_apply_constraints()
	
	_calculate_rope_orientation_with_camera()

func _destroy_rope() -> void:
	particle_data.resize(0)
	simulation_particles = 0

func _draw_rope_particles() -> void:
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES, default_material)
	for i in range(simulation_particles):
		var pos_curr: Vector3 = particle_data.pos_curr[i] - global_transform.origin
		var tangent: Vector3 = particle_data.tangents[i]
		var normal: Vector3 = particle_data.normals[i]
		var binormal: Vector3 = particle_data.binormals[i]
		
		#material_override.set("vertex_color_use_as_albedo", true)
		#surface_set_color(Color.red)
		mesh.surface_add_vertex(pos_curr)
		mesh.surface_add_vertex(pos_curr + 0.3 * tangent)
		
		#surface_set_color(Color.green)
		mesh.surface_add_vertex(pos_curr)
		mesh.surface_add_vertex(pos_curr + 0.3 * normal)
		
		#surface_set_color(Color.blue)
		mesh.surface_add_vertex(pos_curr)
		mesh.surface_add_vertex(pos_curr + 0.3 * binormal)
		#material_override.set("vertex_color_use_as_albedo", false)
	mesh.surface_end()

# give in clockwise order, or maybe anticlockwise?
func _draw_quad(vs: Array, n: Vector3, _t: Vector3, uvx0: float, uvx1: float, c: Color) -> void:
	mesh.surface_set_color(c)
	mesh.surface_set_normal(n)
	# for normal mapping???
	# set_tangent(Plane(-t, 0.0))
	mesh.surface_set_uv(Vector2(uvx0, 0.0))
	mesh.surface_add_vertex(vs[0])
	mesh.surface_set_uv(Vector2(uvx1, 0.0))
	mesh.surface_add_vertex(vs[1])
	mesh.surface_set_uv(Vector2(uvx1, 1.0))
	mesh.surface_add_vertex(vs[2])
	mesh.surface_set_uv(Vector2(uvx0, 0.0))
	mesh.surface_add_vertex(vs[0])
	mesh.surface_set_uv(Vector2(uvx1, 1.0))
	mesh.surface_add_vertex(vs[2])
	mesh.surface_set_uv(Vector2(uvx0, 1.0))
	mesh.surface_add_vertex(vs[3])

func _apply_forces() -> void:
	for i in range(simulation_particles):
		var total_accel: Vector3 = Vector3.ZERO
		if apply_gravity:
			total_accel += gravity * gravity_scale
		if apply_wind and wind_noise != null:
			var pos: Vector3 = particle_data.pos_curr[i]
			var wind_force: float = wind_noise.get_noise_4d(pos.x, pos.y, pos.z, time)
			total_accel += wind_scale * wind * wind_force
		if apply_damping:
			var velocity: Vector3 = particle_data.pos_curr[i] - particle_data.pos_prev[i]
			var drag = -damping_factor * velocity.length() * velocity
			total_accel += drag
		
		particle_data.accel[i] = total_accel

func _verlet_process(delta: float) -> void:
	for i in range(simulation_particles):
		if not particle_data.is_attached[i]:
			var pos_curr: Vector3 = particle_data.pos_curr[i]
			var pos_prev: Vector3 = particle_data.pos_prev[i]
			particle_data.pos_curr[i] = 2.0 * pos_curr - pos_prev + delta * delta * particle_data.accel[i]
			particle_data.pos_prev[i] = pos_curr

var prev_normal: Vector3 # needed for _apply_constraints
func _apply_constraints() -> void:
	for _n in range(iterations):
		for i in range(simulation_particles - 1):
			var r: Vector3 = particle_data.pos_curr[i + 1] - particle_data.pos_curr[i]
			var d: float = r.length() - get_segment_length()
			r = r.normalized()
			if particle_data.is_attached[i]:
				particle_data.pos_curr[i + 1] -= r * d * stiffness
			elif particle_data.is_attached[i + 1]:
				particle_data.pos_curr[i] += r * d * stiffness
			else:
				particle_data.pos_curr[i] += r * d * 0.5 * stiffness
				particle_data.pos_curr[i + 1] -= r * d * 0.5 * stiffness
	
	if apply_collision:
		# check if any objects are in its aabb before doing collisions
		var aabb: AABB = get_aabb()
		collision_check_box.extents = aabb.size * 0.5
		collision_check_param.transform.origin = global_transform.origin + aabb.position + aabb.size * 0.5
		
		var colliders: Array = space_state.intersect_shape(collision_check_param, 1)
		if len(colliders) >= 1:
			for i in range(simulation_particles - 1):
				var partical_params = PhysicsRayQueryParameters3D.new()
				partical_params.from = particle_data.pos_curr[i] + prev_normal * 0.4
				partical_params.to = particle_data.pos_curr[i + 1]
				partical_params.exclude = []
				partical_params.collision_mask = collision_mask
				var result: Dictionary = space_state.intersect_ray(partical_params)
				if result:
					prev_normal = result.normal
					var ydiff: Vector3 = result.position - particle_data.pos_curr[i + 1]
					ydiff = ydiff.project(result.normal)
					#ydiff += rope_width * 0.5 * result.normal
					particle_data.pos_curr[i + 1] += ydiff
					particle_data.pos_prev[i + 1] = particle_data.pos_curr[i + 1]


func _ready() -> void:
	# Configure collision box
	var aabb: AABB = get_aabb()
	collision_check_box = BoxShape3D.new()
	collision_check_box.extents = aabb.size * 0.5
	
	# configure collision check param
	collision_check_param = PhysicsShapeQueryParameters3D.new()
	collision_check_param.collision_mask = collision_mask
	collision_check_param.margin = 0.1
	collision_check_param.shape_rid = collision_check_box.get_rid()
	collision_check_param.transform.origin = aabb.position + collision_check_box.extents
	
	_create_rope()

func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint() and (particle_data == null or particle_data.is_empty()):
		_create_rope()
	
	time += delta
	
	if Engine.get_physics_frames() % int(Engine.physics_ticks_per_second / simulation_rate) != 0:
		return
		
	# make the end follow the end attached object or stay at its attached location
	if not attach_end_to.is_empty():
		particle_data.pos_curr[-1] = get_node(attach_end_to).global_transform.origin
	if attach_start:
		particle_data.pos_curr[0] = global_transform.origin

	if simulate:
		_apply_forces()
		_verlet_process(delta * float(Engine.physics_ticks_per_second / simulation_rate))
		_apply_constraints()
	
	# drawing
	if draw:
		_calculate_rope_orientation_with_camera()
		# @HACK: rope doesnt draw from origin to attach_end_to correctly if rotated
		# calling to_local() in the drawing code is too slow
		global_transform.basis = Basis.IDENTITY
		mesh.clear_surfaces()
		_draw_catmull_curve_baked()
		#_draw_catmull_curve()
		#_draw_linear_curve()
		#_draw_rope_particles()

func _on_VisibilityNotifier_camera_exited(_camera: Camera3D) -> void:
	#simulate = false
	draw = false

func _on_VisibilityNotifier_camera_entered(_camera: Camera3D) -> void:
	#simulate = true
	draw = true
