# MIT License
@tool
extends VerletRope
class_name VerletRopeRigidbody

## Experimental variant of VerletRope with per-particle dynamic body pushing.
## Keeps the original rope setup plug-and-play while improving interactions with moving rigid bodies.

@export_group("Rigidbody Collision")
## Additional push distance applied after a hit to avoid sticking inside dynamic colliders.
@export_range(0.0, 0.2) var rigidbody_collision_push: float = 0.02
## Radius used for per-particle overlap checks against dynamic colliders.
@export_range(0.005, 0.2) var rigidbody_probe_radius: float = 0.03
## Per-step depenetration iterations for stability.
@export_range(1, 8) var rigidbody_depenetration_iterations: int = 2

var _rb_query_shape: SphereShape3D
var _rb_query_params: PhysicsShapeQueryParameters3D

func _ready() -> void:
	super._ready()
	_rb_query_shape = SphereShape3D.new()
	_rb_query_shape.radius = rigidbody_probe_radius
	_rb_query_params = PhysicsShapeQueryParameters3D.new()
	_rb_query_params.shape_rid = _rb_query_shape.get_rid()
	_rb_query_params.collision_mask = dynamic_collision_mask
	_rb_query_params.margin = 0.01

func _physics_process(delta: float) -> void:
	if _rb_query_shape != null and _rb_query_shape.radius != rigidbody_probe_radius:
		_rb_query_shape.radius = rigidbody_probe_radius
	if _rb_query_params != null:
		_rb_query_params.collision_mask = dynamic_collision_mask
	super._physics_process(delta)

func collide_rope(dynamic_collisions: Array) -> void:
	super.collide_rope(dynamic_collisions)
	if rope_collision_behavior == RopeCollisionBehavior.NONE:
		return
	if dynamic_collision_mask == 0:
		return
	if _space_state == null or _rb_query_params == null:
		return

	for _iter in range(rigidbody_depenetration_iterations):
		for i in range(1, simulation_particles):
			if _particle_data == null or i >= _particle_data.particles.size():
				continue
			var particle = _particle_data.particles[i]
			_rb_query_params.transform = Transform3D(Basis.IDENTITY, particle.position_current)
			var hits = _space_state.intersect_shape(_rb_query_params, max_dynamic_collisions)
			if hits.is_empty():
				continue

			var correction := Vector3.ZERO
			for hit in hits:
				if not hit.has("collider"):
					continue
				var collider = hit["collider"]
				if collider == null or not (collider is Node3D):
					continue
				var from_collider = particle.position_current - collider.global_position
				var normal = from_collider.normalized() if from_collider != Vector3.ZERO else Vector3.UP
				correction += normal

			if correction == Vector3.ZERO:
				continue

			particle.position_current += correction.normalized() * rigidbody_collision_push
			_particle_data.particles[i] = particle
