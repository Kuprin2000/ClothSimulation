#pragma once
#include <string>

static const int CONSTRAINT_TYPES_COUNT = 18;

enum class ConstraintType : uint8_t
{
	SEW_VERTICES,
	STRETCH,
	REALISTIC_STRETCH,
	BEND,
	REALISTIC_BEND,
	PHANTOM_VERTICES,
	FIXED_POSITION,
	FIXED_ANGLE,

	SELF_VERTEX_TRIANGLE_COLLISION,
	SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS,
	SELF_EDGE_EDGE_COLLISION,
	COLLIDER_VERTEX_TRIANGLE_COLLISION,
	COLLIDER_EDGE_EDGE_COLLISION,

	SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE,
	SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE,
	SELF_EDGE_EDGE_COLLISION_CANDIDATE,
	COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE,
	COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE
};

static const char *CONSTRAINT_TYPE_NAME[CONSTRAINT_TYPES_COUNT] =
	{
		"sew vertices",
		"stretch",
		"realistic stretch",
		"bend",
		"realistic bend",
		"phantom vertices",
		"fixed position",
		"fixed angle",

		"self vertex triangle collision",
		"self vertex triangle collision between layers",
		"self edge edge collision",
		"collider vertex triangle collision",
		"collider edge edge collision"

		"self vertex triangle collision candidate",
		"self vertex triangle collision between layers candidate",
		"self edge edge collision candidate",
		"collider vertex triangle collision candidate",
		"collider edge edge collision candidate"};

static constexpr int CONSTRAINT_DATA_COUNT[CONSTRAINT_TYPES_COUNT] =
	{
		// SEW_VERTICES: 1 - opposite stiffness, 1 - distance to join vertices, 1 - lambda for XPBD
		3,
		// STRETCH: 1 - recommended distance, 1 - opposite stiffness, 1 - lambda for XPBD
		3,
		// REALISTIC_STRETCH: 4 - to specify reference inverted shape matrix, 1 - reference triangle square, 1 - opposite stiffness, 1 - lambda for XPBD
		7,
		// BEND: 1 - initial angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
		3,
		// REALISTIC_BEND: 16 - Hessian energy matrix, 1 - opposite stiffness, 1 - lambda for XPBD
		18,
		// PHANTOM_VERTICES: 1 - number of vertices
		1,
		// FIXED_POSITION: 3 - position of a vertex
		3,
		// FIXED_ANGLE: 1 - optimal angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
		3,

		// SELF_VERTEX_TRIANGLE_COLLISION: 1 - optimal distance, 1 - flag if vertex is above the triangle
		2,
		// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - optimal distance
		1,
		// SELF_EDGE_EDGE_COLLISION: 1 - optimal distance, 2 - parameters of vertices of the edges
		3,
		// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - recommended distance,  1 - collider triangle id
		2,
		// COLLIDER_EDGE_EDGE_COLLISION: 1 - optimal distance, 1 - point parameter on cloth edge, 1 - point parameter on collider edge,
		// 1 - collider edge vertex a, 1 - collider edge vertex b
		5,

		// candidates have data arrays of the same size filled with zeros
		// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE
		2,
		// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE
		2,
		// SELF_EDGE_EDGE_COLLISION_CANDIDATE
		3,
		// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE
		2,
		// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE
		5};

static constexpr int CONSTRAINT_VERTICES_COUNT[CONSTRAINT_TYPES_COUNT] =
	{
		// SEW_VERTICES: 2 - we sew them
		2,
		// STRETCH: 2 - we control distance between them
		2,
		// REALISTIC_STRETCH: 3 - triangle controlled by the constraint
		3,
		// BEND: 4 - two triangles with common edge
		4,
		// REALISTIC_BEND: 4 - two triangles with common edge
		4,
		// PHANTOM_VERTICES: 25 - this is maximum vertices per phantom
		25,
		// FIXED_POSITION: 1 - one vertex
		1,
		// FIXED_ANGLE: 4 - two triangles with common edge
		4,

		// some arrays store zeros to sync arrays sizes with candidates
		// SELF_VERTEX_TRIANGLE_COLLISION: 1 - vertex, 3 - triangle
		4,
		// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - vertex, 3 - triangle
		4,
		// SELF_EDGE_EDGE_COLLISION: 2 - edge a, 2 - edge b
		4,
		// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - one vertex of the cloth
		1,
		// COLLIDER_EDGE_EDGE_COLLISION: 2 - one edge
		2,

		// candidates have vertices arrays of the same size but can store another values
		// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - vertex, 1 - triangle, 2 - zeros
		4,
		// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE: 1 - vertex, 1 - triangle, 2 - zeros
		4,
		// SELF_EDGE_EDGE_COLLISION_CANDIDATE: 1 - triangle a,  1 - edge a local index, 1 - triangle b, 1 - edge b local index
		4,
		// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - cloth vertex, 1 - collider triangle
		2,
		// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE: 1 - cloth triangle, 1 - cloth edge local index, 1 - collider triangle,
		// 1 - collider triangle local index
		4};

enum class ConstraintKeyGenerationFunction : uint8_t
{
	NONE,
	ONE_VERTEX,
	TWO_VERTICES,
	THREE_VERTICES
};

static constexpr ConstraintKeyGenerationFunction constraint_key_generation_function[CONSTRAINT_TYPES_COUNT] = {
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // SEW_VERTICES
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // STRETCH
	ConstraintKeyGenerationFunction::THREE_VERTICES, // REALISTIC_STRETCH
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // BEND
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // REALISTIC_BEND
	ConstraintKeyGenerationFunction::ONE_VERTEX,	 // PHANTOM_VERTICES
	ConstraintKeyGenerationFunction::ONE_VERTEX,	 // FIXED_POSITION
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // FIXED_ANGLE

	ConstraintKeyGenerationFunction::NONE, // SELF_VERTEX_TRIANGLE_COLLISION
	ConstraintKeyGenerationFunction::NONE, // SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS
	ConstraintKeyGenerationFunction::NONE, // SELF_EDGE_EDGE_COLLISION
	ConstraintKeyGenerationFunction::NONE, // COLLIDER_VERTEX_TRIANGLE_COLLISION
	ConstraintKeyGenerationFunction::NONE, // COLLIDER_EDGE_EDGE_COLLISION

	ConstraintKeyGenerationFunction::NONE, // SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	ConstraintKeyGenerationFunction::NONE, // SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE
	ConstraintKeyGenerationFunction::NONE, // SELF_EDGE_EDGE_COLLISION_CANDIDATE
	ConstraintKeyGenerationFunction::NONE, // COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	ConstraintKeyGenerationFunction::NONE  // COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE
};