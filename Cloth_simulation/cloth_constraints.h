#pragma once
#include <string>

static const int CONSTRAINT_TYPES_COUNT = 18;

enum class ConstraintType : uint8_t
{
	STRETCH,
	REALISTIC_STRETCH,
	BEND,
	REALISTIC_BEND,
	PHANTOM_VERTICES,

	SEW_VERTICES,
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

static constexpr int CONSTRAINT_UINT_COUNT[CONSTRAINT_TYPES_COUNT] =
{
	// STRETCH: 2 - we control distance between them
	2,
	// REALISTIC_STRETCH: 3 - triangle controlled by the constraint
	3,
	// BEND: 4 - two triangles with common edge
	4,
	// REALISTIC_BEND: 4 - two triangles with common edge
	4,
	// PHANTOM_VERTICES: 25 - this is maximum vertices per phantom
	31,

	// SEW_VERTICES: 2 - we sew them
	2,
	// FIXED_POSITION: 1 - one vertex
	1,
	// FIXED_ANGLE: 4 - two triangles with common edge
	4,

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

	// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - vertex, 1 - triangle
	2,
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE: 1 - vertex, 1 - triangle
	2,
	// SELF_EDGE_EDGE_COLLISION_CANDIDATE: 1 - triangle a,  1 - edge a local index, 1 - triangle b, 1 - edge b local index
	4,
	// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - cloth vertex, 1 - collider triangle
	2,
	// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE: 1 - cloth triangle, 1 - cloth edge local index, 1 - collider triangle,
	// 1 - collider triangle local index
	4
};

static constexpr int CONSTRAINT_FLOAT_COUNT[CONSTRAINT_TYPES_COUNT] =
{
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

	// SEW_VERTICES: 1 - opposite stiffness, 1 - distance to join vertices, 1 - lambda for XPBD
	3,
	// FIXED_POSITION: 3 - position of a vertex
	3,
	// FIXED_ANGLE: 1 - optimal angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
	3,

	// SELF_VERTEX_TRIANGLE_COLLISION: 1 - flag if vertex is above the triangle
	1,
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: no data
	0,
	// SELF_EDGE_EDGE_COLLISION: 2 - parameters of vertices of the edges
	2,
	// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - collider triangle id
	1,
	// COLLIDER_EDGE_EDGE_COLLISION: 1 - point parameter on cloth edge, 1 - point parameter on collider edge,
	// 1 - collider edge vertex a, 1 - collider edge vertex b
	4,

	// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	0,
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE
	0,
	// SELF_EDGE_EDGE_COLLISION_CANDIDATE
	0,
	// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	0,
	// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE
	0
};

static constexpr int CONSTRAINT_BYTES_COUNT[CONSTRAINT_TYPES_COUNT] =
{
	// STRETCH
	64,
	// REALISTIC_STRETCH
	64,
	// BEND
	64,
	// REALISTIC_BEND
	128,
	// PHANTOM_VERTICES
	128,

	// SEW_VERTICES
	32,
	// FIXED_POSITION
	32,
	// FIXED_ANGLE
	32,

	// SELF_VERTEX_TRIANGLE_COLLISION
	32,
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS
	32,
	// SELF_EDGE_EDGE_COLLISION
	32,
	// COLLIDER_VERTEX_TRIANGLE_COLLISION
	32,
	// COLLIDER_EDGE_EDGE_COLLISION
	32,

	// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	32,
	// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE
	32,
	// SELF_EDGE_EDGE_COLLISION_CANDIDATE
	32,
	// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE
	32,
	// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE
	32
};

enum class ConstraintKeyGenerationFunction : uint8_t
{
	NONE,
	ONE_VERTEX,
	TWO_VERTICES,
	THREE_VERTICES
};

static constexpr ConstraintKeyGenerationFunction constraint_key_generation_function[CONSTRAINT_TYPES_COUNT] = {
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // STRETCH
	ConstraintKeyGenerationFunction::THREE_VERTICES, // REALISTIC_STRETCH
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // BEND
	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // REALISTIC_BEND
	ConstraintKeyGenerationFunction::ONE_VERTEX,	 // PHANTOM_VERTICES

	ConstraintKeyGenerationFunction::TWO_VERTICES,	 // SEW_VERTICES
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

// internal constraints must have structures which size is multiple of 64 bytes

struct Stretch
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::STRETCH]];
	float m_recommended_distance;
	float m_opposite_stiffness;
	float m_lambda;
	uint32_t m_reserved[11];
};

static_assert(sizeof(Stretch) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::STRETCH]);

struct RealisticStretch
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH]];
	float m_reference_inverted_shape_mat[4];
	float m_reference_triangles_square;
	float m_opposite_stiffness;
	float m_lambda;
	uint32_t m_reserved[6];
};

static_assert(sizeof(RealisticStretch) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::REALISTIC_STRETCH]);

struct Bend
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::BEND]];
	float m_initial_angle;
	float m_opposite_stiffness;
	float m_lambda;
	uint32_t m_reserved[9];
};

static_assert(sizeof(Bend) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::BEND]);

struct RealisticBend
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::REALISTIC_BEND]];
	float m_hessian_energy_mat[16];
	float m_opposite_stiffness;
	float m_lambda;
	uint32_t m_reserved[10];
};

static_assert(sizeof(RealisticBend) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::REALISTIC_BEND]);

struct PhantomVertices
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES]];
	float m_vertices_count;
};

static_assert(sizeof(PhantomVertices) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::PHANTOM_VERTICES]);

// user constraints must have structures which size is multiple of 32 bytes

struct SewVertices
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SEW_VERTICES]];
	float m_opposite_stiffness;
	float m_critical_distance;
	float m_lambda;
	uint32_t m_reserved[3];
};

static_assert(sizeof(SewVertices) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SEW_VERTICES]);

struct FixedPosition
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::FIXED_POSITION]];
	float m_coords[3];
	float m_reserved[4];
};

static_assert(sizeof(FixedPosition) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::FIXED_POSITION]);

struct FixedAngle
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::FIXED_ANGLE]];
	float m_optimal_angle;
	float m_opposite_stiffness;
	float m_lambda;
	uint32_t m_reserved[1];
};

static_assert(sizeof(FixedAngle) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::FIXED_ANGLE]);

// collision constraints must have structures which size is multiple of 32 bytes

struct SelfVertexTriangleCollision
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION]];
	float m_is_above_triangle;
	uint32_t m_reserved[3];
};

static_assert(sizeof(SelfVertexTriangleCollision) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION]);

struct SelfVertexTriangleCollisionBetweenLayers
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS]];
	uint32_t m_reserved[4];
};

static_assert(sizeof(SelfVertexTriangleCollisionBetweenLayers) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS]);

struct SelfEdgeEdgeCollision
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]];
	float m_edge_a_param;
	float m_edge_b_param;
	uint32_t m_reserved[2];
};

static_assert(sizeof(SelfEdgeEdgeCollision) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]);

struct ColliderVertexTriangleCollision
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION]];
	float m_collider_triangle;
	uint32_t m_reserved[6];
};

static_assert(sizeof(ColliderVertexTriangleCollision) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION]);

struct ColliderEdgeEdgeCollision
{
	uint32_t m_uint_data[CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION]];
	float m_cloth_edge_param;
	float m_collider_edge_param;
	float m_collider_edge_vertex_a;
	float m_collider_edge_vertex_b;
	uint32_t m_reserved[2];
};

static_assert(sizeof(ColliderEdgeEdgeCollision) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION]);

struct SelfVertexTriangleCollisionCandidate
{
	uint32_t m_vertex;
	uint32_t m_triangle;
	uint32_t m_reserved[6];
};

static_assert(sizeof(SelfVertexTriangleCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE]);
static_assert(sizeof(SelfVertexTriangleCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION]);

struct SelfVertexTriangleCollisionBetweenLayersCandidate
{
	uint32_t m_vertex;
	uint32_t m_triangle;
	uint32_t m_reserved[6];
};

static_assert(sizeof(SelfVertexTriangleCollisionBetweenLayersCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE]);
static_assert(sizeof(SelfVertexTriangleCollisionBetweenLayersCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS]);

struct SelfEdgeEdgeCollisionCandidate
{
	uint32_t m_triangle_a;
	uint32_t m_triangle_a_local_index;
	uint32_t m_triangle_b;
	uint32_t m_triangle_b_local_index;
	uint32_t m_reserved[4];
};

static_assert(sizeof(SelfEdgeEdgeCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE]);
static_assert(sizeof(SelfEdgeEdgeCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::SELF_EDGE_EDGE_COLLISION]);

struct ColliderVertexTriangleCollisionCandidate
{
	uint32_t m_cloth_vertex;
	uint32_t m_collider_triangle;
	uint32_t m_reserved[6];
};

static_assert(sizeof(ColliderVertexTriangleCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE]);
static_assert(sizeof(ColliderVertexTriangleCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION]);

struct ColliderEdgeEdgeCollisionCandidate
{
	uint32_t m_cloth_triangle;
	uint32_t m_cloth_edge_local_index;
	uint32_t m_collider_triangle;
	uint32_t m_collider_triangle_local_index;
	uint32_t m_reserved[4];
};

static_assert(sizeof(ColliderEdgeEdgeCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE]);
static_assert(sizeof(ColliderEdgeEdgeCollisionCandidate) == CONSTRAINT_BYTES_COUNT[(uint32_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION]);