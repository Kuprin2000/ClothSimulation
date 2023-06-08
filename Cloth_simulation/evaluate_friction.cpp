// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "evaluate_friction.h"
#include "defines.h"

namespace EvaluateFrictionMultithreaded
{
	void evaluateSelfFriction(Cloth& cloth, const ConstraintsBuffers& buffers, int constraint_id, float time_delta)
	{
		// here we use friction model from
		// "Robust Treatment of Collisions, Contact and Friction for Cloth Animation"

		const SelfVertexTriangleCollision* constraint = (SelfVertexTriangleCollision*)buffers.getDataBlock(constraint_id);

		// SELF_VERTEX_TRIANGLE_COLLISION or SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - vertex, 3 - triangle
		const AlignedVector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
		constexpr int vertices_count = CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION];

		// here we can deal only with real vertices
		uint32_t constraint_vertices[vertices_count] = { 0u };
		glm::vec3 vertices_test_coords[vertices_count] = { glm::vec3(0.0f, 0.0f, 0.0f) };
		const glm::vec3* vertices_coords[vertices_count] = { nullptr };
		for (int i = 0; i < vertices_count; ++i)
		{
			constraint_vertices[i] = host_vertices[constraint->m_uint_data[i]];
			vertices_test_coords[i] = cloth.getTestCoords(constraint_vertices[i]);
			vertices_coords[i] = &cloth.getCoords(constraint_vertices[i]);
		}

		const glm::vec3& old_vertex_speed_vector = cloth.getSpeed(constraint_vertices[0]);
		const glm::vec3 new_vertex_speed_vector = (vertices_test_coords[0] - *vertices_coords[0]) / time_delta;

		const glm::vec3 triangle_old_normal = MathUtils::triangleNormal(*vertices_coords[1], *vertices_coords[2], *vertices_coords[3]);
		const glm::vec3 triangle_new_normal = MathUtils::triangleNormal(vertices_test_coords[1], vertices_test_coords[2], vertices_test_coords[3]);

		const float old_normal_speed = glm::dot(old_vertex_speed_vector, triangle_old_normal);
		const float new_normal_speed = glm::dot(new_vertex_speed_vector, triangle_new_normal);
		const float normal_speed_delta = new_normal_speed - old_normal_speed;

		if (normal_speed_delta > 0.0f)
		{
			return;
		}

		const glm::vec3 new_normal_speed_vector = new_normal_speed * triangle_new_normal;
		glm::vec3 new_tangent_speed_vector = new_vertex_speed_vector - new_normal_speed_vector;
		const float new_tangent_speed = glm::length(new_tangent_speed_vector);

		if (new_tangent_speed < FLT_EPSILON)
		{
			return;
		}

		const float friction_coeff = (cloth.getPartFrictionCoeff(cloth.getVertexPartId(constraint_vertices[0])) +
			cloth.getPartFrictionCoeff(cloth.getVertexPartId(constraint_vertices[1]))) *
			0.5f;

		const float correction_coeff = std::max(0.0f, 1.0f + friction_coeff * normal_speed_delta / new_tangent_speed);
		const glm::vec3 modified_new_vertex_speed_vector = new_normal_speed_vector + correction_coeff * new_tangent_speed_vector;

#ifdef CHECK_DELTAS
		glm::vec3 delta = (new_vertex_speed_vector - old_vertex_speed_vector) * time_delta;
		float delta_length = glm::length(delta[0]);
		if (delta_length > MAX_LENGTH)
		{
			const float total_length = delta_length;
		}
#endif

		cloth.getTestCoords(constraint_vertices[0]) += (modified_new_vertex_speed_vector - new_vertex_speed_vector) * time_delta;
	}

	void evaluateColliderFriction(Cloth& cloth, const Collider& collider, const ConstraintsBuffers& buffers, int constraint_id, float time_delta)
	{
		// here we use friction model from
		// "Robust Treatment of Collisions, Contact and Friction for Cloth Animation"

		const ColliderVertexTriangleCollision* constraint = (ColliderVertexTriangleCollision*)buffers.getDataBlock(constraint_id);

		// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - one vertex of the cloth, 1 - zeros
		const AlignedVector<uint32_t>& host_vertices = cloth.getHostOrOriginalVertices();
		constexpr int vertices_count = CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION];

		// here we can deal only with real vertices
		uint32_t constraint_vertices[vertices_count] = { 0u };
		glm::vec3 vertices_test_coords[vertices_count] = { glm::vec3(0.0f, 0.0f, 0.0f) };
		const glm::vec3* vertices_coords[vertices_count] = { nullptr };

		constraint_vertices[0] = host_vertices[constraint->m_uint_data[0]];
		vertices_test_coords[0] = cloth.getTestCoords(constraint_vertices[0]);
		vertices_coords[0] = &cloth.getCoords(constraint_vertices[0]);

		// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - recommended distance, 1 - collider triangle id

		const std::array<const glm::vec3*, 3> collider_triangle = collider.getTriangleCoords((int)roundf(constraint->m_collider_triangle));
		const glm::vec3 triangle_normal = MathUtils::triangleNormal(*collider_triangle[0], *collider_triangle[1], *collider_triangle[2]);

		const glm::vec3& old_vertex_speed_vector = cloth.getSpeed(constraint_vertices[0]);
		const glm::vec3 new_vertex_speed_vector = (vertices_test_coords[0] - *vertices_coords[0]) / time_delta;

		const float old_normal_speed = glm::dot(old_vertex_speed_vector, triangle_normal);
		const float new_normal_speed = glm::dot(new_vertex_speed_vector, triangle_normal);
		const float normal_speed_delta = new_normal_speed - old_normal_speed;

		if (normal_speed_delta > 0.0f)
		{
			return;
		}

		const glm::vec3 new_normal_speed_vector = new_normal_speed * triangle_normal;
		glm::vec3 new_tangent_speed_vector = new_vertex_speed_vector - new_normal_speed_vector;
		const float new_tangent_speed = glm::length(new_tangent_speed_vector);

		if (new_tangent_speed < FLT_EPSILON)
		{
			return;
		}

		const float cloth_friction_coeff = cloth.getPartFrictionCoeff(cloth.getVertexPartId(constraint_vertices[0]));
		const int collider_vertex = collider.getIndices((int)roundf(constraint->m_collider_triangle))[0];
		const float collider_friction_coeff = collider.getPartFrictionCoefficient(collider.getVertexPartId(collider_vertex));
		const float friction_coeff = (cloth_friction_coeff + collider_friction_coeff) * 0.5f;

		const float correction_coeff = std::max(0.0f, 1.0f + friction_coeff * normal_speed_delta / new_tangent_speed);
		const glm::vec3 modified_new_vertex_speed_vector = new_normal_speed_vector + correction_coeff * new_tangent_speed_vector;

#ifdef CHECK_DELTAS
		glm::vec3 delta = (new_vertex_speed_vector - old_vertex_speed_vector) * time_delta;
		float delta_length = glm::length(delta[0]);
		if (delta_length > MAX_LENGTH)
		{
			const float total_length = delta_length;
		}
#endif

		cloth.getTestCoords(constraint_vertices[0]) += (modified_new_vertex_speed_vector - new_vertex_speed_vector) * time_delta;
	}
}