// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "check_collisions.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

namespace CheckCollisionsMultithreaded
{
	void checkSelfVertexTriangleCollision(const Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius,
		int constraint_id)
	{
		const SelfVertexTriangleCollisionCandidate* input_constraint = (SelfVertexTriangleCollisionCandidate*)buffers.getDataBlock(constraint_id);

		// indices
		// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - vertex, 1 - triangle, 2 - zeros
		const glm::uvec3& triangle_vertices = cloth.getIndices(input_constraint->m_triangle);

		// parts
		const int vertex_part_id = cloth.getVertexPartId(input_constraint->m_vertex);
		const int triangle_part_id = cloth.getVertexPartId(triangle_vertices[0]);

		const float vertex_part_thickness = cloth.getPartThickness(vertex_part_id);
		const float triangle_part_thickness = cloth.getPartThickness(triangle_part_id);

		// coords
		const glm::vec3& vertex_coords = cloth.getCoords(input_constraint->m_vertex);
		const glm::vec3& vertex_test_coords = cloth.getTestCoords(input_constraint->m_vertex);

		const std::array<const glm::vec3*, 3> triangle_vertices_coords = cloth.getTriangleCoords(input_constraint->m_triangle);
		const std::array<const glm::vec3*, 3> triangle_vertices_test_coords = cloth.getTriangleTestCoords(input_constraint->m_triangle);

		// set constraint type and disable it
		ConstraintType constraint_type = (cloth.getPartLayerNumber(triangle_part_id) == cloth.getPartLayerNumber(vertex_part_id)) ?
			ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION : ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS;

		buffers.disableConstraint(constraint_id);

		// main calculations
		glm::vec3 closest_point_coords = { 0.0f, 0.0f, 0.0f };
		glm::vec3 closest_point_barycentric_coords = { 0.0f, 0.0f, 0.0f };
		MathUtils::closestPointOfTriangle(vertex_coords, *triangle_vertices_coords[0],
			*triangle_vertices_coords[1], *triangle_vertices_coords[2], closest_point_coords, closest_point_barycentric_coords);

		const glm::vec3 vector_from_vertex_to_closest_point = glm::normalize(closest_point_coords - vertex_coords);

		const glm::vec3 vertex_movement_projection =
			MathUtils::vectorToVectorProjection(vertex_test_coords - vertex_coords, vector_from_vertex_to_closest_point);

		const glm::vec3 triangle_vertices_movements[3] =
		{
			*triangle_vertices_test_coords[0] - *triangle_vertices_coords[0],
			*triangle_vertices_test_coords[1] - *triangle_vertices_coords[1],
			*triangle_vertices_test_coords[2] - *triangle_vertices_coords[2] };
		const glm::vec3 closest_point_movement = MathUtils::interpolateBetweenThreeVectors(triangle_vertices_movements[0],
			triangle_vertices_movements[1], triangle_vertices_movements[2], closest_point_barycentric_coords);
		const glm::vec3 closest_point_movement_projection =
			MathUtils::vectorToVectorProjection(closest_point_movement, vector_from_vertex_to_closest_point);

		float segment_1_param = 0.0f;
		float segment_2_param = 0.0f;
		glm::vec3 segment_1_point = { 0.0f, 0.0f, 0.0f };
		glm::vec3 segment_2_point = { 0.0f, 0.0f, 0.0f };
		const float closest_distance = MathUtils::distanceBetweenLinesSegments(closest_point_coords, closest_point_coords + closest_point_movement,
			vertex_coords, vertex_test_coords, segment_1_param, segment_2_param, segment_1_point, segment_2_point);

		const bool there_is_collision = (closest_distance <= (vertex_part_thickness + triangle_part_thickness + cloth_collision_radius));

		if (!there_is_collision)
		{
			return;
		}

		glm::vec3 triangle_normal = MathUtils::triangleNormal(*triangle_vertices_coords[0], *triangle_vertices_coords[1], *triangle_vertices_coords[2]);

		// TODO: is this correct?
		// Here we handle self-collision check mistake on bad meshes with big angle between neighbour triangles
		const float vector_projection = glm::dot(triangle_normal, glm::normalize(closest_point_coords - vertex_coords));
		if (fabsf(vector_projection) < 0.2f)
		{
			return;
		}

		// create constraint
		if (constraint_type == ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION)
		{
			SelfVertexTriangleCollision* output_constraint = (SelfVertexTriangleCollision*)buffers.getDataBlock(constraint_id);

			// SELF_VERTEX_TRIANGLE_COLLISION: 1 - flag if vertex is above the triangle
			output_constraint->m_is_above_triangle = (float)(glm::dot(triangle_normal, vector_from_vertex_to_closest_point) < 0.0f);

			// SELF_VERTEX_TRIANGLE_COLLISION: 1 - vertex, 3 - triangle
			CopyUtils::setSubvector(glm::value_ptr(triangle_vertices), output_constraint->m_uint_data, 1, 3);
		}
		else
		{
			SelfVertexTriangleCollisionBetweenLayers* output_constraint = (SelfVertexTriangleCollisionBetweenLayers*)buffers.getDataBlock(constraint_id);

			// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: no data

			// SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS: 1 - vertex, 3 - triangle
			CopyUtils::setSubvector(glm::value_ptr(triangle_vertices), output_constraint->m_uint_data, 1, 3);
		}

		buffers.enableConstraint(constraint_id);
		buffers.setConstraintType(constraint_type, constraint_id);
	}

	void checkSelfEdgeEdgeCollision(const Cloth& cloth, ConstraintsBuffers& buffers, float cloth_collision_radius,
		int constraint_id)
	{
		const SelfEdgeEdgeCollisionCandidate* input_constraint = (SelfEdgeEdgeCollisionCandidate*)buffers.getDataBlock(constraint_id);

		// indices
		// SELF_EDGE_EDGE_COLLISION_CANDIDATE: 1 - triangle a,  1 - edge a local index, 1 - triangle b, 1 - edge b local index

		uint32_t edge_a[2] = { 0u };
		uint32_t edge_b[2] = { 0u };
		IndicesUtils::getTriangleEdge(input_constraint->m_triangle_a_local_index, cloth.getIndices(input_constraint->m_triangle_a), edge_a);
		IndicesUtils::getTriangleEdge(input_constraint->m_triangle_b_local_index, cloth.getIndices(input_constraint->m_triangle_b), edge_b);

		// parts
		const float thickness_in_edge_1 = cloth.getPartThickness(cloth.getVertexPartId(edge_a[0]));
		const float thickness_in_edge_2 = cloth.getPartThickness(cloth.getVertexPartId(edge_b[0]));

		// coords
		const glm::vec3& edge_1_start = cloth.getCoords(edge_a[0]);
		const glm::vec3& edge_1_end = cloth.getCoords(edge_a[1]);
		const glm::vec3& edge_2_start = cloth.getCoords(edge_b[0]);
		const glm::vec3& edge_2_end = cloth.getCoords(edge_b[1]);

		buffers.disableConstraint(constraint_id);

		// main calculations
		float closest_point_1_param = 0.0f;
		float closest_point_2_param = 0.0f;
		glm::vec3 edge_1_point = { 0.0f, 0.0f, 0.0f };
		glm::vec3 edge_2_point = { 0.0f, 0.0f, 0.0f };
		const float closest_distance = MathUtils::distanceBetweenLinesSegments(edge_1_start, edge_1_end,
			edge_2_start, edge_2_end, closest_point_1_param, closest_point_2_param, edge_1_point, edge_2_point);

		const glm::vec3 edge_1_start_movement = cloth.getTestCoords(edge_a[0]) - edge_1_start;
		const glm::vec3 edge_1_end_movement = cloth.getTestCoords(edge_a[1]) - edge_1_end;
		const glm::vec3 edge_2_start_movement = cloth.getTestCoords(edge_b[0]) - edge_2_start;
		const glm::vec3 edge_2_end_movement = cloth.getTestCoords(edge_b[1]) - edge_2_end;
		const glm::vec3 closest_point_on_edge_1_movement = glm::mix(edge_1_start_movement, edge_1_end_movement, closest_point_1_param);
		const glm::vec3 closest_point_on_edge_2_movement = glm::mix(edge_2_start_movement, edge_2_end_movement, closest_point_2_param);

		const glm::vec3 vector_from_edge_1_to_edge_2 = (edge_2_point - edge_1_point) / closest_distance;
		const glm::vec3 point_1_movement_projection = MathUtils::vectorToVectorProjection(closest_point_on_edge_1_movement, vector_from_edge_1_to_edge_2);
		const glm::vec3 point_2_movement_projection = MathUtils::vectorToVectorProjection(closest_point_on_edge_2_movement, vector_from_edge_1_to_edge_2);

		float tmp_param_1 = 0.0f;
		float tmp_param_2 = 0.0f;
		const float minimal_distance = MathUtils::distanceBetweenLinesSegments(edge_1_point, edge_1_point + point_1_movement_projection,
			edge_2_point, edge_2_point + point_2_movement_projection, tmp_param_1, tmp_param_2, edge_1_point, edge_2_point);

		const bool need_create_collision = (minimal_distance <= (thickness_in_edge_1 + thickness_in_edge_2 + cloth_collision_radius));

		if (!need_create_collision)
		{
			return;
		}

		// create constraint
		SelfEdgeEdgeCollision* output_constraint = (SelfEdgeEdgeCollision*)buffers.getDataBlock(constraint_id);

		// SELF_EDGE_EDGE_COLLISION: 2 - edge a, 2 - edge b
		CopyUtils::setSubvector(edge_a, output_constraint->m_uint_data, 0, 2);
		CopyUtils::setSubvector(edge_b, output_constraint->m_uint_data, 2, 2);

		// SELF_EDGE_EDGE_COLLISION: 2 - parameters of vertices of the edges
		output_constraint->m_edge_a_param = closest_point_1_param;
		output_constraint->m_edge_b_param = closest_point_2_param;

		buffers.enableConstraint(constraint_id);
		buffers.setConstraintType(ConstraintType::SELF_EDGE_EDGE_COLLISION, constraint_id);
	}

	void checkColliderVertexTriangleCollision(const Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id)
	{
		const ColliderVertexTriangleCollisionCandidate* input_constraint = (ColliderVertexTriangleCollisionCandidate*)buffers.getDataBlock(constraint_id);

		// indices
		// COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - cloth vertex, 1 - collider triangle

		// parts
		const float cloth_thickness_in_vertex = cloth.getPartThickness(cloth.getVertexPartId(input_constraint->m_cloth_vertex));

		// coords
		const glm::vec3& vertex_coords = cloth.getCoords(input_constraint->m_cloth_vertex);
		const glm::vec3& vertex_test_coords = cloth.getTestCoords(input_constraint->m_cloth_vertex);

		const glm::uvec3& triangle_vertices = colliders.getIndices(input_constraint->m_collider_triangle);
		const std::array<const glm::vec3*, 3> triangle_vertices_coords = colliders.getTriangleCoords(input_constraint->m_collider_triangle);

		buffers.disableConstraint(constraint_id);

		// main calculations
		glm::vec3 closest_point_of_triangle_coords;
		glm::vec3 closest_point_of_triangle_barycentric_coords;
		MathUtils::closestPointOfTriangle(vertex_coords, *triangle_vertices_coords[0], *triangle_vertices_coords[1],
			*triangle_vertices_coords[2], closest_point_of_triangle_coords, closest_point_of_triangle_barycentric_coords);

		glm::vec3 vector_from_vertex_to_closest_point = glm::normalize(closest_point_of_triangle_coords - vertex_coords);
		const glm::vec3 vertex_movement_projection = MathUtils::vectorToVectorProjection(vertex_test_coords - vertex_coords, vector_from_vertex_to_closest_point);

		const float smallest_distance = MathUtils::distanceFromPointToLineSegment(vertex_coords,
			vertex_coords + vertex_movement_projection, closest_point_of_triangle_coords);
		const bool need_create_collision = (smallest_distance <= (cloth_thickness_in_vertex + collider_collision_radius));

		if (!need_create_collision)
		{
			return;
		}

		const glm::vec3 triangle_normal = MathUtils::triangleNormal(*triangle_vertices_coords[0], *triangle_vertices_coords[1], *triangle_vertices_coords[2]);

		// TODO: is this correct?
		// Here we handle self-collision check mistake on bad meshes with big angle between neighbour triangles
		const float vector_projection = glm::dot(triangle_normal, glm::normalize(closest_point_of_triangle_coords - vertex_coords));
		if (fabsf(vector_projection) < 0.2f)
		{
			return;
		}

		// create constraint
		ColliderVertexTriangleCollision* output_constraint = (ColliderVertexTriangleCollision*)buffers.getDataBlock(constraint_id);

		// COLLIDER_VERTEX_TRIANGLE_COLLISION:  1 - collider triangle id
		output_constraint->m_collider_triangle = (float)input_constraint->m_collider_triangle;

		// COLLIDER_VERTEX_TRIANGLE_COLLISION: 1 - one vertex of the cloth
		output_constraint->m_uint_data[0] = input_constraint->m_cloth_vertex;

		buffers.enableConstraint(constraint_id);
		buffers.setConstraintType(ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION, constraint_id);
	}

	void checkColliderEdgeEdgeCollision(const Cloth& cloth, ConstraintsBuffers& buffers,
		const Collider& colliders, float collider_collision_radius, int constraint_id)
	{
		const ColliderEdgeEdgeCollisionCandidate* input_constraint = (ColliderEdgeEdgeCollisionCandidate*)buffers.getDataBlock(constraint_id);

		// indices
		// COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE: 1 - cloth triangle, 1 - cloth edge local index, 1 - collider triangle,
		// 1 - collider triangle local index

		const glm::uvec3& cloth_triangle_vertices = cloth.getIndices(input_constraint->m_cloth_triangle);
		const glm::uvec3& collider_triangle_vertices = colliders.getIndices(input_constraint->m_collider_triangle);

		uint32_t cloth_edge[2] = { 0u };
		uint32_t collider_edge[2] = { 0u };

		IndicesUtils::getTriangleEdge(input_constraint->m_cloth_edge_local_index, cloth_triangle_vertices, cloth_edge);
		IndicesUtils::getTriangleEdge(input_constraint->m_collider_triangle_local_index, collider_triangle_vertices, collider_edge);

		// parts
		const float cloth_edge_thickness = cloth.getPartThickness(cloth.getVertexPartId((int)cloth_edge[0]));
		const float distance_to_create_constraint = cloth_edge_thickness + collider_collision_radius;

		// coords
		const glm::vec3& cloth_edge_start = cloth.getCoords((int)cloth_edge[0]);
		const glm::vec3& cloth_edge_end = cloth.getCoords((int)cloth_edge[1]);

		const glm::vec3& collider_edge_start = colliders.getCoords((int)collider_edge[0]);
		const glm::vec3& collider_edge_end = colliders.getCoords((int)collider_edge[1]);

		buffers.disableConstraint(constraint_id);

		// main calculations
		float cloth_edge_param = 0.0f;
		float collider_edge_param = 0.0f;
		glm::vec3 cloth_edge_point = { 0.0f, 0.0f, 0.0f };
		glm::vec3 collider_edge_point = { 0.0f, 0.0f, 0.0f };
		float minimal_distance = MathUtils::distanceBetweenLinesSegments(cloth_edge_start, cloth_edge_end, collider_edge_start,
			collider_edge_end, cloth_edge_param, collider_edge_param, cloth_edge_point, collider_edge_point);

		const glm::vec3 cloth_to_collider_vector = (collider_edge_point - cloth_edge_point) / minimal_distance;

		const glm::vec3 cloth_edge_start_movement = cloth.getTestCoords((int)cloth_edge[0]) - cloth_edge_start;
		const glm::vec3 cloth_edge_end_movement = cloth.getTestCoords((int)cloth_edge[1]) - cloth_edge_end;

		glm::vec3 closest_point_on_cloth_edge_movement =
			glm::mix(cloth_edge_start_movement, cloth_edge_end_movement, cloth_edge_param);
		const glm::vec3 cloth_edge_movement_projection =
			MathUtils::vectorToVectorProjection(closest_point_on_cloth_edge_movement, cloth_to_collider_vector);

		minimal_distance = MathUtils::distanceFromPointToLineSegment(cloth_edge_point,
			cloth_edge_point + cloth_edge_movement_projection, collider_edge_point);
		const bool need_create_collision = (minimal_distance <= distance_to_create_constraint);

		if (!need_create_collision)
		{
			return;
		}

		// create constraint
		ColliderEdgeEdgeCollision* output_constraint = (ColliderEdgeEdgeCollision*)buffers.getDataBlock(constraint_id);

		// COLLIDER_EDGE_EDGE_COLLISION:1 - point parameter on cloth edge, 1 - point parameter on collider edge,
		// 1 - collider edge vertex a, 1 - collider edge vertex b
		output_constraint->m_cloth_edge_param = cloth_edge_param;
		output_constraint->m_collider_edge_param = collider_edge_param;
		output_constraint->m_collider_edge_vertex_a = (float)collider_edge[0];
		output_constraint->m_collider_edge_vertex_b = (float)collider_edge[1];

		// COLLIDER_EDGE_EDGE_COLLISION: 2 - one edge
		CopyUtils::setSubvector(cloth_edge, output_constraint->m_uint_data, 0, 2);

		buffers.enableConstraint(constraint_id);
		buffers.setConstraintType(ConstraintType::COLLIDER_EDGE_EDGE_COLLISION, constraint_id);
	}
}