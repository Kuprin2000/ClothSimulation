// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "primitives_ownership_utils.h"
#include <array>

AlignedVector<uint8_t> PrimitivesOwnershipUtils::generatePrimitivesOwnership(const AlignedVector<glm::uvec3>& indices)
{
	// Here we setup data on what primitives (vertices and edges) each triangle owns.
	// Triangle can own some of it's vertices and some of it's edges. Each vertex and
	// each edge can be owned by only one triangle. We need to distribute primitives
	// among triangles for collisions search optimization. You can read about this in
	// Chris Lewin's "Cloth Self Collision with Predictive Contacts" in "Rep-Tri" paragraph.
	// This function creates bitmask for each triangle to show primitives it owns.

	// primitives that are already owned
	std::set<uint32_t> owned_vertices;
	std::set<std::array<uint32_t, 2>> owned_edges;

	// result array
	AlignedVector<uint8_t> result((int)indices.size());

	// bitmask for triangle
	TrianglePrimitivesOwnership owned_primitives;

	int triangle_index = 0;
	std::array<uint32_t, 2> key_for_edge;

	for (const auto& triangle_vertices : indices)
	{
		owned_primitives.raw_data = 0u;

		// check vertices
		if (!owned_vertices.contains(triangle_vertices[0]))
		{
			owned_vertices.insert(triangle_vertices[0]);
			owned_primitives.structure.vertex_a = 1;
		}
		if (!owned_vertices.contains(triangle_vertices[1]))
		{
			owned_vertices.insert(triangle_vertices[1]);
			owned_primitives.structure.vertex_b = 1;
		}
		if (!owned_vertices.contains(triangle_vertices[2]))
		{
			owned_vertices.insert(triangle_vertices[2]);
			owned_primitives.structure.vertex_c = 1;
		}

		// check edges
		// edge 0,1
		key_for_edge =
		{ std::min(triangle_vertices[0], triangle_vertices[1]), std::max(triangle_vertices[0], triangle_vertices[1]) };

		if (!owned_edges.contains(key_for_edge))
		{
			owned_edges.insert(key_for_edge);
			owned_primitives.structure.edge_a = 1;
		}

		// edge 0,2
		key_for_edge =
		{ std::min(triangle_vertices[0], triangle_vertices[2]), std::max(triangle_vertices[0], triangle_vertices[2]) };

		if (!owned_edges.contains(key_for_edge))
		{
			owned_edges.insert(key_for_edge);
			owned_primitives.structure.edge_b = 1;
		}

		// edge 1,2
		key_for_edge =
		{ std::min(triangle_vertices[1], triangle_vertices[2]), std::max(triangle_vertices[1], triangle_vertices[2]) };

		if (!owned_edges.contains(key_for_edge))
		{
			owned_edges.insert(key_for_edge);
			owned_primitives.structure.edge_c = 1;
		}

		result[triangle_index] = owned_primitives.raw_data;
		++triangle_index;
	}

	return result;
}