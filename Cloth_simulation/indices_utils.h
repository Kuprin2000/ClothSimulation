#pragma once
#include "glm/glm.hpp"

namespace IndicesUtils
{
	static const uint32_t NO_HOST_VERTEX = UINT32_MAX;

	inline void getTriangleEdge(int edge_local_index, const glm::uvec3& triangle_vertices, uint32_t* dst)
	{
		constexpr int index_1[3] = { 0, 0, 1 };
		constexpr int index_2[3] = { 1, 2, 1 };

		dst[0] = triangle_vertices[index_1[edge_local_index]];
		dst[1] = triangle_vertices[index_2[edge_local_index]];
	}
}
