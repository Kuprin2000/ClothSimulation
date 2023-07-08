#pragma once
#include <set>
#include <vector>
#include "indices_utils.h"
#include "keys_utils.h"
#include "aligned_vector.h"

namespace PrimitivesOwnershipUtils
{
	union TrianglePrimitivesOwnership
	{
		uint8_t raw_data;

		struct
		{
			uint8_t vertex_a : 1;
			uint8_t vertex_b : 1;
			uint8_t vertex_c : 1;
			uint8_t edge_a : 1;
			uint8_t edge_b : 1;
			uint8_t edge_c : 1;
		} structure;
	};

	_NODISCARD AlignedVector::AlignedVector<uint8_t> generatePrimitivesOwnership(const AlignedVector::AlignedVector<glm::uvec3>& indices);

	_NODISCARD inline bool triangleOwnsTheVertex(int vertex_local_index, TrianglePrimitivesOwnership primitives_ownership)
	{
		return (primitives_ownership.raw_data >> vertex_local_index) & 0x1;
	}

	_NODISCARD inline bool triangleOwnsTheEdge(int edge_local_index, TrianglePrimitivesOwnership primitives_ownership)
	{
		return (primitives_ownership.raw_data >> (edge_local_index + 3)) & 0x1;
	}
}
