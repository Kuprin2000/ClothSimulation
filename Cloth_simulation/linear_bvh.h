#pragma once
#include <vector>
#include <array>
#include <execution>
#include <stack>
#include <list>
#include <cmath>
#include "math_utils.h"
#include "indices_utils.h"
#include "bounding_box.h"
#include "aligned_vector.h"

static const uint32_t NO_PRIMITIVE = UINT32_MAX;

struct Node
{
	Node* m_parent = nullptr;
	Node* m_left_child = nullptr;
	Node* m_right_child = nullptr;
	uint32_t m_primitive_id = NO_PRIMITIVE;
	BoundingBox m_bnd_box;
	uint16_t reserved[4] = { 0u, 0u, 0u, 0u };
};

class LinearBVH
{
public:
	LinearBVH() = default;

	LinearBVH(const LinearBVH&) = delete;

	LinearBVH(LinearBVH&&) = default;

	LinearBVH& operator=(LinearBVH&& other) = default;

	void insertTriangles(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap);

	void insertMovingTriangles(const AlignedVector::AlignedVector<glm::vec3>& old_vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& new_vertices_coords,
		const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap);

	_NODISCARD std::vector<int> findCollisionsWithBndBox(const BoundingBox& bnd_box);

	_NODISCARD const AlignedVector::AlignedVector<Node>& getPrimitivesNodes() const
	{
		return m_leaf_nodes;
	}

private:
	struct PointWithCode
	{
		uint32_t index = 0u;
		uint32_t code = 0u;
	};

	_NODISCARD uint32_t expandBits(uint32_t value) const
	{
		value = (value * 0x00010001u) & 0xFF0000FFu;
		value = (value * 0x00000101u) & 0x0F00F00Fu;
		value = (value * 0x00000011u) & 0xC30C30C3u;
		value = (value * 0x00000005u) & 0x49249249u;
		return value;
	}

	_NODISCARD AlignedVector::AlignedVector<PointWithCode> sortMorton(const AlignedVector::AlignedVector<glm::vec3>& points) const;

	_NODISCARD int delta(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int a, int b) const;

	_NODISCARD int safe_delta(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int a, int b) const;

	_NODISCARD glm::ivec2 determineRange(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int i) const;

	_NODISCARD int findSplit(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, glm::ivec2 range) const;

	void generateHeirarchy(const AlignedVector::AlignedVector<PointWithCode>& sorted_points);

	void createLayers(Node* current_node, int layer, std::vector<std::vector<Node*>>& layers) const;

	void calcBoundingBoxes(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap);

	void calcBoundingBoxes(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& vertices_test_coords,
		const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap);

private:
	AlignedVector::AlignedVector<Node> m_leaf_nodes;
	AlignedVector::AlignedVector<Node> m_internal_nodes;
};