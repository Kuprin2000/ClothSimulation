// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "linear_bvh.h"

void LinearBVH::insertTriangles(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap)
{
	AlignedVector::AlignedVector<glm::vec3> centers(vertices_indices.size(), { 0.0f, 0.0f, 0.0f });
	const float coeff_1 = 1.0f / 3.0f;
	for (int i = 0; i < vertices_indices.size(); ++i)
	{
		const glm::uvec3& indices = vertices_indices[i];
		centers[i] = (vertices_coords[indices[0]] + vertices_coords[indices[1]] + vertices_coords[indices[2]]) * coeff_1;
	}

	float max_delta = 0.0f;
	glm::vec3 min_values = { FLT_MAX, FLT_MAX, FLT_MAX };
	for (const auto& elem : centers)
	{
		max_delta = std::max({ max_delta , fabs(elem.x), fabs(elem.y), fabs(elem.z) });
		min_values = { std::min(min_values.x, elem.x), std::min(min_values.y, elem.y), std::min(min_values.z, elem.z) };
	}
	float coeff_2 = 0.5f / max_delta;

	for (int i = 0; i < centers.size(); ++i)
	{
		centers[i] -= min_values;
		centers[i] *= coeff_2;
	}

	const AlignedVector::AlignedVector<PointWithCode> sorted_vertices = sortMorton(centers);

	generateHeirarchy(sorted_vertices);

	calcBoundingBoxes(vertices_coords, vertices_indices, gap);
}

void LinearBVH::insertMovingTriangles(const AlignedVector::AlignedVector<glm::vec3>& old_vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& new_vertices_coords,
	const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap)
{
	AlignedVector::AlignedVector<glm::vec3> centers(vertices_indices.size(), { 0.0f, 0.0f, 0.0f });
	const float coeff_1 = 1.0f / 6.0f;
	for (int i = 0; i < vertices_indices.size(); ++i)
	{
		const glm::uvec3& indices = vertices_indices[i];
		centers[i] = old_vertices_coords[indices[0]] + old_vertices_coords[indices[1]] + old_vertices_coords[indices[2]];
		centers[i] += new_vertices_coords[indices[0]] + new_vertices_coords[indices[1]] + new_vertices_coords[indices[2]];
		centers[i] *= coeff_1;
	}

	float max_delta = 0.0f;
	glm::vec3 min_values = { FLT_MAX, FLT_MAX, FLT_MAX };
	for (const auto& elem : centers)
	{
		max_delta = std::max({ max_delta , fabs(elem.x), fabs(elem.y), fabs(elem.z) });
		min_values = { std::min(min_values.x, elem.x), std::min(min_values.y, elem.y), std::min(min_values.z, elem.z) };
	}
	float coeff_2 = 0.5f / max_delta;

	for (int i = 0; i < centers.size(); ++i)
	{
		centers[i] -= min_values;
		centers[i] *= coeff_2;
	}

	const AlignedVector::AlignedVector<PointWithCode> sorted_vertices = sortMorton(centers);

	generateHeirarchy(sorted_vertices);

	calcBoundingBoxes(old_vertices_coords, new_vertices_coords, vertices_indices, gap);
}

std::vector<int> LinearBVH::findCollisionsWithBndBox(const BoundingBox& bnd_box)
{
	if (m_leaf_nodes.empty())
	{
		return {};
	}

	std::vector<int> result;

	const Node* current_node = &m_internal_nodes[0];

	std::stack<const Node*> candidates_stack;
	candidates_stack.push(nullptr);

	while (current_node)
	{
		const bool left_overlaps = current_node->m_left_child && BoundingBox::thereIsOVerlap(current_node->m_left_child->m_bnd_box, bnd_box);
		const bool right_overlaps = current_node->m_right_child && BoundingBox::thereIsOVerlap(current_node->m_right_child->m_bnd_box, bnd_box);

		if (left_overlaps && current_node->m_left_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_left_child->m_primitive_id);
		}
		if (right_overlaps && current_node->m_right_child->m_primitive_id != NO_PRIMITIVE)
		{
			result.push_back(current_node->m_right_child->m_primitive_id);
		}

		const bool left_is_candidate = left_overlaps && (current_node->m_left_child->m_primitive_id == NO_PRIMITIVE);
		const bool right_is_candidate = right_overlaps && (current_node->m_right_child->m_primitive_id == NO_PRIMITIVE);

		if (left_is_candidate || right_is_candidate)
		{
			const Node* right_child = current_node->m_right_child;
			current_node = left_is_candidate ? current_node->m_left_child : current_node->m_right_child;

			if (left_is_candidate && right_is_candidate)
			{
				candidates_stack.push(right_child);
			}
		}
		else
		{
			current_node = candidates_stack.top();
			candidates_stack.pop();
		}
	}

	return result;
}

_NODISCARD AlignedVector::AlignedVector<LinearBVH::PointWithCode> LinearBVH::sortMorton(const AlignedVector::AlignedVector<glm::vec3>& points) const
{
	AlignedVector::AlignedVector<PointWithCode> points_to_sort(points.size());

	for (uint32_t i = 0; i < points.size(); ++i)
	{
		glm::vec3 elem = points[i];
		elem.x = std::min(std::max(elem.x * 1024.0f, 0.0f), 1023.0f);
		elem.y = std::min(std::max(elem.y * 1024.0f, 0.0f), 1023.0f);
		elem.z = std::min(std::max(elem.z * 1024.0f, 0.0f), 1023.0f);
		points_to_sort[i] =
		{ i, expandBits((uint32_t)elem.x) * 4 + expandBits((uint32_t)elem.y) * 2 + expandBits((uint32_t)elem.z) };
	}

	std::sort(std::execution::par, points_to_sort.begin(), points_to_sort.end(), [](PointWithCode a, PointWithCode b) { return a.code < b.code; });

	return points_to_sort;
}

_NODISCARD int LinearBVH::delta(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int a, int b) const
{
	return __lzcnt(sorted_points[a].code ^ sorted_points[b].code);
}

_NODISCARD int LinearBVH::safe_delta(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int a, int b) const
{
	return (b < 0 || b >= sorted_points.size()) ? -1 : __lzcnt(sorted_points[a].code ^ sorted_points[b].code);
}

_NODISCARD glm::ivec2 LinearBVH::determineRange(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, int i) const
{
	const int direction = MathUtils::sign(delta(sorted_points, i, i + 1) - safe_delta(sorted_points, i, i - 1));
	const int delta_min = safe_delta(sorted_points, i, i - direction);

	int l_max = 2;
	while (safe_delta(sorted_points, i, i + l_max * direction) > delta_min)
	{
		l_max *= 2;
	}

	int l = 0;
	for (int t = l_max / 2; t; t /= 2)
	{
		if (safe_delta(sorted_points, i, i + (l + t) * direction) > delta_min)
		{
			l += t;
		}
	}

	return { i, i + l * direction };
}

_NODISCARD int LinearBVH::findSplit(const AlignedVector::AlignedVector<PointWithCode>& sorted_points, glm::ivec2 range) const
{
	const int delta_node = safe_delta(sorted_points, range.x, range.y);
	int s = 0;
	int t = (range.x > range.y) ? range.x - range.y : range.y - range.x;
	int direction = (range.x > range.y) ? -1 : 1;

	do
	{
		t = (t + 1) >> 1;
		if (safe_delta(sorted_points, range.x, range.x + (s + t) * direction) > delta_node)
		{
			s += t;
		}
	} while (t > 1);

	return range.x + s * direction + std::min(direction, 0);
}

void LinearBVH::generateHeirarchy(const AlignedVector::AlignedVector<PointWithCode>& sorted_points)
{
	m_leaf_nodes.clear();
	m_internal_nodes.clear();

	const int points_count = (int)sorted_points.size();
	m_leaf_nodes.resize(points_count);
	m_internal_nodes.resize(points_count - 1);

	for (int i = 0; i < points_count; ++i)
	{
		m_leaf_nodes[i].m_primitive_id = sorted_points[i].index;
	}

#pragma omp parallel for
	for (int i = 0; i < points_count - 1; ++i)
	{
		const glm::ivec2 range = determineRange(sorted_points, i);
		const int split = findSplit(sorted_points, range);

		Node* left_child = (std::min(range.x, range.y) == split) ? &m_leaf_nodes[split] : &m_internal_nodes[split];
		Node* right_child = (std::max(range.x, range.y) == split + 1) ? &m_leaf_nodes[split + 1] : &m_internal_nodes[split + 1];

		m_internal_nodes[i].m_left_child = left_child;
		m_internal_nodes[i].m_right_child = right_child;
		left_child->m_parent = &m_internal_nodes[i];
		right_child->m_parent = &m_internal_nodes[i];
	}
}

void LinearBVH::createLayers(Node* current_node, int layer, std::vector<std::vector<Node*>>& layers) const
{
	if (layers.size() < layer + 1)
	{
		layers.resize(layer + 1);
	}

	layers[layer].push_back(current_node);

	if (current_node->m_left_child && current_node->m_left_child->m_primitive_id == NO_PRIMITIVE)
	{
		createLayers(current_node->m_left_child, layer + 1, layers);
	}
	if (current_node->m_right_child && current_node->m_right_child->m_primitive_id == NO_PRIMITIVE)
	{
		createLayers(current_node->m_right_child, layer + 1, layers);
	}
}

void LinearBVH::calcBoundingBoxes(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap)
{
#pragma omp parallel for
	for (int i = 0; i < m_leaf_nodes.size(); ++i)
	{
		const glm::uvec3& indices = vertices_indices[m_leaf_nodes[i].m_primitive_id];
		const glm::vec3 triangle_vertices_coords[3] =
		{
			vertices_coords[indices[0]],
			vertices_coords[indices[1]],
			vertices_coords[indices[2]]
		};

		BoundingBox box;
		box.setTriangle(triangle_vertices_coords, gap);
		m_leaf_nodes[i].m_bnd_box = box;
	}

	std::vector<std::vector<Node*>> layers((uint32_t)log2f(float(m_internal_nodes.size())) + 2u);
	createLayers(&m_internal_nodes[0], 0, layers);

#pragma omp parallel
	{
		for (int i = (int)layers.size() - 1; i >= 0; --i)
		{
#pragma omp for
			for (int j = 0; j < layers[i].size(); ++j)
			{
				BoundingBox box;
				Node* node = layers[i][j];
				if (node->m_left_child)
				{
					box.addBndBox(node->m_left_child->m_bnd_box);
				}
				if (node->m_right_child)
				{
					box.addBndBox(node->m_right_child->m_bnd_box);
				}
				node->m_bnd_box = box;
			}
		}
	}
}

void LinearBVH::calcBoundingBoxes(const AlignedVector::AlignedVector<glm::vec3>& vertices_coords, const AlignedVector::AlignedVector<glm::vec3>& vertices_test_coords,
	const AlignedVector::AlignedVector<glm::uvec3>& vertices_indices, float gap)
{
#pragma omp parallel for
	for (int i = 0; i < (int)m_leaf_nodes.size(); ++i)
	{
		const glm::uvec3& indices = vertices_indices[m_leaf_nodes[i].m_primitive_id];
		const glm::vec3 triangle_vertices_coords[3] =
		{
			vertices_coords[indices[0]],
			vertices_coords[indices[1]],
			vertices_coords[indices[2]]
		};

		const glm::vec3 triangle_vertices_test_coords[3] =
		{
			vertices_test_coords[indices[0]],
			vertices_test_coords[indices[1]],
			vertices_test_coords[indices[2]]
		};

		BoundingBox box;
		box.setMovingTriangle(triangle_vertices_coords, triangle_vertices_test_coords, gap);
		m_leaf_nodes[i].m_bnd_box = box;
	}

	std::vector<std::vector<Node*>> layers((uint32_t)log2f(float(m_internal_nodes.size())) + 2u);
	createLayers(&m_internal_nodes[0], 0, layers);

#pragma omp parallel
	{
		for (int i = (int)layers.size() - 1; i >= 0; --i)
		{
#pragma omp for
			for (int j = 0; j < layers[i].size(); ++j)
			{
				BoundingBox box;
				Node* node = layers[i][j];
				if (node->m_left_child)
				{
					box.addBndBox(node->m_left_child->m_bnd_box);
				}
				if (node->m_right_child)
				{
					box.addBndBox(node->m_right_child->m_bnd_box);
				}
				node->m_bnd_box = box;
			}
		}
	}
}