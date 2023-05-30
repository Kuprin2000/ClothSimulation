// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "collider.h"
#include <algorithm>

void Collider::pushColliders(const std::vector<Collider>& colliders)
{
	const bool is_empty = m_coords.empty();

	const int old_parts_count = is_empty ? 0 : (int)m_borders.size();
	int number_of_parts_to_add = 0;
	for (const auto& collider : colliders)
	{
		number_of_parts_to_add += (int)collider.m_borders.size();
	}
	const int new_parts_count = old_parts_count + number_of_parts_to_add;

	const int old_vertices_count = (int)m_coords.size();
	const int old_triangles_count = (int)m_indices.size();

	int new_vertices_count = old_vertices_count;
	int new_triangles_count = old_triangles_count;
	for (const auto& collider : colliders)
	{
		new_vertices_count += (int)collider.getCoords().size();
		new_triangles_count += (int)collider.getIndices().size();
	}

	m_coords.reserve(new_vertices_count);
	for (const auto& collider : colliders)
	{
		m_coords.insert(m_coords.end(), collider.m_coords.begin(), collider.m_coords.end());
	}

	m_normals.reserve(new_vertices_count);
	for (const auto& collider : colliders)
	{
		m_normals.insert(m_normals.end(), collider.m_normals.begin(), collider.m_normals.end());
	}

	m_indices.reserve(new_triangles_count);
	int tmp_size = 0;
	int index_accumulator = old_vertices_count;
	for (const auto& collider : colliders)
	{
		tmp_size = (int)m_indices.size();
		m_indices.insert(m_indices.end(), collider.m_indices.begin(), collider.m_indices.end());
		std::for_each(m_indices.begin() + tmp_size, m_indices.end(),
			[index_accumulator](glm::uvec3& value)
			{
				value[0] += index_accumulator;
				value[1] += index_accumulator;
				value[2] += index_accumulator;
			});

		index_accumulator += (int)collider.m_coords.size();
	}

	m_primitives_ownership.reserve(new_triangles_count);
	for (const auto& collider : colliders)
	{
		m_primitives_ownership.insert(m_primitives_ownership.end(), collider.m_primitives_ownership.begin(), collider.m_primitives_ownership.end());
	}

	m_borders.resize(old_parts_count);
	m_borders.reserve(new_parts_count);
	index_accumulator = old_vertices_count;

	for (const auto& collider : colliders)
	{
		m_borders.insert(m_borders.end(), collider.m_borders.begin(), collider.m_borders.end());

		std::for_each(m_borders.begin() + old_parts_count, m_borders.end(),
			[index_accumulator](int& value)
			{
				value += index_accumulator;
			});

		index_accumulator += (int)collider.m_coords.size();
	}

	m_friction_coefficients.resize(old_parts_count);
	m_friction_coefficients.reserve(new_parts_count);
	for (const auto& collider : colliders)
	{
		m_friction_coefficients.insert(m_friction_coefficients.end(), collider.m_friction_coefficients.begin(), collider.m_friction_coefficients.end());
	}
}
