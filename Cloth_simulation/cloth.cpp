// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "cloth.h"
#include <algorithm>
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "primitives_ownership_utils.h"
#include "copy_utils.h"

Cloth::Cloth(const AlignedVector::AlignedVector<glm::vec3>& coords, const AlignedVector::AlignedVector<glm::uvec3>& indices, const AlignedVector::AlignedVector<float>& opposite_masses,
	const MaterialProperties& material_props, int layer_number, bool use_realistic_stretch, bool use_realistic_bending)
{
	m_vertices_data.m_coords = coords;
	m_vertices_data.m_test_coords = coords;
	m_vertices_data.m_indices = indices;
	m_vertices_data.m_normals.resize(m_vertices_data.m_coords.size(), { 0.0f, 0.0f, 0.0f });
	m_vertices_data.m_opposite_masses = opposite_masses;
	m_vertices_data.m_speeds.resize(coords.size(), { 0.0f, 0.0f, 0.0f });

	m_vertices_data.m_host_and_original_vertices.resize(coords.size());
	for (int i = 0; i < m_vertices_data.m_host_and_original_vertices.size(); ++i)
	{
		m_vertices_data.m_host_and_original_vertices[i] = i;
	}

	m_vertices_data.m_primitives_ownership = PrimitivesOwnershipUtils::generatePrimitivesOwnership(indices);
	updateNormals();

	setupInternalConstraints(material_props, use_realistic_stretch, use_realistic_bending);

	m_parts_data.m_borders = { (int)coords.size() };
	m_parts_data.m_layers_numbers = { 1u };
	m_parts_data.m_bending_stiffness = { material_props.m_bending_stiffness };
	m_parts_data.m_friction_coeffs = { material_props.m_friction_coeff };
	m_parts_data.m_thicknesses = { material_props.m_thickness };

	m_parts_data.m_elasticity_tensors = { elasticityTensor(material_props) };
}

void Cloth::pushClothes(const std::vector<Cloth>& clothes)
{
	bool ok = !getPhantomVerticesCount();
	for (const auto& cloth : clothes)
	{
		ok = ok && !cloth.getPhantomVerticesCount();
	}

	if (!ok)
	{
		throw std::exception("Can't concat cloth with phantom vertices!");
	}

	const bool is_empty = m_vertices_data.m_coords.empty();

	// setup counters
	const int old_parts_count = is_empty ? 0 : (int)m_parts_data.m_borders.size();
	int number_of_clothes = 0;
	for (const auto& cloth : clothes)
	{
		number_of_clothes += (int)cloth.m_parts_data.m_borders.size();
	}
	const int new_parts_count = old_parts_count + number_of_clothes;

	const int old_vertices_count = (int)m_vertices_data.m_coords.size();
	const int old_triangles_count = (int)m_vertices_data.m_indices.size();

	int new_vertices_count = old_vertices_count;
	int new_triangles_count = old_triangles_count;
	for (const auto& cloth : clothes)
	{
		new_vertices_count += (int)cloth.m_vertices_data.m_coords.size();
		new_triangles_count += (int)cloth.m_vertices_data.m_indices.size();
	}

	// update m_vertices_data

	m_vertices_data.m_coords.reserve(new_vertices_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_coords.insert(m_vertices_data.m_coords.end(),
			cloth.m_vertices_data.m_coords.begin(), cloth.m_vertices_data.m_coords.end());
	}

	m_vertices_data.m_test_coords.reserve(new_vertices_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_test_coords.insert(m_vertices_data.m_test_coords.end(),
			cloth.m_vertices_data.m_test_coords.begin(), cloth.m_vertices_data.m_test_coords.end());
	}

	m_vertices_data.m_indices.reserve(new_triangles_count);
	int index_accumulator = old_vertices_count;
	int tmp_size = 0;
	for (const auto& cloth : clothes)
	{
		const auto& tmp_ref = cloth.m_vertices_data;

		tmp_size = (int)m_vertices_data.m_indices.size();
		m_vertices_data.m_indices.insert(m_vertices_data.m_indices.end(),
			tmp_ref.m_indices.begin(), tmp_ref.m_indices.end());

		std::for_each(m_vertices_data.m_indices.begin() + tmp_size, m_vertices_data.m_indices.end(),
			[index_accumulator](glm::uvec3& value)
			{
				value[0] += index_accumulator;
				value[1] += index_accumulator;
				value[2] += index_accumulator;
			});

		index_accumulator += (int)tmp_ref.m_coords.size();
	}

	m_vertices_data.m_normals.reserve(new_vertices_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_normals.insert(m_vertices_data.m_normals.end(),
			cloth.m_vertices_data.m_normals.begin(), cloth.m_vertices_data.m_normals.end());
	}

	m_vertices_data.m_opposite_masses.reserve(new_vertices_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_opposite_masses.insert(m_vertices_data.m_opposite_masses.end(),
			cloth.m_vertices_data.m_opposite_masses.begin(), cloth.m_vertices_data.m_opposite_masses.end());
	}

	m_vertices_data.m_speeds.reserve(new_vertices_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_speeds.insert(m_vertices_data.m_speeds.end(),
			cloth.m_vertices_data.m_speeds.begin(), cloth.m_vertices_data.m_speeds.end());
	}

	m_vertices_data.m_host_and_original_vertices.reserve(new_vertices_count);
	index_accumulator = old_vertices_count;
	for (const auto& cloth : clothes)
	{
		const auto& tmp_ref = cloth.m_vertices_data;

		tmp_size = (int)m_vertices_data.m_host_and_original_vertices.size();
		m_vertices_data.m_host_and_original_vertices.insert(m_vertices_data.m_host_and_original_vertices.end(),
			tmp_ref.m_host_and_original_vertices.begin(), tmp_ref.m_host_and_original_vertices.end());

		std::for_each(m_vertices_data.m_host_and_original_vertices.begin() + tmp_size, m_vertices_data.m_host_and_original_vertices.end(),
			[index_accumulator](uint32_t& value)
			{
				value += index_accumulator;

			});

		index_accumulator += (int)tmp_ref.m_coords.size();
	}

	m_vertices_data.m_primitives_ownership.reserve(new_triangles_count);
	for (const auto& cloth : clothes)
	{
		m_vertices_data.m_primitives_ownership.insert(m_vertices_data.m_primitives_ownership.end(),
			cloth.m_vertices_data.m_primitives_ownership.begin(), cloth.m_vertices_data.m_primitives_ownership.end());
	}

	// update m_internal_constraints

	std::array<int, CONSTRAINT_TYPES_COUNT> total_constraints_per_type = m_constraints_data.m_internal_constraints.getStoredConstraintsCounters();
	for (const auto& cloth : clothes)
	{
		const std::array<int, CONSTRAINT_TYPES_COUNT>& constraints_per_type =
			cloth.m_constraints_data.m_internal_constraints.getStoredConstraintsCounters();

		for (int j = 0; j < CONSTRAINT_TYPES_COUNT; ++j)
		{
			total_constraints_per_type[j] += constraints_per_type[j];
		}
	}

	ConstraintsBuffers new_buffers = ConstraintsBuffers::reserveBuffers(total_constraints_per_type);
	new_buffers.pushBuffer(m_constraints_data.m_internal_constraints, 0);

	int new_constraint_id = m_constraints_data.m_internal_constraints.getConstraintsCount();
	ConstraintType new_constraint_type;
	std::vector<uint32_t> new_constraint_vertices;
	std::vector<KeysUtils::ConstraintKey> new_constraint_keys;
	index_accumulator = old_vertices_count;

	for (const auto& cloth : clothes)
	{
		const ConstraintsBuffers& constraints_to_add = cloth.m_constraints_data.m_internal_constraints;
		new_buffers.pushBuffer(constraints_to_add, index_accumulator);

		// add new constraints to the search map
		for (int j = 0; j < constraints_to_add.getConstraintsCount(); ++j)
		{
			new_constraint_type = constraints_to_add.getConstraintType(j);
			new_constraint_vertices =
			{ constraints_to_add.getConstraintVertices(j), constraints_to_add.getConstraintVertices(j) + CONSTRAINT_UINT_COUNT[(int)new_constraint_type] };
			new_constraint_keys = KeysUtils::generateConstraintKeys(new_constraint_vertices, new_constraint_type);

			std::for_each(new_constraint_keys.begin(), new_constraint_keys.end(), [index_accumulator](KeysUtils::ConstraintKey& key)
				{
					key[0] += index_accumulator;
					key[1] += index_accumulator; });

			KeysUtils::insertKeysToMap(new_constraint_keys, new_constraint_id, m_constraints_data.m_internal_constraints_map);
			++new_constraint_id;
		}

		index_accumulator += (int)cloth.m_vertices_data.m_coords.size();
	}
	m_constraints_data.m_internal_constraints = std::move(new_buffers);

	// update m_parts_data

	m_parts_data.m_borders.resize(old_parts_count);
	m_parts_data.m_borders.reserve(new_parts_count);
	index_accumulator = old_vertices_count;

	for (const auto& cloth : clothes)
	{
		tmp_size = (int)m_parts_data.m_borders.size();
		m_parts_data.m_borders.insert(m_parts_data.m_borders.end(),
			cloth.m_parts_data.m_borders.begin(), cloth.m_parts_data.m_borders.end());
		std::for_each(m_parts_data.m_borders.begin() + tmp_size, m_parts_data.m_borders.end(),
			[index_accumulator](int& value)
			{
				value += index_accumulator;
			});

		index_accumulator += (int)cloth.m_vertices_data.m_coords.size();
	}

	m_parts_data.m_layers_numbers.resize(old_parts_count);
	m_parts_data.m_layers_numbers.reserve(new_parts_count);
	for (const auto& cloth : clothes)
	{
		m_parts_data.m_layers_numbers.insert(m_parts_data.m_layers_numbers.end(),
			cloth.m_parts_data.m_layers_numbers.begin(), cloth.m_parts_data.m_layers_numbers.end());
	}

	m_parts_data.m_elasticity_tensors.resize(old_parts_count);
	m_parts_data.m_elasticity_tensors.reserve(new_parts_count);
	for (const auto& cloth : clothes)
	{
		m_parts_data.m_elasticity_tensors.insert(m_parts_data.m_elasticity_tensors.end(),
			cloth.m_parts_data.m_elasticity_tensors.begin(), cloth.m_parts_data.m_elasticity_tensors.end());
	}

	m_parts_data.m_bending_stiffness.resize(old_parts_count);
	m_parts_data.m_bending_stiffness.reserve(new_parts_count);
	for (const auto& cloth : clothes)
	{
		m_parts_data.m_bending_stiffness.insert(m_parts_data.m_bending_stiffness.end(),
			cloth.m_parts_data.m_bending_stiffness.begin(), cloth.m_parts_data.m_bending_stiffness.end());
	}

	m_parts_data.m_friction_coeffs.resize(old_parts_count);
	m_parts_data.m_friction_coeffs.reserve(new_parts_count);
	;
	for (const auto& cloth : clothes)
	{
		m_parts_data.m_friction_coeffs.insert(m_parts_data.m_friction_coeffs.end(),
			cloth.m_parts_data.m_friction_coeffs.begin(), cloth.m_parts_data.m_friction_coeffs.end());
	}

	m_parts_data.m_thicknesses.resize(old_parts_count);
	m_parts_data.m_thicknesses.reserve(new_parts_count);
	for (const auto& cloth : clothes)
	{
		m_parts_data.m_thicknesses.insert(m_parts_data.m_thicknesses.end(),
			cloth.m_parts_data.m_thicknesses.begin(), cloth.m_parts_data.m_thicknesses.end());
	}
}

glm::mat3x3 Cloth::elasticityTensor(const MaterialProperties& material_props) const
{
	// You can read about this in "Position-Based Simulation of Continuous Materials", p.6
	float division_coefficient = 1.0f / (1.0f - material_props.m_poisson_welt * material_props.m_poisson_warp);

	// TIP: second constructor parameter is [0][1]
	return {
		material_props.m_young_welt * division_coefficient, material_props.m_young_warp * material_props.m_poisson_welt * division_coefficient, 0.0f,
		material_props.m_young_welt * material_props.m_poisson_warp * division_coefficient, material_props.m_young_warp * division_coefficient, 0.0f,
		0.0f, 0.0f, material_props.m_shear_modulus };
}

glm::mat2x2 Cloth::triangleInverseShapeMatrix(int triangle_index) const
{
	// We need this for realistic stretch constraint. You can read about this
	// constraint in "Position-Based Simulation of Continuous Materials", p. 6.
	// The way we calc this matrix can be fond here
	// https://www.continuummechanics.org/finiteelementmapping.html

	const auto& triangle_vertices = m_vertices_data.m_indices[triangle_index];
	return glm::inverse(MathUtils::triangleShapeMatrix(m_vertices_data.m_coords[triangle_vertices[0]],
		m_vertices_data.m_coords[triangle_vertices[1]], m_vertices_data.m_coords[triangle_vertices[2]]));
}

std::array<int, CONSTRAINT_TYPES_COUNT> Cloth::predictInternalConstraintsCount(bool use_realistic_stretch, bool use_realistic_bending) const
{
	int bend_constraint_type = use_realistic_bending ? (size_t)ConstraintType::REALISTIC_BEND : (size_t)ConstraintType::BEND;
	std::array<int, CONSTRAINT_TYPES_COUNT> counters = { 0 };

	std::set<KeysUtils::ConstraintKey> visited_edges;
	KeysUtils::ConstraintKey key;

	for (const auto& triangle : m_vertices_data.m_indices)
	{
		counters[(size_t)ConstraintType::REALISTIC_STRETCH] += (int)use_realistic_stretch;

		// edge 1 (0,1)
		key = { std::min(triangle[1], triangle[0]), std::max(triangle[1], triangle[0]) };

		// if we have not visited this edge before, create stretch constraint
		if (!visited_edges.contains(key))
		{
			counters[(size_t)ConstraintType::STRETCH] += (int)(!use_realistic_stretch);
			visited_edges.insert(key);
		}
		// if we have visited this edge before, create bend constraint
		else
		{
			++counters[bend_constraint_type];
		}

		// edge 2 (0,2)
		key = { std::min(triangle[0], triangle[2]), std::max(triangle[0], triangle[2]) };

		// if we have not visited this edge before, create stretch constraint
		if (!visited_edges.contains(key))
		{
			counters[(size_t)ConstraintType::STRETCH] += (int)(!use_realistic_stretch);
			visited_edges.insert(key);
		}
		// if we have visited this edge before, create bend constraint
		else
		{
			++counters[bend_constraint_type];
		}

		// edge 3 (1,2)
		// the second index in edge must be larger then the first one
		key = { std::min(triangle[1], triangle[2]), std::max(triangle[1], triangle[2]) };

		// if we have not visited this edge before, create stretch constraint
		if (!visited_edges.contains(key))
		{
			counters[(size_t)ConstraintType::STRETCH] += (int)(!use_realistic_stretch);
			visited_edges.insert(key);
		}
		// if we have visited this edge before, create bend constraint
		else
		{
			++counters[bend_constraint_type];
		}
	}

	return counters;
}

void Cloth::setupInternalConstraints(const MaterialProperties& material_props, bool use_realistic_stretch, bool use_realistic_bending)
{
	int triangle_index = 0;

	// data for new constraint
	int new_constraint_id = 0;
	std::vector<KeysUtils::ConstraintKey> new_constraint_keys;
	std::vector<uint32_t> tmp_vertices_array;
	std::vector<float> tmp_data_array;

	std::map<KeysUtils::ConstraintKey, int> visited_edges;

	m_constraints_data.m_internal_constraints = ConstraintsBuffers::reserveBuffers(predictInternalConstraintsCount(use_realistic_stretch, use_realistic_bending));

	for (const auto& triangle : m_vertices_data.m_indices)
	{
		// if we use realistic stretch we should create stretch constraint for this triangle
		if (use_realistic_stretch)
		{
			createRealisticStretchConstraint(triangle_index, material_props.m_stretch_stiffness, tmp_vertices_array, tmp_data_array);
			m_constraints_data.m_internal_constraints.pushConstraint(ConstraintType::REALISTIC_STRETCH, tmp_vertices_array, tmp_data_array, false);

			new_constraint_keys = KeysUtils::generateConstraintKeys({ triangle[0], triangle[1], triangle[2] }, ConstraintType::REALISTIC_STRETCH);
			KeysUtils::insertKeysToMap(new_constraint_keys, new_constraint_id, m_constraints_data.m_internal_constraints_map);
			++new_constraint_id;
		}

		KeysUtils::ConstraintKey edges[3] =
		{
			{std::min(triangle[0], triangle[1]), std::max(triangle[0], triangle[1])},
			{std::min(triangle[0], triangle[2]), std::max(triangle[0], triangle[2])},
			{std::min(triangle[1], triangle[2]), std::max(triangle[1], triangle[2])} };

		for (const auto& edge : edges)
		{
			// we have not visited this edge before, create stretch constraint
			if (!visited_edges.contains(edge))
			{
				if (!use_realistic_stretch)
				{
					createStretchConstraint(edge[0], edge[1], material_props.m_stretch_stiffness, tmp_vertices_array, tmp_data_array);
					m_constraints_data.m_internal_constraints.pushConstraint(ConstraintType::STRETCH, tmp_vertices_array, tmp_data_array, false);

					new_constraint_keys = KeysUtils::generateConstraintKeys({ edge[0], edge[1] }, ConstraintType::STRETCH);
					KeysUtils::insertKeysToMap(new_constraint_keys, new_constraint_id, m_constraints_data.m_internal_constraints_map);
					++new_constraint_id;
				}

				visited_edges[edge] = triangle_index;
			}

			// we have visited this edge before, create bend constraint
			else
			{
				const int second_triangle_index = visited_edges[edge];

				if (use_realistic_bending)
				{
					createRealisticBendConstraint(triangle_index, second_triangle_index, edge, material_props.m_bending_stiffness, tmp_vertices_array, tmp_data_array);
					m_constraints_data.m_internal_constraints.pushConstraint(ConstraintType::REALISTIC_BEND, tmp_vertices_array, tmp_data_array, false);
					new_constraint_keys = KeysUtils::generateConstraintKeys(tmp_vertices_array, ConstraintType::REALISTIC_BEND);
				}
				else
				{
					createBendConstraint(triangle_index, second_triangle_index, edge, material_props.m_bending_stiffness, tmp_vertices_array, tmp_data_array);
					m_constraints_data.m_internal_constraints.pushConstraint(ConstraintType::BEND, tmp_vertices_array, tmp_data_array, false);
					new_constraint_keys = KeysUtils::generateConstraintKeys(tmp_vertices_array, ConstraintType::BEND);
				}

				KeysUtils::insertKeysToMap(new_constraint_keys, new_constraint_id, m_constraints_data.m_internal_constraints_map);
				++new_constraint_id;
			}
		}

		++triangle_index;
	}
}

void Cloth::createStretchConstraint(uint32_t vertex_a, uint32_t vertex_b, float stiffness, std::vector<uint32_t>& dst_vertices, std::vector<float>& dst_data) const
{
	dst_vertices.resize(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::STRETCH]);
	dst_data.resize(CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::STRETCH]);

	// STRETCH: 2 - we control distance between them
	dst_vertices = { std::min(vertex_a, vertex_b), std::max(vertex_a, vertex_b) };

	// STRETCH: 1 - recommended distance, 1 - opposite stiffness, 1 - lambda for XPBD
	dst_data[0] = edgeLength(vertex_a, vertex_b);
	dst_data[1] = 1.0f / stiffness;
	dst_data[2] = 0.0f;
}

void Cloth::createRealisticStretchConstraint(uint32_t triangle_index, float stiffness, std::vector<uint32_t>& dst_vertices, std::vector<float>& dst_data) const
{
	dst_vertices.resize(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH]);
	dst_data.resize(CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::REALISTIC_STRETCH]);

	// REALISTIC_STRETCH: 3 - triangle controlled by the constraint
	dst_vertices = { m_vertices_data.m_indices[triangle_index][0], m_vertices_data.m_indices[triangle_index][1], m_vertices_data.m_indices[triangle_index][2] };

	const glm::mat2x2 shape_matrix = triangleInverseShapeMatrix(triangle_index);

	// REALISTIC_STRETCH: 4 - to specify reference inverted shape matrix, 1 - reference triangle square, 1 - opposite stiffness, 1 - lambda for XPBD
	memcpy(dst_data.data(), glm::value_ptr(shape_matrix), 4 * sizeof(float));
	dst_data[4] = MathUtils::triangleSquare(
		m_vertices_data.m_coords[dst_vertices[0]],
		m_vertices_data.m_coords[dst_vertices[1]],
		m_vertices_data.m_coords[dst_vertices[2]]);
	dst_data[5] = 1.0f / stiffness;
	dst_data[6] = 0.0f;
}

void Cloth::createBendConstraint(uint32_t triangle_a, uint32_t triangle_b, const KeysUtils::ConstraintKey& common_edge,
	float bending_stiffness, std::vector<uint32_t>& dst_vertices, std::vector<float>& dst_data) const
{
	dst_vertices.resize(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::BEND]);
	dst_data.resize(CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::BEND]);

	const glm::uvec3& triangle_a_vertices = m_vertices_data.m_indices[triangle_a];
	const glm::uvec3& triangle_b_vertices = m_vertices_data.m_indices[triangle_b];

	uint32_t free_vertex_in_triangle_a = 0u;
	uint32_t free_vertex_in_triangle_b = 0u;
	for (int i = 0; i < 3; ++i)
	{
		if (triangle_a_vertices[i] != common_edge[0] && triangle_a_vertices[i] != common_edge[1])
		{
			free_vertex_in_triangle_a = triangle_a_vertices[i];
		}
		if (triangle_b_vertices[i] != common_edge[0] && triangle_b_vertices[i] != common_edge[1])
		{
			free_vertex_in_triangle_b = triangle_b_vertices[i];
		}
	}

	// Let the 'edge' array be [n,m]. We have two triangles with this edge as common. We also have arrays of
	// triangles vertices indices. One of these two arrays contains common edge indices in right order and the other one
	// contains common edge indices in reversed order. We should identify which of triangles contains common edge indices in right order.

	std::string triangle_a_indices_str = std::to_string(triangle_a_vertices[0]) + "," + std::to_string(triangle_a_vertices[1]) + "," + std::to_string(triangle_a_vertices[2]);
	triangle_a_indices_str += "," + triangle_a_indices_str;
	const std::string common_edge_indices_str = std::to_string(common_edge[0]) + "," + std::to_string(common_edge[1]);
	const bool triangle_a_contains_edge_indices_in_right_order = (triangle_a_indices_str.find(common_edge_indices_str) != std::string::npos);

	// Now we can create a right order of triangles indices to put them to the new constraint. What we want to get is:
	// 1) [0] and [1] are indices of common edge
	// 2) [2] and [3] are indices of free vertices of the triangles
	// 3) first triangle vertices are [1], [0], [2]
	// 4) second triangle vertices are [0], [1], [3]
	// 5) both triangles should go around counterclockwise

	// BEND: 4 - two triangles with common edge
	dst_vertices[0] = common_edge[(int)triangle_a_contains_edge_indices_in_right_order];
	dst_vertices[1] = common_edge[(int)(!triangle_a_contains_edge_indices_in_right_order)];
	dst_vertices[2] = free_vertex_in_triangle_a;
	dst_vertices[3] = free_vertex_in_triangle_b;

	const glm::vec3* vertices_coords[4] =
	{
		&m_vertices_data.m_coords[dst_vertices[0]],
		&m_vertices_data.m_coords[dst_vertices[1]],
		&m_vertices_data.m_coords[dst_vertices[2]],
		&m_vertices_data.m_coords[dst_vertices[3]] };

	const glm::vec3 vector_21 = *vertices_coords[0] - *vertices_coords[1];
	const glm::vec3 vector_31 = *vertices_coords[0] - *vertices_coords[2];
	const glm::vec3 vector_41 = *vertices_coords[0] - *vertices_coords[3];

	const glm::vec3 triangle_a_normal = glm::normalize(glm::cross(vector_21, vector_31));
	const glm::vec3 triangle_b_normal = glm::normalize(glm::cross(vector_21, vector_41));

	// create constraint data
	// BEND: 1 - initial angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
	dst_data[0] = MathUtils::safeAcos(glm::dot(triangle_a_normal, triangle_b_normal));
	dst_data[1] = 1.0f / bending_stiffness;
	dst_data[2] = 0.0f;
}

void Cloth::createRealisticBendConstraint(uint32_t triangle_a, uint32_t triangle_b, const KeysUtils::ConstraintKey& common_edge,
	float bending_stiffness, std::vector<uint32_t>& dst_vertices, std::vector<float>& dst_data) const
{
	dst_vertices.resize(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::REALISTIC_BEND]);
	dst_data.resize(CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::REALISTIC_BEND]);

	const glm::uvec3& triangle_a_vertices = m_vertices_data.m_indices[triangle_a];
	const glm::uvec3& triangle_b_vertices = m_vertices_data.m_indices[triangle_b];

	uint32_t free_vertex_in_triangle_a = 0u;
	uint32_t free_vertex_in_triangle_b = 0u;
	for (int i = 0; i < 3; ++i)
	{
		if (triangle_a_vertices[i] != common_edge[0] && triangle_a_vertices[i] != common_edge[1])
		{
			free_vertex_in_triangle_a = triangle_a_vertices[i];
		}
		if (triangle_b_vertices[i] != common_edge[0] && triangle_b_vertices[i] != common_edge[1])
		{
			free_vertex_in_triangle_b = triangle_b_vertices[i];
		}
	}

	// Let the 'edge' array be [n,m]. We have two triangles with this edge as common. We also have arrays of
	// triangles vertices indices. One of these two arrays contains common edge indices in right order and the other one
	// contains common edge indices in reversed order. We should identify which of triangles contains common edge indices in right order.

	std::string triangle_a_indices_str = std::to_string(triangle_a_vertices[0]) + "," + std::to_string(triangle_a_vertices[1]) + "," + std::to_string(triangle_a_vertices[2]);
	triangle_a_indices_str += "," + triangle_a_indices_str;
	const std::string common_edge_indices_str = std::to_string(common_edge[0]) + "," + std::to_string(common_edge[1]);
	const bool triangle_a_contains_edge_indices_in_right_order = (triangle_a_indices_str.find(common_edge_indices_str) != std::string::npos);

	// Now we can create a right order of triangles indices to put them to the new constraint. What we want to get is:
	// 1) [0] and [1] are indices of common edge
	// 2) [2] and [3] are indices of free vertices of the triangles
	// 3) first triangle vertices are [0], [1], [2]
	// 4) second triangle vertices are [1], [0], [3]
	// 5) both triangles should go around counterclockwise
	// REALISTIC_BEND: 4 - two triangles with common edge
	dst_vertices[0] = common_edge[(int)(!triangle_a_contains_edge_indices_in_right_order)];
	dst_vertices[1] = common_edge[(int)triangle_a_contains_edge_indices_in_right_order];
	dst_vertices[2] = free_vertex_in_triangle_a;
	dst_vertices[3] = free_vertex_in_triangle_b;

	// we need to calculate energy matrix for this constraint
	const auto energy_matrix = MathUtils::HessianEnergyMatrix(m_vertices_data.m_coords[dst_vertices[0]],
		m_vertices_data.m_coords[dst_vertices[1]], m_vertices_data.m_coords[dst_vertices[2]], m_vertices_data.m_coords[dst_vertices[3]]);

	// REALISTIC_BEND: 16 - Hessian energy matrix, 1 - opposite stiffness, 1 - lambda for XPBD
	memcpy(dst_data.data(), glm::value_ptr(energy_matrix), 16 * sizeof(float));
	dst_data[16] = 1.0f / bending_stiffness;
	dst_data[17] = 0.0f;
}