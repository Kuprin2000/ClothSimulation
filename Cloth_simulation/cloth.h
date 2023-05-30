#pragma once
#include "math_utils.h"
#include "constraints_buffers.h"
#include "cloth_constraints.h"

// material properties
struct MaterialProperties
{
	float m_stretch_stiffness = 500.0f;		// stiffness of not realistic stretch constraint
	float m_young_welt = 20.0f;				// Young modules at two directions
	float m_young_warp = 20.0f;
	float m_poisson_welt = 0.1f;			// Poisson coefficients at two directions
	float m_poisson_warp = 0.1f;
	float m_shear_modulus = 0.2f;			// shear module
	float m_friction_coeff = 0.5f;			// friction parameter
	float m_thickness = 0.2f;				// cloth thickness
	float m_bending_stiffness = 500.0f;		// bending stiffness
	float m_density = 1.0f;					// density of cloth (kilograms per square meter)
};

struct PhantomVerticesDescriptor
{
	int m_original_vertex = 0u;
	int m_first_phantom_index = 0u;
	uint8_t m_phantoms_count = 0u;
};

class Cloth
{
public:
	Cloth() = delete;

	Cloth(const Cloth&) = delete;

	Cloth(Cloth&&) = default;

	Cloth& operator=(Cloth&&) = default;

	Cloth(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, const std::vector<float>& opposite_masses,
		const MaterialProperties& material_props, int layer_number, bool use_realistic_stretch, bool use_realistic_bending);

	// get count

	_NODISCARD int getRealVerticesCount() const
	{
		return (int)m_vertices_data.m_opposite_masses.size();
	}

	_NODISCARD int getPhantomVerticesCount() const
	{
		return (int)(m_vertices_data.m_coords.size() - m_vertices_data.m_opposite_masses.size());
	}

	_NODISCARD int getTrianglesCount() const
	{
		return (int)m_vertices_data.m_indices.size();
	}

	_NODISCARD int getPartsCount() const
	{
		return m_vertices_data.m_coords.empty() ? 0 : (int)m_parts_data.m_borders.size();
	}

	_NODISCARD int getInternalConstraintsCount() const
	{
		return m_constraints_data.m_internal_constraints.getConstraintsCount();
	}

	// setters and getters

	_NODISCARD const glm::vec3& getCoords(int index) const
	{
		return m_vertices_data.m_coords[index];
	}

	_NODISCARD const glm::vec3& getTestCoords(int index) const
	{
		return m_vertices_data.m_test_coords[index];
	}

	_NODISCARD glm::vec3& getTestCoords(int index)
	{
		return m_vertices_data.m_test_coords[index];
	}

	_NODISCARD std::array<const glm::vec3*, 3> getTriangleCoords(int index) const
	{
		const glm::uvec3& vertices_indices = m_vertices_data.m_indices[index];
		return {
			&m_vertices_data.m_coords[vertices_indices[0]],
			&m_vertices_data.m_coords[vertices_indices[1]],
			&m_vertices_data.m_coords[vertices_indices[2]] };
	}

	_NODISCARD std::vector<glm::vec3>& getCoords()
	{
		return m_vertices_data.m_coords;
	}

	_NODISCARD const std::vector<glm::vec3>& getCoords() const
	{
		return m_vertices_data.m_coords;
	}

	_NODISCARD std::array<const glm::vec3*, 3> getTriangleTestCoords(int index) const
	{
		const glm::uvec3& vertices_indices = m_vertices_data.m_indices[index];
		return {
			&m_vertices_data.m_test_coords[vertices_indices[0]],
			&m_vertices_data.m_test_coords[vertices_indices[1]],
			&m_vertices_data.m_test_coords[vertices_indices[2]] };
	}

	_NODISCARD std::vector<glm::vec3>& getTestCoords()
	{
		return m_vertices_data.m_test_coords;
	}

	void setTestCoords(const glm::vec3& coords, int index)
	{
		m_vertices_data.m_test_coords[index] = coords;
	}

	void setTestCoords(const std::vector<glm::vec3>& test_coords)
	{
		m_vertices_data.m_test_coords = test_coords;
	}

	_NODISCARD const glm::uvec3& getIndices(int index) const
	{
		return m_vertices_data.m_indices[index];
	}

	_NODISCARD const std::vector<glm::uvec3>& getIndices() const
	{
		return m_vertices_data.m_indices;
	}

	_NODISCARD const glm::vec3& getNormal(int index) const
	{
		return m_vertices_data.m_normals[index];
	}

	_NODISCARD const std::vector<glm::vec3>& getNormals() const
	{
		return m_vertices_data.m_normals;
	}

	void setNormals(const std::vector<glm::vec3>& normals)
	{
		m_vertices_data.m_normals = normals;
	}

	_NODISCARD float getOppositeMass(int index) const
	{
		return m_vertices_data.m_opposite_masses[index];
	}

	_NODISCARD const std::vector<float>& getOppositeMasses() const
	{
		return m_vertices_data.m_opposite_masses;
	}


	_NODISCARD const glm::vec3& getSpeed(int index) const
	{
		return m_vertices_data.m_speeds[index];
	}

	_NODISCARD std::vector<glm::vec3>& getSpeeds()
	{
		return m_vertices_data.m_speeds;
	}

	_NODISCARD uint32_t getHostOrOriginalVertex(int index) const
	{
		return m_vertices_data.m_host_and_original_vertices[index];
	}

	_NODISCARD std::vector<uint32_t>& getHostOrOriginalVertices()
	{
		return m_vertices_data.m_host_and_original_vertices;
	}

	void setHostOrOriginalVertex(int host_vertex, int index)
	{
		m_vertices_data.m_host_and_original_vertices[index] = host_vertex;
	}

	_NODISCARD PrimitivesOwnershipUtils::TrianglePrimitivesOwnership getTrianglePrimitivesOwnership(int index) const
	{
		return PrimitivesOwnershipUtils::TrianglePrimitivesOwnership(m_vertices_data.m_primitives_ownership[index]);
	}

	_NODISCARD const std::vector<uint8_t>& getTrianglesPrimitivesOwnership() const
	{
		return m_vertices_data.m_primitives_ownership;
	}

	_NODISCARD ConstraintsBuffers& getInternalConstraints()
	{
		return m_constraints_data.m_internal_constraints;
	}

	_NODISCARD const std::map<KeysUtils::ConstraintKey, std::vector<int>>& getConstraintSearchMap() const
	{
		return m_constraints_data.m_internal_constraints_map;
	}

	_NODISCARD int getVertexPartId(int index) const
	{
		index = m_vertices_data.m_host_and_original_vertices[index];
		int result = -1;

		for (int i = 0; i < m_parts_data.m_borders.size(); ++i)
		{
			result += int(index < m_parts_data.m_borders[i]);
		}

		return result;
	}

	_NODISCARD uint8_t getPartLayerNumber(int index) const
	{
		return m_parts_data.m_layers_numbers[index];
	}

	_NODISCARD const glm::mat3x3& getPartElasticityTensor(int index) const
	{
		return m_parts_data.m_elasticity_tensors[index];
	}

	_NODISCARD float getPartFrictionCoeff(int index) const
	{
		return m_parts_data.m_friction_coeffs[index];
	}


	_NODISCARD float getPartThickness(int index) const
	{
		return m_parts_data.m_thicknesses[index];
	}

	// add part

	void pushClothes(const std::vector<Cloth>& parts_to_add);

	// synchronization

	void syncSlaveVertices()
	{
		const int real_vertices_count = getRealVerticesCount();
		for (int i = 0; i < real_vertices_count; ++i)
		{
			m_vertices_data.m_coords[i] = m_vertices_data.m_coords[m_vertices_data.m_host_and_original_vertices[i]];
			m_vertices_data.m_test_coords[i] = m_vertices_data.m_test_coords[m_vertices_data.m_host_and_original_vertices[i]];
		}
	}

	void syncOriginalsVertices()
	{
		const int real_vertices_count = getRealVerticesCount();
		for (int i = real_vertices_count; i < m_vertices_data.m_host_and_original_vertices.size(); ++i)
		{
			m_vertices_data.m_coords[m_vertices_data.m_host_and_original_vertices[i]] = m_vertices_data.m_coords[i];
			m_vertices_data.m_test_coords[m_vertices_data.m_host_and_original_vertices[i]] = m_vertices_data.m_test_coords[i];
		}
	}

	void syncPhantomVertices()
	{
		const int real_vertices_count = getRealVerticesCount();
		for (int i = real_vertices_count; i < m_vertices_data.m_host_and_original_vertices.size(); ++i)
		{
			m_vertices_data.m_coords[i] = m_vertices_data.m_coords[m_vertices_data.m_host_and_original_vertices[i]];
			m_vertices_data.m_test_coords[i] = m_vertices_data.m_test_coords[m_vertices_data.m_host_and_original_vertices[i]];
		}
	}

	// update normals

	void updateNormals()
	{
		MathUtils::calculateNormals(m_vertices_data.m_coords, m_vertices_data.m_indices,
			m_vertices_data.m_primitives_ownership, m_vertices_data.m_normals);
	}

	// phantom vertices
	void setPhantomVertices(const std::vector<PhantomVerticesDescriptor>& replaced_vertices, int phantom_vertices_count)
	{
		// m_constraints_data is already updated so we have to update only m_vertices_data

		const int total_vertices_count = getRealVerticesCount() + phantom_vertices_count;

		m_vertices_data.m_host_and_original_vertices.resize(total_vertices_count);
		for (const auto& replaced_vertex : replaced_vertices)
		{
			for (int i = replaced_vertex.m_first_phantom_index; i < replaced_vertex.m_first_phantom_index + replaced_vertex.m_phantoms_count; ++i)
			{
				m_vertices_data.m_host_and_original_vertices[i] = replaced_vertex.m_original_vertex;
			}
		}

		m_vertices_data.m_coords.resize(total_vertices_count);
		m_vertices_data.m_test_coords.resize(total_vertices_count);
		const int real_vertices_count = getRealVerticesCount();
		for (int i = real_vertices_count; i < m_vertices_data.m_host_and_original_vertices.size(); ++i)
		{
			m_vertices_data.m_coords[i] = m_vertices_data.m_coords[m_vertices_data.m_host_and_original_vertices[i]];
			m_vertices_data.m_test_coords[i] = m_vertices_data.m_test_coords[m_vertices_data.m_host_and_original_vertices[i]];
		}
	}

	void replacePhantomsWithOriginals(std::vector<uint32_t>& vertices) const
	{
		const int real_vertices_count = getRealVerticesCount();
		for (auto& vertex : vertices)
		{
			vertex = ((int)vertex >= real_vertices_count) ? m_vertices_data.m_host_and_original_vertices[vertex] : vertex;
		}
	}

private:
	_NODISCARD glm::mat3x3 elasticityTensor(const MaterialProperties& material_props) const;

	_NODISCARD glm::mat2x2 triangleInverseShapeMatrix(int index) const;

	_NODISCARD std::array<int, CONSTRAINT_TYPES_COUNT> predictInternalConstraintsCount(bool use_realistic_stretch, bool use_realistic_bending) const;

	void setupInternalConstraints(const MaterialProperties& material_props, bool use_realistic_stretch, bool use_realistic_bending);

	_NODISCARD float edgeLength(int vertex_a, int vertex_b) const
	{
		return glm::distance(m_vertices_data.m_coords[vertex_a], m_vertices_data.m_coords[vertex_b]);
	}

	void createStretchConstraint(uint32_t vertex_a, uint32_t vertex_b, float stiffness,
		std::vector<uint32_t>& vertices, std::vector<float>& data) const;

	void createRealisticStretchConstraint(uint32_t triangle_index, float stiffness,
		std::vector<uint32_t>& _vertices, std::vector<float>& data) const;

	void createBendConstraint(uint32_t triangle_a, uint32_t triangle_b, const KeysUtils::ConstraintKey& common_edge, float bending_stiffness,
		std::vector<uint32_t>& vertices, std::vector<float>& data) const;

	void createRealisticBendConstraint(uint32_t triangle_a, uint32_t triangle_b, const KeysUtils::ConstraintKey& common_edge, float bending_stiffness,
		std::vector<uint32_t>& vertices, std::vector<float>& data) const;

private:
	struct
	{
		std::vector<glm::vec3> m_coords;	   // coordinates of real and phantom vertices
		std::vector<glm::vec3> m_test_coords; // test coordinates of real and phantom vertices
		std::vector<glm::uvec3> m_indices;	   // indices of real vertices
		std::vector<glm::vec3> m_normals;	   // normals of real vertices

		std::vector<float> m_opposite_masses; // opposite masses of real vertices
		std::vector<glm::vec3> m_speeds;	  // speeds of real vertices

		// host of of real and phantom vertices
		// 1) for real vertices we store:
		// a) it's own index if it has no host
		// b) it's host's index
		// 2) for phantom vertex we store original vertex index
		std::vector<uint32_t> m_host_and_original_vertices;

		std::vector<uint8_t> m_primitives_ownership; // shows which vertices and edges each triangle owns

	} m_vertices_data;

	struct
	{
		ConstraintsBuffers m_internal_constraints;										 // internal constraints of the cloth
		std::map<KeysUtils::ConstraintKey, std::vector<int>> m_internal_constraints_map; // map to search constraints
	} m_constraints_data;

	struct
	{
		std::vector<int> m_borders;			   // array to identify which part vertex belongs to
		std::vector<uint8_t> m_layers_numbers; // array to identify layer number of each vertex group

		std::vector<glm::mat3x3> m_elasticity_tensors; // elasticity tensors of cloth parts
		std::vector<float> m_bending_stiffness;			// bending stiffness of the cloth parts
		std::vector<float> m_friction_coeffs;			// friction coefficients
		std::vector<float> m_thicknesses;				// concatenated thicknesses of cloth parts
	} m_parts_data;
};
