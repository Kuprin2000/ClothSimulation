#pragma once
#include <vector>
#include "indices_utils.h"
#include "math_utils.h"
#include "copy_utils.h"

class Collider
{
public:
	Collider() = delete;

	Collider(const Collider&) = delete;

	Collider(Collider&&) = default;

	Collider& operator=(Collider&&) = default;

	Collider(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, float friction_coefficient) : m_coords(coords), m_indices(indices)
	{
		m_normals.resize(m_coords.size(), { 0.0f, 0.0f, 0.0f });
		m_primitives_ownership = PrimitivesOwnershipUtils::generatePrimitivesOwnership(m_indices);
		MathUtils::calculateNormals(m_coords, m_indices, m_primitives_ownership, m_normals);
		m_borders.push_back((uint32_t)coords.size());
		m_friction_coefficients.push_back(friction_coefficient);
	}

	_NODISCARD int getVerticesCount() const
	{
		return (int)m_coords.size();
	}

	_NODISCARD int getTrianglesCount() const
	{
		return (int)m_indices.size();
	}

	_NODISCARD const glm::vec3& getCoords(int index) const
	{
		return m_coords[index];
	}

	_NODISCARD const std::vector<glm::vec3>& getCoords() const
	{
		return m_coords;
	}

	_NODISCARD std::array<const glm::vec3*, 3> getTriangleCoords(int index) const
	{
		const glm::uvec3& indices = m_indices[index];
		return {
			&m_coords[indices[0]],
			&m_coords[indices[1]],
			&m_coords[indices[2]] };
	}

	_NODISCARD const std::vector<glm::vec3>& getNormals() const
	{
		return m_normals;
	}

	_NODISCARD glm::uvec3 getIndices(int index) const
	{
		return m_indices[index];
	}

	_NODISCARD const std::vector<glm::uvec3>& getIndices() const
	{
		return m_indices;
	}

	_NODISCARD PrimitivesOwnershipUtils::TrianglePrimitivesOwnership getTrianglePrimitivesOwnership(int index) const
	{
		return PrimitivesOwnershipUtils::TrianglePrimitivesOwnership(m_primitives_ownership[index]);
	}

	_NODISCARD const std::vector<uint8_t>& getTrianglesPrimitivesOwnership() const
	{
		return m_primitives_ownership;
	}

	_NODISCARD int getVertexPartId(int index) const
	{
		int result = -1;

		for (int i = 0; i < m_borders.size(); ++i)
		{
			result += int(index < (int)m_borders[i]);
		}

		return result;
	}

	_NODISCARD float getPartFrictionCoefficient(int index) const
	{
		return m_friction_coefficients[index];
	}

	void pushColliders(const std::vector<Collider>& colliders);

private:
	std::vector<glm::vec3> m_coords;
	std::vector<glm::vec3> m_normals;
	std::vector<glm::uvec3> m_indices;
	std::vector<uint8_t> m_primitives_ownership;
	std::vector<int> m_borders;
	std::vector<float> m_friction_coefficients;
};