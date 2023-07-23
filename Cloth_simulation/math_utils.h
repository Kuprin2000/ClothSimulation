#pragma once
#include <vector>
#include <array>
#include <omp.h>
#include "primitives_ownership_utils.h"
#include "glm/glm.hpp"
#include "bool_vector.h"

namespace MathUtils
{
	static constexpr float PI = 3.14159265f;

	class FastSet
	{
	public:
		FastSet() = default;

		FastSet(int max_value)
		{
			m_has_value.resize((size_t)max_value + 1u);
		}

		FastSet(const std::vector<int>& values, int max_value): m_values(values)
		{
			m_has_value.resize((size_t)max_value + 1u);

			for (auto elem : values)
			{
				m_has_value.setValue(true, elem);
			}
		}

		_NODISCARD int getMaxValue() const
		{
			return (int)m_has_value.size();
		}

		_NODISCARD size_t size() const
		{
			return m_values.size();
		}

		void updateMaxValue(int new_value)
		{
			m_has_value.resize((size_t)new_value + 1u);
		}

		_NODISCARD const std::vector<int>& getValues() const
		{
			return m_values;
		}

		_NODISCARD bool contains(int value) const
		{
			return m_has_value.size() > value && m_has_value[value];
		}

		void insert(int value)
		{
			if (!m_has_value[value])
			{
				m_values.push_back(value);
				m_has_value.setValue(true, value);
			}
		}

		void insert(const std::vector<int>& values)
		{
			for (auto elem : values)
			{
				if (!m_has_value[elem])
				{
					m_values.push_back(elem);
					m_has_value.setValue(true, elem);
				}
			}
		}

		void erase(int value)
		{
			if (m_has_value[value])
			{
				m_values.erase(std::find(m_values.begin(), m_values.end(), value));
				m_has_value.setValue(false, value);
			}
		}

		void erase(const std::vector<int>& values)
		{
			for (auto elem : values)
			{
				if (m_has_value[elem])
				{
					m_values.erase(std::find(m_values.begin(), m_values.end(), elem));
					m_has_value.setValue(false, elem);
				}
			}
		}

		void clear()
		{
			m_values.clear();
			m_has_value.fill(false);
		}

	private:
		std::vector<int> m_values;
		BoolVector::BoolVector m_has_value;
	};

	_NODISCARD inline FastSet uniteSets(const FastSet& set_a, const FastSet& set_b)
	{
		const FastSet& max_set = (set_a.size() > set_b.size()) ? set_a : set_b;
		const FastSet& min_set = (set_a.size() > set_b.size()) ? set_b : set_a;

		FastSet result(max_set.getValues(), std::max(set_a.getMaxValue(), set_b.getMaxValue()));

		const std::vector<int> min_set_values = min_set.getValues();
		for (auto elem : min_set_values)
		{
			result.insert(elem);
		}

		return result;
	}

	_NODISCARD inline FastSet intersectSets(const FastSet& set_a, const FastSet& set_b)
	{
		const FastSet& max_set = (set_a.size() > set_b.size()) ? set_a : set_b;
		const FastSet& min_set = (set_a.size() > set_b.size()) ? set_b : set_a;

		FastSet result(std::max(set_a.getMaxValue(), set_b.getMaxValue()));

		const std::vector<int> min_set_values = min_set.getValues();
		for (auto elem : min_set_values)
		{
			if (max_set.contains(elem))
			{
				result.insert(elem);
			}
		}

		return result;
	}

	_NODISCARD inline FastSet subtractSet(const FastSet& set_a, const FastSet& set_b)
	{
		FastSet result(std::max(set_a.getMaxValue(), set_b.getMaxValue()));

		const std::vector<int> set_a_values = set_a.getValues();
		for (auto elem : set_a_values)
		{
			if (!set_b.contains(elem))
			{
				result.insert(elem);
			}
		}

		return result;
	}

	_NODISCARD inline glm::vec3 triangleNormal(const glm::vec3& vertex_1, const glm::vec3& vertex_2, const glm::vec3& vertex_3,
		bool need_to_normalize = true)
	{
		glm::vec3 normal = glm::cross(vertex_2 - vertex_1, vertex_3 - vertex_1);

		if (need_to_normalize)
		{
			normal = glm::normalize(normal);
		}

		return normal;
	}

	_NODISCARD inline std::array<glm::vec2, 3> triangleVerticesInLocalBasis(const glm::vec3& vertex_a, const glm::vec3& vertex_b,
		const glm::vec3& vertex_c, const glm::vec3& first_basis_vector, const glm::vec3& second_basis_vector)
	{
		// make all vertices to be near the origin
		glm::vec3 vertices_local_coords[3] =
		{
			{0.0f, 0.0f, 0.0f},
			vertex_b - vertex_a,
			vertex_c - vertex_a };

		std::array<glm::vec2, 3> vertices_coords_on_plane =
		{
			glm::vec2(glm::dot(vertices_local_coords[0], first_basis_vector), glm::dot(vertices_local_coords[0], second_basis_vector)),
			glm::vec2(glm::dot(vertices_local_coords[1], first_basis_vector), glm::dot(vertices_local_coords[1], second_basis_vector)),
			glm::vec2(glm::dot(vertices_local_coords[2], first_basis_vector), glm::dot(vertices_local_coords[2], second_basis_vector)) };

		return vertices_coords_on_plane;
	}

	_NODISCARD inline std::array<glm::vec2, 3> triangleVerticesInLocalBasis(const glm::vec3& vertex_a, const glm::vec3& vertex_b, const glm::vec3& vertex_c)
	{
		// create basis with the origin in vertex a
		const glm::vec3 triangle_normal = triangleNormal(vertex_a, vertex_b, vertex_c);
		const glm::vec3 first_basis_vector = glm::normalize(vertex_b - vertex_a);
		const glm::vec3 second_basis_vector = glm::cross(triangle_normal, first_basis_vector);

		return triangleVerticesInLocalBasis(vertex_a, vertex_b, vertex_c, first_basis_vector, second_basis_vector);
	}

	_NODISCARD inline float triangleSquare(const glm::vec3& vertex_1, const glm::vec3& vertex_2, const glm::vec3& vertex_3)
	{
		return 0.5f * glm::length(glm::cross(vertex_2 - vertex_1, vertex_3 - vertex_1));
	}

	_NODISCARD inline float safeAcos(float value)
	{
		if (value > 1.0f)
		{
			return acosf(1.0f);
		}
		if (value < -1.0f)
		{
			return acosf(-1.0f);
		}
		return acosf(value);
	}

	_NODISCARD inline float cotangentBetweenVectors(const glm::vec3& vector_a, const glm::vec3& vector_b)
	{
		// we should not care about normalization
		return glm::dot(vector_a, vector_b) / glm::length(glm::cross(vector_a, vector_b));
	}

	_NODISCARD glm::mat4x4 HessianEnergyMatrix(const glm::vec3& vertex_0,
		const glm::vec3& vertex_1, const glm::vec3& vertex_2, const glm::vec3& vertex_3);

	// took this from Christer Ericson "real-time collision detection" 2004
	_NODISCARD float distanceBetweenLinesSegments(const glm::vec3& segment_1_start, const glm::vec3& segment_1_end,
		const glm::vec3& segment_2_start, const glm::vec3& segment_2_end, float& param_1, float& param_2, glm::vec3& res_point_1, glm::vec3& res_point_2);

	// took this from Christer Ericson "real-time collision detection" 2004
	void closestPointOfTriangle(const glm::vec3& vertex, const glm::vec3& triangle_vertex_1, const glm::vec3& triangle_vertex_2,
		const glm::vec3& triangle_vertex_3, glm::vec3& dst_coords, glm::vec3& dst_barycentric_coords);

	_NODISCARD inline glm::vec3 interpolateBetweenThreeVectors(const glm::vec3& a_vector, const glm::vec3& b_vector, const glm::vec3& c_vector,
		const glm::vec3& barycentric_coords)
	{
		return { a_vector * barycentric_coords[0] + b_vector * barycentric_coords[1] + c_vector * barycentric_coords[2] };
	}

	_NODISCARD inline glm::vec3 vectorToVectorProjection(const glm::vec3& vector_to_project, const glm::vec3& vector_to_project_on)
	{
		return glm::dot(vector_to_project, vector_to_project_on) * vector_to_project_on;
	}

	// took this from Christer Ericson "real-time collision detection" 2004
	_NODISCARD inline float distanceFromPointToLineSegment(const glm::vec3& segment_start, const glm::vec3& segment_end, const glm::vec3& point)
	{
		const glm::vec3 ab = segment_end - segment_start;
		const glm::vec3 ac = point - segment_start;

		const float e = glm::dot(ac, ab);

		if (e <= 0.0f)
		{
			return glm::length(ac);
		}

		const float f = glm::dot(ab, ab);
		if (e >= f)
		{
			const glm::vec3 bc = point - segment_end;
			return glm::length(bc);
		}

		return sqrtf(glm::dot(ac, ac) - e * e / f);
	}

	inline void triangleLocalBasis(const glm::vec3& vertex_a_coords, const glm::vec3& vertex_b_coords,
		const glm::vec3& vertex_c_coords, glm::vec3& first_basis_vector, glm::vec3& second_basis_vector)
	{
		first_basis_vector = glm::normalize(vertex_b_coords - vertex_a_coords);
		second_basis_vector = glm::cross(triangleNormal(vertex_a_coords, vertex_b_coords, vertex_c_coords), first_basis_vector);
	}

	_NODISCARD inline float traceOfMatrix22(const glm::mat2x2& matrix)
	{
		return matrix[0][0] + matrix[1][1];
	}

	_NODISCARD inline glm::vec3 vectorFromLocalToGlobal(const glm::vec2& vector, const glm::vec3& first_basis_vector, const glm::vec3& second_basis_vector)
	{
		return vector[0] * first_basis_vector + vector[1] * second_basis_vector;
	}

	// direction_vector must be normalized
	_NODISCARD inline glm::vec3 closestPointOnLine(const glm::vec3& point, const glm::vec3& point_of_line_a, glm::vec3& direction_vector)
	{
		const float projection_on_line_length = glm::dot(point - point_of_line_a, direction_vector);
		return point_of_line_a + projection_on_line_length * direction_vector;
	}

	_NODISCARD inline float tripleProduct(const glm::vec3& vector_1, const glm::vec3& vector_2, const glm::vec3& vector_3)
	{
		return glm::dot(vector_1, glm::cross(vector_2, vector_3));
	}

	_NODISCARD inline float dihedralAngleBetweenTriangles(const glm::vec3& common_edge_vertex_1, const glm::vec3& common_edge_vertex_2,
		const glm::vec3& free_vertex_of_a, const glm::vec3& free_vertex_of_b)
	{
		const glm::vec3 normal_a = triangleNormal(common_edge_vertex_1, common_edge_vertex_2, free_vertex_of_a);
		const glm::vec3 normal_b = triangleNormal(common_edge_vertex_2, common_edge_vertex_1, free_vertex_of_b);

		return safeAcos(glm::dot(normal_a, normal_b));
	}

	_NODISCARD inline float signedDihedralAngleBetweenTriangles(const glm::vec3& vertex_1, const glm::vec3& vertex_2,
		const glm::vec3& vertex_3, const glm::vec3& vertex_4)
	{
		const glm::vec3& common_edge_vertex_1 = vertex_1;
		const glm::vec3& common_edge_vertex_2 = vertex_2;

		const glm::vec3& free_vertex_in_triangle_a = vertex_3;
		const glm::vec3& free_vertex_in_triangle_b = vertex_4;

		glm::vec3 edge_vector = glm::normalize(common_edge_vertex_2 - common_edge_vertex_1);

		const glm::vec3 closest_point_on_edge_for_free_vertex_a = closestPointOnLine(free_vertex_in_triangle_a, common_edge_vertex_1, edge_vector);
		const glm::vec3 closest_point_on_edge_for_free_vertex_b = closestPointOnLine(free_vertex_in_triangle_b, common_edge_vertex_1, edge_vector);

		const glm::vec3 shortest_vector_from_edge_to_free_vertex_a = free_vertex_in_triangle_a - closest_point_on_edge_for_free_vertex_a;
		const glm::vec3 shortest_vector_from_edge_to_free_vertex_b = free_vertex_in_triangle_b - closest_point_on_edge_for_free_vertex_b;

		const float result = dihedralAngleBetweenTriangles(vertex_1, vertex_2, vertex_3, vertex_4);

		const int angle_direction_indicator = (int)glm::sign(tripleProduct(shortest_vector_from_edge_to_free_vertex_a,
			shortest_vector_from_edge_to_free_vertex_b, edge_vector));

		return PI - result * (1.0f - 2.0f * angle_direction_indicator);
	}

	_NODISCARD inline glm::vec3 transformVectorWithQuat(const glm::vec3& vector, const glm::vec4& quat)
	{
		const float qx = quat[0];
		const float qy = quat[1];
		const float qz = quat[2];
		const float qw = quat[3];
		const float x = vector[0];
		const float y = vector[1];
		const float z = vector[2];

		float uvx = qy * z - qz * y;
		float uvy = qz * x - qx * z;
		float uvz = qx * y - qy * x;

		float uuvx = qy * uvz - qz * uvy;
		float uuvy = qz * uvx - qx * uvz;
		float uuvz = qx * uvy - qy * uvx;

		const float w2 = qw * 2.0f;
		uvx *= w2;
		uvy *= w2;
		uvz *= w2;

		uuvx *= 2.0f;
		uuvy *= 2.0f;
		uuvz *= 2.0f;

		return { x + uvx + uuvx, y + uvy + uuvy, z + uvz + uuvz };
	}

	// direction vector must be normalized
	_NODISCARD inline glm::vec3 rotateVectorAroundDirection(const glm::vec3& vector, const glm::vec3& direction, float angle)
	{
		const glm::vec4 quat =
		{ sinf(-angle * 0.5f) * direction[0], sinf(-angle * 0.5f) * direction[1], sinf(-angle * 0.5f) * direction[2], cosf(-angle * 0.5f) };

		return transformVectorWithQuat(vector, quat);
	}

	_NODISCARD inline bool floatToBool(float value)
	{
		return (fabsf(value) > FLT_EPSILON);
	}

	_NODISCARD inline glm::mat2x2 triangleShapeMatrix(const glm::vec3& vertex_a, const glm::vec3& vertex_b, const glm::vec3& vertex_c)
	{
		const std::array<glm::vec2, 3> vertices_coords_on_plane = MathUtils::triangleVerticesInLocalBasis(vertex_a, vertex_b, vertex_c);

		// TIP: second constructor parameter is [0][1]
		return
		{
			vertices_coords_on_plane[0][0] - vertices_coords_on_plane[2][0],
			vertices_coords_on_plane[0][1] - vertices_coords_on_plane[2][1],
			vertices_coords_on_plane[1][0] - vertices_coords_on_plane[2][0],
			vertices_coords_on_plane[1][1] - vertices_coords_on_plane[2][1]
		};
	}

	inline void calculateNormals(const AlignedVector::AlignedVector<glm::vec3>& coords, const AlignedVector::AlignedVector<glm::uvec3>& indices,
		const AlignedVector::AlignedVector<uint8_t>& primitives_ownership, AlignedVector::AlignedVector<glm::vec3>& normals)
	{
		for (int i = 0; i < normals.size(); ++i)
		{
			normals[i] = { 0.0f, 0.0f, 0.0f };
		}

		for (const auto& triangle : indices)
		{
			const glm::vec3 triangle_normal = triangleNormal(coords[triangle[0]], coords[triangle[1]], coords[triangle[2]]);

			normals[triangle[0]] += triangle_normal;
			normals[triangle[1]] += triangle_normal;
			normals[triangle[2]] += triangle_normal;
		}

		for (int i = 0; i < normals.size(); ++i)
		{
			const float length = glm::length(normals[i]);
			if (length > FLT_EPSILON)
			{
				normals[i] /= length;
			}
		}
	}

	inline int sign(int value)
	{
		return (0 < value) - (value < 0);
	}
}
