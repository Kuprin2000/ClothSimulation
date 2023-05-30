#pragma once
#include <cmath>
#include <algorithm>
#include "glm/glm.hpp"

class BoundingBox
{
public:
	void setVoid()
	{
		m_data[0] = 1.0f;
		m_data[1] = -1.0f;
		m_data[2] = 1.0f;
		m_data[3] = -1.0f;
		m_data[4] = 1.0f;
		m_data[5] = -1.0f;
		m_data[6] = 0.0f;
	}

	void setVertex(const glm::vec3& coords, float gap)
	{
		m_data[0] = coords[0] - gap;
		m_data[1] = coords[0] + gap;
		m_data[2] = coords[1] - gap;
		m_data[3] = coords[1] + gap;
		m_data[4] = coords[2] - gap;
		m_data[5] = coords[2] + gap;
		m_data[6] = gap;
	}

	void setMovingVertex(const glm::vec3& old_coords, const glm::vec3& new_coords, float gap)
	{
		m_data[0] = fmin(old_coords[0], new_coords[0]) - gap;
		m_data[1] = fmax(old_coords[0], new_coords[0]) + gap;
		m_data[2] = fmin(old_coords[1], new_coords[1]) - gap;
		m_data[3] = fmax(old_coords[1], new_coords[1]) + gap;
		m_data[4] = fmin(old_coords[2], new_coords[2]) - gap;
		m_data[5] = fmax(old_coords[2], new_coords[2]) + gap;
		m_data[6] = gap;
	}

	void setTriangle(const glm::vec3* vertices_coords, float gap)
	{
		m_data[0] = std::min({ vertices_coords[0][0], vertices_coords[1][0], vertices_coords[2][0] }) - gap;
		m_data[1] = std::max({ vertices_coords[0][0], vertices_coords[1][0], vertices_coords[2][0] }) + gap;
		m_data[2] = std::min({ vertices_coords[0][1], vertices_coords[1][1], vertices_coords[2][1] }) - gap;
		m_data[3] = std::max({ vertices_coords[0][1], vertices_coords[1][1], vertices_coords[2][1] }) + gap;
		m_data[4] = std::min({ vertices_coords[0][2], vertices_coords[1][2], vertices_coords[2][2] }) - gap;
		m_data[5] = std::max({ vertices_coords[0][2], vertices_coords[1][2], vertices_coords[2][2] }) + gap;
		m_data[6] = gap;
	}

	void setMovingTriangle(const glm::vec3* old_coords, const glm::vec3* new_coords, float gap)
	{
		m_data[0] = std::min({ old_coords[0][0], old_coords[1][0], old_coords[2][0], new_coords[0][0], new_coords[1][0], new_coords[2][0] }) - gap;
		m_data[1] = std::max({ old_coords[0][0], old_coords[1][0], old_coords[2][0], new_coords[0][0], new_coords[1][0], new_coords[2][0] }) + gap;
		m_data[2] = std::min({ old_coords[0][1], old_coords[1][1], old_coords[2][1], new_coords[0][1], new_coords[1][1], new_coords[2][1] }) - gap;
		m_data[3] = std::max({ old_coords[0][1], old_coords[1][1], old_coords[2][1], new_coords[0][1], new_coords[1][1], new_coords[2][1] }) + gap;
		m_data[4] = std::min({ old_coords[0][2], old_coords[1][2], old_coords[2][2], new_coords[0][2], new_coords[1][2], new_coords[2][2] }) - gap;
		m_data[5] = std::max({ old_coords[0][2], old_coords[1][2], old_coords[2][2], new_coords[0][2], new_coords[1][2], new_coords[2][2] }) + gap;
		m_data[6] = gap;
	}

	void addBndBox(const BoundingBox& bnd_box)
	{
		m_data[0] = (bnd_box.m_data[0] < m_data[0]) ? bnd_box.m_data[0] : m_data[0];
		m_data[1] = (bnd_box.m_data[1] > m_data[1]) ? bnd_box.m_data[1] : m_data[1];
		m_data[2] = (bnd_box.m_data[2] < m_data[2]) ? bnd_box.m_data[2] : m_data[2];
		m_data[3] = (bnd_box.m_data[3] > m_data[3]) ? bnd_box.m_data[3] : m_data[3];
		m_data[4] = (bnd_box.m_data[4] < m_data[4]) ? bnd_box.m_data[4] : m_data[4];
		m_data[5] = (bnd_box.m_data[5] > m_data[5]) ? bnd_box.m_data[5] : m_data[5];
	}

	_NODISCARD bool contains(const glm::vec3& point_coords) const
	{
		return point_coords[0] >= m_data[0] &&
			point_coords[0] <= m_data[1] &&
			point_coords[1] >= m_data[2] &&
			point_coords[1] <= m_data[3] &&
			point_coords[2] >= m_data[4] &&
			point_coords[2] <= m_data[5];
	}

	_NODISCARD float getGap() const
	{
		return m_data[6];
	}

	_NODISCARD float getVolume() const
	{
		return (m_data[1] - m_data[0]) * (m_data[3] - m_data[2]) * (m_data[5] - m_data[4]);
	}

	_NODISCARD static float calcBndBoxesOverlap(const BoundingBox& box_a, const BoundingBox& box_b)
	{
		float overlap_dimensions[3];

		overlap_dimensions[0] = fmin(box_a.m_data[1], box_b.m_data[1]) - fmax(box_a.m_data[0], box_b.m_data[0]);
		if (overlap_dimensions[0] <= 0.0f)
		{
			return 0.0f;
		}

		overlap_dimensions[1] = fmin(box_a.m_data[3], box_b.m_data[3]) - fmax(box_a.m_data[2], box_b.m_data[2]);
		if (overlap_dimensions[1] <= 0.0f)
		{
			return 0.0f;
		}

		overlap_dimensions[2] = fmin(box_a.m_data[5], box_b.m_data[5]) - fmax(box_a.m_data[4], box_b.m_data[4]);
		if (overlap_dimensions[2] <= 0.0f)
		{
			return 0.0f;
		}

		return overlap_dimensions[0] * overlap_dimensions[1] * overlap_dimensions[2];
	}

	_NODISCARD static bool thereIsOVerlap(const BoundingBox& box_a, const BoundingBox& box_b)
	{
		if (box_a.m_data[1] < box_b.m_data[0] || box_a.m_data[0] > box_b.m_data[1])
		{
			return false;
		}

		if (box_a.m_data[3] < box_b.m_data[2] || box_a.m_data[2] > box_b.m_data[3])
		{
			return false;
		}

		if (box_a.m_data[5] < box_b.m_data[4] || box_a.m_data[4] > box_b.m_data[5])
		{
			return false;
		}

		return true;
	}

private:
	// x_min, x_max, y_min, y_max, z_min, z_max, gap, volume
	float m_data[7] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f };
};
