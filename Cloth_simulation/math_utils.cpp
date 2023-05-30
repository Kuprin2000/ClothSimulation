// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "math_utils.h"
#include <algorithm>

glm::mat4x4 MathUtils::HessianEnergyMatrix(const glm::vec3& vertex_0,
	const glm::vec3& vertex_1, const glm::vec3& vertex_2, const glm::vec3& vertex_3)
{
	const glm::vec3 vector_0 = vertex_1 - vertex_0;
	const glm::vec3 vector_1 = vertex_2 - vertex_0;
	const glm::vec3 vector_2 = vertex_3 - vertex_0;
	const glm::vec3 vector_3 = vertex_2 - vertex_1;
	const glm::vec3 vector_4 = vertex_3 - vertex_1;

	const float c_01 = cotangentBetweenVectors(vector_0, vector_1);
	const float c_02 = cotangentBetweenVectors(vector_0, vector_2);
	const float c_03 = cotangentBetweenVectors(-vector_0, vector_3);
	const float c_04 = cotangentBetweenVectors(-vector_0, vector_4);

	glm::vec4 vector_k =
	{
		c_03 + c_04,
		c_01 + c_02,
		-c_01 - c_03,
		-c_02 - c_04 };

	glm::mat4x4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result[j][i] = vector_k[i] * vector_k[j];
		}
	}

	return result * (3.0f / (triangleSquare(vertex_0, vertex_1, vertex_2) + triangleSquare(vertex_0, vertex_1, vertex_3)));
}

float MathUtils::distanceBetweenLinesSegments(const glm::vec3& segment_1_start, const glm::vec3& segment_1_end,
	const glm::vec3& segment_2_start, const glm::vec3& segment_2_end, float& param_1, float& param_2, glm::vec3& res_point_1, glm::vec3& res_point_2)
{
	const glm::vec3 d1 = segment_1_end - segment_1_start;
	const glm::vec3 d2 = segment_2_end - segment_2_start;
	const glm::vec3 r = segment_1_start - segment_2_start;
	const float a = glm::dot(d1, d1);
	const float e = glm::dot(d2, d2);
	const float f = glm::dot(d2, r);
	if (a <= FLT_EPSILON && e <= FLT_EPSILON)
	{
		param_1 = param_2 = 0.0f;
		res_point_1 = segment_1_start;
		res_point_2 = segment_2_start;
		return glm::length(res_point_1 - res_point_2);
	}
	if (a <= FLT_EPSILON)
	{
		param_1 = 0.0f;
		param_2 = f / e;
		param_2 = std::clamp(param_2, 0.0f, 1.0f);
	}
	else
	{
		const float c = glm::dot(d1, r);
		if (e <= FLT_EPSILON)
		{
			param_2 = 0.0f;
			param_1 = std::clamp(-c / a, 0.0f, 1.0f);
		}
		else
		{
			const float b = glm::dot(d1, d2);
			const float denom = a * e - b * b;

			if (fabsf(denom) > FLT_EPSILON)
			{
				param_1 = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}
			else
			{
				param_1 = 0.0f;
			}

			param_2 = (b * param_1 + f) / e;

			if (param_2 < 0.0f)
			{
				param_2 = 0.0f;
				param_1 = std::clamp(-c / a, 0.0f, 1.0f);
			}
			else if (param_2 > 1.0f) {
				param_2 = 1.0f;
				param_1 = std::clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	res_point_1 = segment_1_start + d1 * param_1;
	res_point_2 = segment_2_start + d2 * param_2;
	return glm::length(res_point_1 - res_point_2);
}

void MathUtils::closestPointOfTriangle(const glm::vec3& vertex, const glm::vec3& triangle_vertex_1, const glm::vec3& triangle_vertex_2,
	const glm::vec3& triangle_vertex_3, glm::vec3& dst_coords, glm::vec3& dst_barycentric_coords)
{
	const glm::vec3 ab = triangle_vertex_2 - triangle_vertex_1;
	const glm::vec3 ac = triangle_vertex_3 - triangle_vertex_1;
	const glm::vec3 ap = vertex - triangle_vertex_1;

	const float d1 = glm::dot(ab, ap);
	const float d2 = glm::dot(ac, ap);

	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		dst_coords = triangle_vertex_1;
		dst_barycentric_coords = { 1.0f, 0.0f, 0.0f };
		return;
	}

	const glm::vec3 bp = vertex - triangle_vertex_2;
	const float d3 = glm::dot(ab, bp);
	const float d4 = glm::dot(ac, bp);

	if (d3 >= 0.0f && d4 <= d3)
	{
		dst_coords = triangle_vertex_2;
		dst_barycentric_coords = { 0.0f, 1.0f, 0.0f };
		return;
	}

	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		const float v = d1 / (d1 - d3);
		dst_coords = triangle_vertex_1 + v * ab;
		dst_barycentric_coords = { 1 - v, v, 0.0f };
		return;
	}

	const glm::vec3 cp = vertex - triangle_vertex_3;
	const float d5 = glm::dot(ab, cp);
	const float d6 = glm::dot(ac, cp);

	if (d6 >= 0.0f && d5 <= d6)
	{
		dst_coords = triangle_vertex_3;
		dst_barycentric_coords = { 0.0f, 0.0f, 1.0f };
		return;
	}

	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		const float w = d2 / (d2 - d6);
		dst_coords = triangle_vertex_1 + w * ac;
		dst_barycentric_coords = { 1.0f - w, 0.0f, w };
		return;
	}

	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		dst_coords = triangle_vertex_2 + w * (triangle_vertex_3 - triangle_vertex_2);
		dst_barycentric_coords = { 0.0f, 1.0f - w, w };
		return;
	}

	const float denom = 1.0f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;

	dst_coords = triangle_vertex_1 + v * ab + w * ac;
	dst_barycentric_coords = { 1.0f - v - w, v, w };
}