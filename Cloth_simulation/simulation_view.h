#pragma once
#include "math_utils.h"

enum class DrawMode : int
{
	FILL,
	POINTS,
	LINES
};

struct DrawParameters
{
	DrawMode m_cloth_draw_mode = DrawMode::FILL;
	DrawMode m_collider_draw_mode = DrawMode::FILL;
	DrawMode m_candidate_draw_mode = DrawMode::FILL;

	glm::vec3 m_background_color = { 0.9f, 0.9f, 0.9f };
	glm::vec3 m_cloth_color = { 0.8f, 0.8f, 0.8f };
	glm::vec3 m_collider_color = { 0.7f, 0.7f, 0.7f };
	glm::vec3 m_candidate_color = { 0.5f, 0.5f, 0.5f };
	glm::vec3 m_vertex_to_mark_color = { 1.0f, 0.0f, 0.0f };

	glm::vec3 m_camera_position = { 0.0f, 0.0f, 200.0f };
	glm::vec3 m_camera_up = { 0.0f, 1.0f, 0.0f };
	float m_camera_pitch = 0.0f;
	float m_camera_yaw = -MathUtils::PI * 0.5f;
	float m_camera_roll = 0.0f;
	float m_camera_fov = 45.0f;

	glm::vec3 m_light_source_position = { 0.0f, 200.0f, 200.0f };
	glm::vec3 m_light_source_color = { 1.0f, 1.0f, 1.0f };
	float m_light_source_intensity = 0.8f;
	float m_ambient_light_intensity = 0.8f;

	float m_cloth_specular_intensity = 0.2f;
	float m_collider_specular_intensity = 0.2f;
	float m_candidate_specular_intensity = 0.2f;

	float m_aspect_ratio = 16.0f / 9.0f;

	int m_vertices_to_mark[2] = { 0 };

	float m_points_size = 5.0f;
};

class SimulationView
{
public:
	SimulationView();

	_NODISCARD DrawParameters& getDrawParameters()
	{
		return m_draw_parameters;
	}

	void newFrameStart(float aspect_ratio);

	void render_cloth(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, const std::vector<glm::vec3>& normals, bool mark_vertices) const
	{
		setUniforms(m_draw_parameters.m_cloth_color, m_draw_parameters.m_cloth_specular_intensity, true, mark_vertices);
		renderInternal((int)normals.size(), (int)indices.size(), coords, indices, normals, m_draw_parameters.m_cloth_draw_mode, m_cloth_vbo, m_cloth_ebo);
	}

	void render_cloth(int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::vec3>& normals) const
	{
		setUniforms(m_draw_parameters.m_cloth_color, m_draw_parameters.m_cloth_specular_intensity, true, false);
		renderInternal((int)normals.size(), triangles_count, coords, normals, m_draw_parameters.m_cloth_draw_mode, m_cloth_vbo, m_cloth_ebo);
	}

	void render_collider(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, const std::vector<glm::vec3>& normals) const
	{
		setUniforms(m_draw_parameters.m_collider_color, m_draw_parameters.m_collider_specular_intensity, true, false);
		renderInternal((int)coords.size(), (int)indices.size(), coords, indices, normals, m_draw_parameters.m_collider_draw_mode, m_collider_vbo, m_collider_ebo);
	}

	void render_collider(int vertices_count, int triangles_count) const
	{
		setUniforms(m_draw_parameters.m_collider_color, m_draw_parameters.m_collider_specular_intensity, true, false);
		renderInternal(vertices_count, triangles_count, m_draw_parameters.m_collider_draw_mode, m_collider_vbo, m_collider_ebo);
	}

	void render_candidate_object(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices) const
	{
		setUniforms(m_draw_parameters.m_candidate_color, m_draw_parameters.m_candidate_specular_intensity, false, false);
		renderInternal((int)coords.size(), (int)indices.size(), coords, indices, m_draw_parameters.m_candidate_draw_mode, m_candidate_vbo, m_candidate_ebo);
	}

private:
	void setUniforms(const glm::vec3& color, float specular_intensity, bool use_normals, bool mark_vertices) const;

	void renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices,
		const std::vector<glm::vec3>& normals, DrawMode draw_mode, int vbo, int ebo) const;

	void renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::vec3>& normals, DrawMode draw_mode, int vbo, int ebo) const;

	void renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, DrawMode draw_mode, int vbo, int ebo) const;

	void renderInternal(int vertices_count, int triangles_count, DrawMode draw_mode, int vbo, int ebo) const;

private:
	unsigned int m_vao = 0u;

	unsigned int m_cloth_vbo = 0u;
	unsigned int m_cloth_ebo = 0u;

	unsigned int m_collider_vbo = 0u;
	unsigned int m_collider_ebo = 0u;

	unsigned int m_candidate_vbo = 0u;
	unsigned int m_candidate_ebo = 0u;

	int m_color_location = 0;
	int m_marked_vertex_color_location = 0;
	int m_view_matrix_location = 0;
	int m_projection_matrix_location = 0;
	int m_light_source_position_location = 0;
	int m_light_source_color_location = 0;
	int m_intensities_location = 0;				// light source intensity, ambient light intensity, material specular intensity
	int m_camera_position_location = 0;
	int m_vertices_to_mark_location = 0;
	int m_flags_location = 0;					// use_normals, mark_vertices
	int m_points_size_location = 0;

	int m_shader_program = 0;

	DrawParameters m_draw_parameters;
};
