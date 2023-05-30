// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "simulation_view.h"
#include "glad/glad.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

SimulationView::SimulationView()
{
	// create vertex shader
	const char* vertex_shader_source =
		"#version 330 core\n"
		"layout (location = 0) in vec3 coords;\n"
		"layout (location = 1) in vec3 normals;\n"
		"uniform mat4 view_mat;\n"
		"uniform mat4 projection_mat;\n"
		"uniform ivec2 vertices_to_mark;\n"
		"uniform vec2 flags;\n"
		"uniform float points_size;\n"
		"out vec3 fragment_coords;\n"
		"out vec3 normal;\n"
		"out float need_mark;\n"
		"void main()\n"
		"{\n"
		"   gl_Position = projection_mat * view_mat * vec4(coords.x, coords.y, coords.z, 1.0f);\n"
		"	gl_PointSize = points_size;"
		"	fragment_coords = coords;\n"
		"	normal = normals;\n"
		" 	need_mark = float((gl_VertexID == vertices_to_mark.x) || (gl_VertexID == vertices_to_mark.y));\n"
		" 	need_mark = need_mark * float(abs(flags.y) > 0.001f);\n"
		"}\0";

	unsigned int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
	glCompileShader(vertex_shader);
	int success;
	char infoLog[512];
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
		throw std::exception("Vertex shader compilation failed");
	}

	// create fragment shader
	const char* fragment_shader_source =
		"#version 330 core\n"
		"uniform vec4 vertex_color;\n"
		"uniform vec4 marked_vertex_color;\n"
		"uniform vec3 light_source_position;\n"
		"uniform vec4 light_source_color;\n"
		"uniform vec3 intensities;\n"
		"uniform vec2 flags;\n"
		"uniform vec3 camera_position;\n"
		"in vec3 normal;\n"
		"in vec3 fragment_coords;\n"
		"in float need_mark;\n"
		"out vec4 fragment_color;\n"
		"void main()\n"
		"{\n"
		"	vec3 light_direction = normalize(light_source_position - fragment_coords);\n"
		"	vec3 normalized_normal = normalize(normal) * (-1.0f + 2 * float(gl_FrontFacing));\n"
		"	float diffuse = max(dot(normalized_normal, light_direction), 0.0f);\n"
		"	vec3 view_direction = normalize(camera_position - fragment_coords);\n"
		"	vec3 reflection_direction = reflect(-light_direction, normalized_normal);\n"
		"	float specular = intensities.z * pow(max(dot(view_direction, reflection_direction), 0.0f), 32);\n"
		"	fragment_color = (intensities.y + diffuse + specular) * intensities.x * light_source_color * vertex_color;\n"
		"	fragment_color = mix(fragment_color, marked_vertex_color, need_mark);\n"
		"	fragment_color = mix(vertex_color, fragment_color, flags.x);\n"
		"}\0";

	unsigned int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
		throw std::exception("Fragment shader compilation failed");
	}

	// create shader program
	m_shader_program = glCreateProgram();
	glAttachShader(m_shader_program, vertex_shader);
	glAttachShader(m_shader_program, fragment_shader);
	glLinkProgram(m_shader_program);
	glGetProgramiv(m_shader_program, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetProgramInfoLog(m_shader_program, 512, NULL, infoLog);
		throw std::exception("Shader program initialization failed");
	}
	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);

	// create buffers
	glGenVertexArrays(1, &m_vao);

	glGenBuffers(1, &m_cloth_vbo);
	glGenBuffers(1, &m_cloth_ebo);

	glGenBuffers(1, &m_collider_vbo);
	glGenBuffers(1, &m_collider_ebo);

	glGenBuffers(1, &m_candidate_vbo);
	glGenBuffers(1, &m_candidate_ebo);

	// get uniforms locations
	m_color_location = glGetUniformLocation(m_shader_program, "vertex_color");
	m_marked_vertex_color_location = glGetUniformLocation(m_shader_program, "marked_vertex_color");
	m_view_matrix_location = glGetUniformLocation(m_shader_program, "view_mat");
	m_projection_matrix_location = glGetUniformLocation(m_shader_program, "projection_mat");
	m_light_source_position_location = glGetUniformLocation(m_shader_program, "light_source_position");
	m_light_source_color_location = glGetUniformLocation(m_shader_program, "light_source_color");
	m_intensities_location = glGetUniformLocation(m_shader_program, "intensities");
	m_camera_position_location = glGetUniformLocation(m_shader_program, "camera_position");
	m_vertices_to_mark_location = glGetUniformLocation(m_shader_program, "vertices_to_mark");
	m_flags_location = glGetUniformLocation(m_shader_program, "flags");
	m_points_size_location = glGetUniformLocation(m_shader_program, "points_size");
}

void SimulationView::newFrameStart(float aspect_ratio)
{
	glClearColor(m_draw_parameters.m_background_color[0], m_draw_parameters.m_background_color[1], m_draw_parameters.m_background_color[2], 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	m_draw_parameters.m_aspect_ratio = aspect_ratio;
}

void SimulationView::setUniforms(const glm::vec3& color, float specular_intensity, bool use_normals, bool mark_vertices) const
{
	const glm::vec3 camera_front = glm::normalize(glm::vec3(
		cosf(m_draw_parameters.m_camera_yaw) * cosf(m_draw_parameters.m_camera_pitch),
		sinf(m_draw_parameters.m_camera_pitch),
		sinf(m_draw_parameters.m_camera_yaw) * cosf(m_draw_parameters.m_camera_pitch)));

	const glm::mat4x4 view_matrix =
		glm::lookAt(m_draw_parameters.m_camera_position, m_draw_parameters.m_camera_position + camera_front, m_draw_parameters.m_camera_up);
	const glm::mat4x4 projection_matrix =
		glm::perspective(glm::radians(m_draw_parameters.m_camera_fov), m_draw_parameters.m_aspect_ratio, 0.1f, 2000.0f);

	glUniform4f(m_color_location, color[0], color[1], color[2], 1.0f);
	glUniform4f(m_marked_vertex_color_location, m_draw_parameters.m_vertex_to_mark_color[0], m_draw_parameters.m_vertex_to_mark_color[1], m_draw_parameters.m_vertex_to_mark_color[2], 1.0f);
	glUniformMatrix4fv(m_view_matrix_location, 1, false, glm::value_ptr(view_matrix));
	glUniformMatrix4fv(m_projection_matrix_location, 1, false, glm::value_ptr(projection_matrix));
	glUniform3f(m_light_source_position_location, m_draw_parameters.m_light_source_position[0], m_draw_parameters.m_light_source_position[1], m_draw_parameters.m_light_source_position[2]);
	glUniform4f(m_light_source_color_location, m_draw_parameters.m_light_source_color[0], m_draw_parameters.m_light_source_color[1], m_draw_parameters.m_light_source_color[2], 1.0f);
	glUniform3f(m_intensities_location, m_draw_parameters.m_light_source_intensity, m_draw_parameters.m_ambient_light_intensity, specular_intensity);
	glUniform3f(m_camera_position_location, m_draw_parameters.m_camera_position[0], m_draw_parameters.m_camera_position[1], m_draw_parameters.m_camera_position[2]);
	glUniform2i(m_vertices_to_mark_location, m_draw_parameters.m_vertices_to_mark[0], m_draw_parameters.m_vertices_to_mark[1]);
	glUniform2f(m_flags_location, (float)use_normals, (float)mark_vertices);
	glUniform1f(m_points_size_location, m_draw_parameters.m_points_size);
}


void SimulationView::renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices,
	const std::vector<glm::vec3>& normals, DrawMode draw_mode, int vbo, int ebo) const
{
	const uint64_t u_vertices_count = (uint64_t)vertices_count;
	const uint64_t bytes_length = u_vertices_count * 3u * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * bytes_length, nullptr, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bytes_length, coords.data());
	glBufferSubData(GL_ARRAY_BUFFER, bytes_length, bytes_length, normals.data());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)bytes_length);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * indices.size() * sizeof(uint32_t), indices.data(), GL_DYNAMIC_DRAW);

	glBindVertexArray(m_vao);

	glUseProgram(m_shader_program);

	switch (draw_mode)
	{
	case DrawMode::FILL:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::POINTS:
		glDrawElements(GL_POINTS, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::LINES:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	default:
		throw std::exception("Wrong draw mode");
	}
}

void SimulationView::renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::vec3>& normals, DrawMode draw_mode, int vbo, int ebo) const
{
	const uint64_t u_vertices_count = (uint64_t)vertices_count;
	const uint64_t bytes_length = u_vertices_count * 3u * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * bytes_length, nullptr, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bytes_length, coords.data());
	glBufferSubData(GL_ARRAY_BUFFER, bytes_length, bytes_length, normals.data());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)bytes_length);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

	glBindVertexArray(m_vao);

	glUseProgram(m_shader_program);

	switch (draw_mode)
	{
	case DrawMode::FILL:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::POINTS:
		glDrawElements(GL_POINTS, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::LINES:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	default:
		throw std::exception("Wrong draw mode");
	}
}

void SimulationView::renderInternal(int vertices_count, int triangles_count, const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices, DrawMode draw_mode, int vbo, int ebo) const
{
	const uint64_t u_vertices_count = (uint64_t)vertices_count;
	const uint64_t bytes_length = u_vertices_count * 3u * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, bytes_length, nullptr, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bytes_length, coords.data());
	glBufferSubData(GL_ARRAY_BUFFER, bytes_length, bytes_length, coords.data());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)bytes_length);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * indices.size() * sizeof(uint32_t), indices.data(), GL_DYNAMIC_DRAW);

	glBindVertexArray(m_vao);

	glUseProgram(m_shader_program);

	switch (draw_mode)
	{
	case DrawMode::FILL:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::POINTS:
		glDrawElements(GL_POINTS, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::LINES:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	default:
		throw std::exception("Wrong draw mode");
	}
}

void SimulationView::renderInternal(int vertices_count, int triangles_count, DrawMode draw_mode, int vbo, int ebo) const
{
	const uint64_t u_vertices_count = (uint64_t)vertices_count;
	const uint64_t bytes_length = u_vertices_count * 3u * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)bytes_length);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

	glBindVertexArray(m_vao);

	glUseProgram(m_shader_program);

	switch (draw_mode)
	{
	case DrawMode::FILL:
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::POINTS:
		glDrawElements(GL_POINTS, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	case DrawMode::LINES:
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawElements(GL_TRIANGLES, 3u * GLsizei(triangles_count), GL_UNSIGNED_INT, 0);
		break;
	default:
		throw std::exception("Wrong draw mode");
	}
}
