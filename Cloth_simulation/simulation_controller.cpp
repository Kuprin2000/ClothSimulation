// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "simulation_controller.h"
#include <Windows.h>
#include <fstream>
#include <filesystem>
#include <limits>
#include "implot/implot.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "collider.h"

constexpr int STATISTIC_SIZE = 300;
int STATISTIC_INDEX = 0;

SimulationController::SimulationController(SimulationModel* model, SimulationView* view) : m_model(model), m_view(view)
{
	if (!m_model || !m_view)
	{
		throw std::exception("Got invalid pointers");
	}

	m_found_files.push_back("None");

	std::filesystem::directory_iterator iter(std::filesystem::current_path());
	for (; iter != std::filesystem::end(iter); ++iter)
	{
		if (iter->path().extension().string() == m_settings.m_file_extention)
		{
			m_found_files.emplace_back(iter->path().filename().string());
		}
	}

	m_found_files_ptrs.reserve(m_found_files.size());
	for (const auto& file : m_found_files)
	{
		m_found_files_ptrs.push_back(file.c_str());
	}

	m_statistic.m_framerate.resize(STATISTIC_SIZE, 0u);

#ifdef MEASURE_TIME
	m_statistic.m_rtree_creation_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_find_collisions_candidates_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_check_collisions_candidates_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_collisions_constraints_graph_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_user_constraints_graph_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_evaluate_constraints_time.resize(STATISTIC_SIZE, 0u);
	m_statistic.m_evaluate_friction_time.resize(STATISTIC_SIZE, 0u);
#endif

	m_statistic.m_collision_constraints_count.resize(STATISTIC_SIZE, 0u);
}

void SimulationController::newFrame()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	m_view->newFrameStart(ImGui::GetIO().DisplaySize.x / ImGui::GetIO().DisplaySize.y);

	showInterface();
	simulationStep(ImGui::GetIO().DeltaTime);

	render();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void SimulationController::simulationStep(float time_delta)
{
	m_last_frame_time = time_delta;

	if (m_state != SimulationState::SIMULATION_RUN)
	{
		return;
	}

	if (!m_simulation_frame_number)
	{
		m_model->prepareForSimulation();
	}

	float simulation_time_delta = (m_simulation_frame_number < m_settings.m_first_frames_count) ?
		m_settings.m_first_frames_time_step : time_delta;
	simulation_time_delta = m_settings.m_use_real_time ? simulation_time_delta : m_settings.m_time_step;

	pushStatistic(m_model->simulationStep(simulation_time_delta), time_delta);

	++m_simulation_frame_number;
	m_simulation_frame_number = m_simulation_frame_number % UINT64_MAX;


	if (!m_settings.m_use_real_time && fabsf(m_settings.m_pause_between_frames) > FLT_EPSILON)
	{
		Sleep((DWORD)(m_settings.m_pause_between_frames * 1000.0f));
	}
}

void SimulationController::showInterface()
{
	showDrawSettingsWindow();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	switch (m_state)
	{
	case SimulationState::ADD_CLOTH_AND_COLLIDERS:
		showClothAndCollidersWindow();
		break;
	case SimulationState::ADD_CONSTRAINTS:
		showConstraintsWindow();
		break;
	case SimulationState::CONFIGURE_SIMULATION:
		showSimulationParamsWindow();
		break;
	case SimulationState::SIMULATION_RUN:
	case SimulationState::SIMULATION_PAUSED:
		showRunningSimulationWindow();
		showStatisticWindow();
		break;
	default:
		break;
	}
}

void SimulationController::render() const
{
	const Cloth& cloth = m_model->getCloth();
	const Collider& collider = m_model->getCollider();

	if (m_state == SimulationState::SIMULATION_RUN || m_state == SimulationState::SIMULATION_PAUSED)
	{
		// when simulation is in process cloth indices are constant
		m_view->render_cloth(cloth.getTrianglesCount(), cloth.getCoords(), cloth.getNormals());
		// when simulation is in process all collider buffers are constant
		m_view->render_collider(collider.getVerticesCount(), collider.getTrianglesCount());
	}
	else
	{
		m_view->render_cloth(cloth.getCoords(), cloth.getIndices(), cloth.getNormals(), (m_state == SimulationState::ADD_CONSTRAINTS));
		m_view->render_collider(collider.getCoords(), collider.getIndices(), collider.getNormals());
	}

	if (m_state == SimulationState::ADD_CLOTH_AND_COLLIDERS)
	{
		m_view->render_candidate_object(m_candidate_object.m_coords, m_candidate_object.m_indices);
	}
}

void SimulationController::moveCamera(MovementDirection direction, bool fast)
{
	DrawParameters& draw_parameters = m_view->getDrawParameters();
	const glm::vec3 camera_front = glm::normalize(glm::vec3(
		cosf(draw_parameters.m_camera_yaw) * cosf(draw_parameters.m_camera_pitch),
		sinf(draw_parameters.m_camera_pitch),
		sinf(draw_parameters.m_camera_yaw) * cosf(draw_parameters.m_camera_pitch)));
	const float coeff = (1.0f + float(fast)) * m_settings.m_camera_linear_speed * m_last_frame_time;

	switch (direction)
	{
	case MovementDirection::FRONT:
		draw_parameters.m_camera_position += camera_front * coeff;
		break;
	case MovementDirection::BACK:
		draw_parameters.m_camera_position -= camera_front * coeff;
		break;
	case MovementDirection::LEFT:
		draw_parameters.m_camera_position -= glm::cross(camera_front, draw_parameters.m_camera_up) * coeff;
		break;
	case MovementDirection::RIGHT:
		draw_parameters.m_camera_position += glm::cross(camera_front, draw_parameters.m_camera_up) * coeff;
		break;
	case MovementDirection::UP:
		draw_parameters.m_camera_position += draw_parameters.m_camera_up * coeff;
		break;
	case MovementDirection::DOWN:
		draw_parameters.m_camera_position -= draw_parameters.m_camera_up * coeff;
		break;
	default:
		throw std::exception("Wrong camera movement direction");
		break;
	}
}

bool SimulationController::loadCloth()
{
	std::vector<glm::vec3> coords;
	std::vector<glm::uvec3> indices;
	if (!readGeometryFromFile(coords, indices, m_found_files[m_new_cloth_parameters.m_file_index]))
	{
		return false;
	}

	transformCoords(m_new_cloth_parameters.m_rotations, m_new_cloth_parameters.m_size, m_new_cloth_parameters.m_center_coords, coords);

	float cloth_square = 0.0;
	for (int i = 0; i < indices.size(); ++i)
	{
		cloth_square += MathUtils::triangleSquare(coords[indices[i][0]], coords[indices[i][1]], coords[indices[i][2]]);
	}

	const float vertex_mass = cloth_square * m_new_cloth_parameters.m_material_props.m_density / coords.size();
	std::vector<float> opposite_masses(coords.size(), vertex_mass);

	std::vector<Cloth> new_cloth;
	new_cloth.emplace_back(std::move(coords), std::move(indices), std::move(opposite_masses), m_new_cloth_parameters.m_material_props,
		m_new_cloth_parameters.m_layer_number, m_new_cloth_parameters.m_use_realistic_stretch, m_new_cloth_parameters.m_use_realistic_bending);
	m_model->addClothParts(std::move(new_cloth));

	return true;
}

bool SimulationController::loadCollider()
{
	std::vector<glm::vec3> coords;
	std::vector<glm::uvec3> indices;
	if (!readGeometryFromFile(coords, indices, m_found_files[m_new_collider_parameters.m_file_index]))
	{
		return false;
	}

	transformCoords(m_new_collider_parameters.m_rotations, m_new_collider_parameters.m_size, m_new_collider_parameters.m_center_coords, coords);

	std::vector<Collider> new_collider;
	new_collider.emplace_back(std::move(coords), std::move(indices), m_new_collider_parameters.m_friction_coeff);
	m_model->addColliders(std::move(new_collider));

	return true;
}

bool SimulationController::loadClothCandidate()
{
	if (!m_new_cloth_parameters.m_file_index)
	{
		m_candidate_object.m_coords.clear();
		m_candidate_object.m_indices.clear();
		m_candidate_object.m_average_edge_length = 0.0f;
		return true;
	}

	if (!readGeometryFromFile(m_candidate_object.m_coords, m_candidate_object.m_indices, m_found_files[m_new_cloth_parameters.m_file_index]))
	{
		return false;
	}

	transformCoords(m_new_cloth_parameters.m_rotations, m_new_cloth_parameters.m_size, m_new_cloth_parameters.m_center_coords, m_candidate_object.m_coords);

	m_candidate_object.m_average_edge_length = getAverageEdgeLength(m_candidate_object.m_coords, m_candidate_object.m_indices);

	// these parameters are optimal
	m_new_cloth_parameters.m_material_props.m_thickness = m_candidate_object.m_average_edge_length * 0.1f;
	m_model->getSettings().m_max_collision_radius_for_cloth = m_candidate_object.m_average_edge_length * 0.1f;

	return true;
}

bool SimulationController::loadColliderCandidate()
{
	if (!m_new_collider_parameters.m_file_index)
	{
		m_candidate_object.m_coords.clear();
		m_candidate_object.m_indices.clear();
		m_candidate_object.m_average_edge_length = 0.0f;
		return true;
	}

	if (!readGeometryFromFile(m_candidate_object.m_coords, m_candidate_object.m_indices, m_found_files[m_new_collider_parameters.m_file_index]))
	{
		return false;
	}

	transformCoords(m_new_collider_parameters.m_rotations, m_new_collider_parameters.m_size, m_new_collider_parameters.m_center_coords, m_candidate_object.m_coords);

	m_candidate_object.m_average_edge_length = getAverageEdgeLength(m_candidate_object.m_coords, m_candidate_object.m_indices);

	return true;
}

float SimulationController::getAverageEdgeLength(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices) const
{
	float result = 0.0f;
	for (const auto& triangle : indices)
	{
		result += glm::distance(coords[triangle[0]], coords[triangle[1]]);
		result += glm::distance(coords[triangle[1]], coords[triangle[2]]);
		result += glm::distance(coords[triangle[2]], coords[triangle[0]]);
	}

	return result / indices.size() / 3.0f;
}

void SimulationController::transformCoords(const glm::vec3& rotations, const glm::vec3& size, const glm::vec3& translation, std::vector<glm::vec3>& coords) const
{
	glm::mat4x4 rotation = glm::identity<glm::mat4x4>();
	rotation = glm::rotate(rotation, glm::radians(rotations[0]), glm::vec3(1.0f, 0.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(rotations[1]), glm::vec3(0.0f, 1.0f, 0.0f));
	rotation = glm::rotate(rotation, glm::radians(rotations[2]), glm::vec3(0.0f, 0.0f, 1.0f));

	for (int i = 0; i < coords.size(); ++i)
	{
		coords[i] = rotation * glm::vec4(coords[i], 0.0f);
		coords[i] *= size;
		coords[i] += translation;
	}
}

bool SimulationController::readGeometryFromFile(std::vector<glm::vec3>& coords, std::vector<glm::uvec3>& indices, const std::string& file_path) const
{
	std::ifstream stream;
	stream.open(file_path.c_str());
	if (!stream.is_open())
	{
		return false;
	}

	stream.ignore(1000, '\n');
	stream.ignore(1000, ' ');

	int vertices_count = 0;
	stream >> vertices_count;
	coords.resize(vertices_count);

	stream.ignore(1000, ' ');
	int triangles_count = 0;
	stream >> triangles_count;
	indices.resize(triangles_count);

	stream.ignore(1000, '\n');
	stream.ignore(1000, '\n');
	stream.ignore(1000, '\n');

	for (int i = 0; i < vertices_count; ++i)
	{
		stream.ignore(1000, ' ');
		stream >> coords[i][0];
		stream >> coords[i][1];
		stream >> coords[i][2];
	}

	for (int i = 0; i < triangles_count; ++i)
	{
		stream.ignore(1000, ' ');

		stream >> indices[i][0];
		stream >> indices[i][1];
		stream >> indices[i][2];

		--indices[i][0];
		--indices[i][1];
		--indices[i][2];
	}

	stream.close();

	return true;
}

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationController::addFixedVertex(const std::vector<FixedVertexTask>& tasks)
{
	return m_model->addFixedVertex(tasks);
}

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationController::addFixedAngle(const std::vector<FixedAngleTask>& tasks)
{
	return m_model->addFixedAngle(tasks);
}

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationController::addSewing(const std::vector<SewTask>& tasks)
{
	return m_model->addSewing(tasks);
}

void SimulationController::showDrawSettingsWindow()
{
	DrawParameters& draw_parameters = m_view->getDrawParameters();
	bool need_clamp = false;

	ImGui::Begin("Draw parameters");

	constexpr const char* draw_modes[3] = { "Fill", "Points", "Lines" };

	ImGui::Text("Cloth");
	ImGui::ListBox("Cloth draw mode", (int*)&draw_parameters.m_cloth_draw_mode, draw_modes, 3);
	ImGui::ColorEdit3("Cloth color", glm::value_ptr(draw_parameters.m_cloth_color));
	need_clamp |= ImGui::InputFloat("Cloth specular light ", &draw_parameters.m_cloth_specular_intensity);

	ImGui::Text("Collider");
	ImGui::ListBox("Collider draw mode", (int*)&draw_parameters.m_collider_draw_mode, draw_modes, 3);
	ImGui::ColorEdit3("Colliders color", glm::value_ptr(draw_parameters.m_collider_color));
	need_clamp |= ImGui::InputFloat("Collider specular light ", &draw_parameters.m_collider_specular_intensity);

	ImGui::Text("Candidate");
	ImGui::ListBox("Candidate draw mode", (int*)&draw_parameters.m_candidate_draw_mode, draw_modes, 3);
	ImGui::ColorEdit3("Candidate color", glm::value_ptr(draw_parameters.m_candidate_color));
	need_clamp |= ImGui::InputFloat("Candidate specular light ", &draw_parameters.m_candidate_specular_intensity);

	ImGui::Text("Vertices of user defined constraints");
	ImGui::ColorEdit3("Vertices color", glm::value_ptr(draw_parameters.m_vertex_to_mark_color));

	ImGui::Text("Background");
	ImGui::ColorEdit3("Background color", glm::value_ptr(draw_parameters.m_background_color));

	ImGui::Text("Light sources");
	ImGui::InputFloat3("Light source position", glm::value_ptr(draw_parameters.m_light_source_position));
	ImGui::ColorEdit3("Light source color", glm::value_ptr(draw_parameters.m_light_source_color));
	ImGui::InputFloat("Light source intensity", &draw_parameters.m_light_source_intensity);
	ImGui::InputFloat("Ambient light intensity", &draw_parameters.m_ambient_light_intensity);

	ImGui::Text("Camera");
	need_clamp |= ImGui::InputFloat("Camera FOV", &draw_parameters.m_camera_fov);

	ImGui::Text("Points");
	need_clamp |= ImGui::InputFloat("Points size", &draw_parameters.m_points_size);

	ImGui::End();

	if (need_clamp)
	{
		clampDrawParameters();
	}
}

void SimulationController::showClothAndCollidersWindow()
{
	bool need_reload_candidate = false;
	bool need_clamp = false;

	ImGui::Begin("Add cloth and colliders");

	if (ImGui::Button("Next step") && !m_model->isEmpty())
	{
		m_state = SimulationState::ADD_CONSTRAINTS;
	}

	ImGui::Separator();

	if (ImGui::BeginTabBar("Add object"))
	{
		if (ImGui::BeginTabItem("Add cloth"))
		{
			if (!m_add_cloth_tab_selected)
			{
				m_add_cloth_tab_selected = true;
				need_reload_candidate = true;
			}

			ImGui::Text("File");
			need_reload_candidate |= ImGui::ListBox("File", &m_new_cloth_parameters.m_file_index, m_found_files_ptrs.data(), (int)m_found_files_ptrs.size());

			ImGui::Text("Transformations");
			need_reload_candidate |= ImGui::InputFloat3("Scale", glm::value_ptr(m_new_cloth_parameters.m_size));
			need_reload_candidate |= ImGui::InputFloat3("Movement", glm::value_ptr(m_new_cloth_parameters.m_center_coords));
			need_reload_candidate |= ImGui::InputFloat3("Rotations along axis", glm::value_ptr(m_new_cloth_parameters.m_rotations));

			ImGui::Text("Edges");
			std::stringstream string;
			string.precision(3);
			string << "Average edge length " << m_candidate_object.m_average_edge_length;
			ImGui::Text(string.str().c_str());

			ImGui::Text("Layer");
			ImGui::InputInt("Layer number", &m_new_cloth_parameters.m_layer_number);

			ImGui::Text("Material properties");
			need_clamp |= ImGui::InputFloat("Thickness (cm)", &m_new_cloth_parameters.m_material_props.m_thickness);
			need_clamp |= ImGui::InputFloat("Friction coeff (0-1)", &m_new_cloth_parameters.m_material_props.m_friction_coeff);
			need_clamp |= ImGui::InputFloat("Density (kg/cm^2)", &m_new_cloth_parameters.m_material_props.m_density);

			ImGui::Text("Bending");
			ImGui::Checkbox("Use realistic bending", &m_new_cloth_parameters.m_use_realistic_bending);
			need_clamp |= ImGui::InputFloat("Material bending stiffness (0-10^5)", &m_new_cloth_parameters.m_material_props.m_bending_stiffness);

			ImGui::Text("Stretch");
			ImGui::Checkbox("Use realistic stretch", &m_new_cloth_parameters.m_use_realistic_stretch);
			if (m_new_cloth_parameters.m_use_realistic_stretch)
			{
				need_clamp |= ImGui::InputFloat("Stretch stiffness (0-10^5)", &m_new_cloth_parameters.m_material_props.m_stretch_stiffness);
				need_clamp |= ImGui::InputFloat("Young welt", &m_new_cloth_parameters.m_material_props.m_young_welt);
				need_clamp |= ImGui::InputFloat("Young warp", &m_new_cloth_parameters.m_material_props.m_young_warp);
				need_clamp |= ImGui::InputFloat("Poisson welt (0-1)", &m_new_cloth_parameters.m_material_props.m_poisson_welt);
				need_clamp |= ImGui::InputFloat("Poisson warp (0-1)", &m_new_cloth_parameters.m_material_props.m_poisson_warp);
				need_clamp |= ImGui::InputFloat("Shear modulus (0-1)", &m_new_cloth_parameters.m_material_props.m_shear_modulus);
			}
			else
			{
				need_clamp |= ImGui::InputFloat("Stretch stiffness (0-10^5)", &m_new_cloth_parameters.m_material_props.m_stretch_stiffness);
			}

			if (need_reload_candidate)
			{
				const bool ok = loadClothCandidate();
			}

			if (m_new_cloth_parameters.m_file_index && ImGui::Button("Add cloth"))
			{
				const bool ok = loadCloth();
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Add collider"))
		{
			if (m_add_cloth_tab_selected)
			{
				m_add_cloth_tab_selected = false;
				need_reload_candidate = true;
			}

			need_reload_candidate |= ImGui::ListBox("File", &m_new_collider_parameters.m_file_index, m_found_files_ptrs.data(), (int)m_found_files.size());

			ImGui::Text("Transformations");
			need_reload_candidate |= ImGui::InputFloat3("Scale", glm::value_ptr(m_new_collider_parameters.m_size));
			need_reload_candidate |= ImGui::InputFloat3("Movement", glm::value_ptr(m_new_collider_parameters.m_center_coords));
			need_reload_candidate |= ImGui::InputFloat3("Rotations along axis", glm::value_ptr(m_new_collider_parameters.m_rotations));

			ImGui::Text("Edges");
			std::stringstream string;
			string.precision(3);
			string << "Average edge length " << m_candidate_object.m_average_edge_length;
			ImGui::Text(string.str().c_str());

			ImGui::Text("Material properties");
			need_clamp |= ImGui::InputFloat("Friction coeff (0-1)", &m_new_collider_parameters.m_friction_coeff);

			if (need_reload_candidate)
			{
				const bool ok = loadColliderCandidate();
			}

			if (m_new_collider_parameters.m_file_index && ImGui::Button("Add collider"))
			{
				const bool ok = loadCollider();
			}

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::End();

	if (need_clamp)
	{
		clampClothAndColliderParameters();
	}
}

void SimulationController::showConstraintsWindow()
{
	bool need_clamp = false;

	ImGui::Begin("Add constraints");

	if (ImGui::Button("Next step"))
	{
		m_state = SimulationState::CONFIGURE_SIMULATION;
	}

	ImGui::Separator();

	if (ImGui::BeginTabBar("Parameters"))
	{
		DrawParameters& draw_parameters = m_view->getDrawParameters();

		if (ImGui::BeginTabItem("Add fixed vertex"))
		{
			need_clamp |= ImGui::InputInt("Vertex id", &m_new_fixed_vertex.m_vertex);
			ImGui::Checkbox("Use current position", &m_new_fixed_vertex.m_use_current_position);
			if (!m_new_fixed_vertex.m_use_current_position)
			{
				ImGui::InputFloat3("Vertex coords", glm::value_ptr(m_new_fixed_vertex.m_coords));
			}
			if (ImGui::Button("Add fixed vertex") && m_model)
			{
				addFixedVertex({ m_new_fixed_vertex });
			}

			draw_parameters.m_vertices_to_mark[0] = m_new_fixed_vertex.m_vertex;
			draw_parameters.m_vertices_to_mark[1] = m_new_fixed_vertex.m_vertex;

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Add fixed angle"))
		{
			need_clamp |= ImGui::InputInt("Vertex a id", &m_new_fixed_angle.m_vertex_a);
			need_clamp |= ImGui::InputInt("Vertex b id", &m_new_fixed_angle.m_vertex_b);
			ImGui::InputFloat("Angle (rad)", &m_new_fixed_angle.m_angle);
			need_clamp |= ImGui::InputFloat("Stiffness (0-10^5)", &m_new_fixed_angle.m_stiffness);
			if (ImGui::Button("Add fixed angle") && m_model)
			{
				addFixedAngle({ m_new_fixed_angle });
			}

			draw_parameters.m_vertices_to_mark[0] = m_new_fixed_angle.m_vertex_a;
			draw_parameters.m_vertices_to_mark[1] = m_new_fixed_angle.m_vertex_b;

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Add sewing"))
		{
			need_clamp |= ImGui::InputInt("Vertex a id", &m_new_sew_task.m_vertex_a);
			need_clamp |= ImGui::InputInt("Vertex b id", &m_new_sew_task.m_vertex_b);
			need_clamp |= ImGui::InputFloat("Stiffness (0-10^5)", &m_new_sew_task.m_sewing_stiffness);
			need_clamp |= ImGui::InputFloat("Distance to join vertices (sm)", &m_new_sew_task.m_distance_to_join_vertices);
			if (ImGui::Button("Add sewing") && m_model)
			{
				addSewing({ m_new_sew_task });
			}

			draw_parameters.m_vertices_to_mark[0] = m_new_sew_task.m_vertex_a;
			draw_parameters.m_vertices_to_mark[1] = m_new_sew_task.m_vertex_b;

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	if (need_clamp)
	{
		clampConstraintsParameters();
	}

	ImGui::End();
}

void SimulationController::showSimulationParamsWindow()
{
	ChangedParameters changed_parameters;

	ImGui::Begin("Configure simulation");

	if (ImGui::Button("Start simulation"))
	{
		m_state = SimulationState::SIMULATION_RUN;
	}

	ImGui::Separator();

	ModelSettings& model_settings = m_model->getSettings();

	ImGui::Text("Speeds and accelerations");
	ImGui::InputFloat("Speed damping coefficient (0-1)", &model_settings.m_speed_damping_coefficient);
	ImGui::InputFloat("Gravitational acceleration (sm/s^2)", &model_settings.m_gravity);

	ImGui::Text("Collisions");
	ImGui::InputFloat("Max cloth collision radius (sm)", &model_settings.m_max_collision_radius_for_cloth);
	ImGui::InputFloat("Max collider collision radius (sm)", &model_settings.m_max_collision_radius_for_colliders);
	changed_parameters.m_need_recreate_r_tree |= ImGui::InputInt("R-tree min children", &model_settings.m_r_tree_min);
	changed_parameters.m_need_recreate_r_tree |= ImGui::InputInt("R-tree max children", &model_settings.m_r_tree_max);

	ImGui::Text("Calculations");
	changed_parameters.m_need_recreate_internal_constraints_graph |= ImGui::InputInt("Internal constraints threads count", &model_settings.m_internal_constraints_threads_count);
	changed_parameters.m_need_recreate_other_constraints_graphs |= ImGui::InputInt("Other constraints threads count", &model_settings.m_other_constraints_threads_count);
	ImGui::InputInt("Check collisions threads count", &model_settings.m_check_collisions_threads_count);
	changed_parameters.m_need_recreate_internal_constraints_graph |= ImGui::InputInt("Preferred partitions count", &model_settings.m_preferred_partitions_count);
	ImGui::InputInt("Iterations count", &model_settings.m_iterations_count);
	ImGui::Checkbox("Real time simulation", &m_settings.m_use_real_time);
	if (!m_settings.m_use_real_time)
	{
		ImGui::InputFloat("Time step (s)", &m_settings.m_time_step);
		ImGui::InputFloat("Pause between frames (s)", &m_settings.m_pause_between_frames);
	}

	if (changed_parameters.m_need_recreate_r_tree || changed_parameters.m_need_recreate_internal_constraints_graph ||
		changed_parameters.m_need_recreate_other_constraints_graphs)
	{
		clampSimulationParameters();
		m_model->notifySettingsUpdated(changed_parameters);
	}

	ImGui::End();
}

void SimulationController::showRunningSimulationWindow()
{
	ChangedParameters changed_parameters;

	ImGui::Begin("Configure simulation");

	if (m_state == SimulationState::SIMULATION_RUN && ImGui::Button("Pause simulation"))
	{
		m_state = SimulationState::SIMULATION_PAUSED;
	}
	if (m_state == SimulationState::SIMULATION_PAUSED && ImGui::Button("Continue simulation"))
	{
		m_state = SimulationState::SIMULATION_RUN;
	}

	ImGui::Separator();

	ModelSettings& model_settings = m_model->getSettings();

	ImGui::Text("Speeds and accelerations");
	ImGui::InputFloat("Speed damping coefficient (0-1)", &model_settings.m_speed_damping_coefficient);
	ImGui::InputFloat("Gravitational acceleration (sm/s^2)", &model_settings.m_gravity);

	ImGui::Text("Collisions");
	ImGui::InputFloat("Max cloth collision radius (sm)", &model_settings.m_max_collision_radius_for_cloth);
	ImGui::InputFloat("Max collider collision radius (sm)", &model_settings.m_max_collision_radius_for_colliders);
	changed_parameters.m_need_recreate_r_tree |= ImGui::InputInt("R-tree min children", &model_settings.m_r_tree_min);
	changed_parameters.m_need_recreate_r_tree |= ImGui::InputInt("R-tree max children", &model_settings.m_r_tree_max);

	ImGui::Text("Calculations");
	ImGui::InputInt("Iterations count", &model_settings.m_iterations_count);
	ImGui::Checkbox("Real time simulation", &m_settings.m_use_real_time);
	if (!m_settings.m_use_real_time)
	{
		ImGui::InputFloat("Time step (s)", &m_settings.m_time_step);
		ImGui::InputFloat("Pause between frames (s)", &m_settings.m_pause_between_frames);
	}

	ImGui::End();

	if (changed_parameters.m_need_recreate_r_tree || changed_parameters.m_need_recreate_internal_constraints_graph ||
		changed_parameters.m_need_recreate_other_constraints_graphs)
	{
		clampSimulationParameters();
		m_model->notifySettingsUpdated(changed_parameters);
	}
}

ImPlotPoint sampleUin8Stats(int i, void* data)
{
	return ImPlotPoint(i, ((uint8_t*)data)[(i + STATISTIC_INDEX) % STATISTIC_SIZE]);
}

ImPlotPoint sampleUin16Stats(int i, void* data)
{
	return ImPlotPoint(i, ((uint16_t*)data)[(i + STATISTIC_INDEX) % STATISTIC_SIZE]);
}

ImPlotPoint sampleUin32Stats(int i, void* data)
{
	return ImPlotPoint(i, ((uint32_t*)data)[(i + STATISTIC_INDEX) % STATISTIC_SIZE]);
}

void SimulationController::showStatisticWindow()
{
	ImGui::Begin("Statistic");

	int plots_flag = ImPlotFlags_::ImPlotFlags_NoMouseText + ImPlotFlags_::ImPlotFlags_NoInputs +
		ImPlotFlags_::ImPlotFlags_NoMenus + ImPlotFlags_::ImPlotFlags_NoBoxSelect + ImPlotFlags_::ImPlotFlags_NoLegend;
	if (ImPlot::BeginPlot("Framerate", { 600, 300 }, plots_flag))
	{
		ImPlot::SetupAxis(ImAxis_::ImAxis_X1, "", ImPlotAxisFlags_::ImPlotAxisFlags_NoDecorations);
		ImPlot::SetupAxis(ImAxis_::ImAxis_Y1, "", ImPlotAxisFlags_::ImPlotAxisFlags_AutoFit);
		ImPlot::PlotLineG("FPS", sampleUin16Stats, m_statistic.m_framerate.data(), STATISTIC_SIZE);
		ImPlot::EndPlot();
	};

	plots_flag -= ImPlotFlags_::ImPlotFlags_NoLegend;
#ifdef MEASURE_TIME
	if (ImPlot::BeginPlot("Timings", { 600, 300 }, plots_flag))
	{
		ImPlot::SetupAxis(ImAxis_::ImAxis_X1, "", ImPlotAxisFlags_::ImPlotAxisFlags_NoDecorations);
		ImPlot::SetupAxis(ImAxis_::ImAxis_Y1, "Miliseconds", ImPlotAxisFlags_::ImPlotAxisFlags_AutoFit);
		ImPlot::PlotLineG("Create R-tree", sampleUin16Stats, m_statistic.m_rtree_creation_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("Find collisions candidates", sampleUin16Stats, m_statistic.m_find_collisions_candidates_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("Check collisions candidates", sampleUin16Stats, m_statistic.m_check_collisions_candidates_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("Collisions graph", sampleUin16Stats, m_statistic.m_collisions_constraints_graph_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("User constraints graph", sampleUin16Stats, m_statistic.m_user_constraints_graph_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("Evaluate constraints", sampleUin16Stats, m_statistic.m_evaluate_constraints_time.data(), STATISTIC_SIZE);
		ImPlot::PlotLineG("Evaluate friction", sampleUin16Stats, m_statistic.m_evaluate_friction_time.data(), STATISTIC_SIZE);
		ImPlot::EndPlot();
	};
#endif

	std::string text = "Internal constraints count: " + std::to_string(m_statistic.m_internal_constraints_count);
	ImGui::Text(text.c_str());

	text = "Phantom constraints count: " + std::to_string(m_statistic.m_phantom_constraints_count);
	ImGui::Text(text.c_str());

	plots_flag += ImPlotFlags_::ImPlotFlags_NoLegend;
	if (ImPlot::BeginPlot("Collision constraints", { 600, 300 }, plots_flag))
	{
		ImPlot::SetupAxis(ImAxis_::ImAxis_X1, "", ImPlotAxisFlags_::ImPlotAxisFlags_NoDecorations);
		ImPlot::SetupAxis(ImAxis_::ImAxis_Y1, "", ImPlotAxisFlags_::ImPlotAxisFlags_AutoFit);
		ImPlot::PlotLineG("Collision constraints count", sampleUin32Stats, m_statistic.m_collision_constraints_count.data(), STATISTIC_SIZE);
		ImPlot::EndPlot();
	}

	ImGui::Text("User constraints");
	text = "User constraints count: " + std::to_string(m_statistic.m_user_defined_constraints_count);
	ImGui::Text(text.c_str());

	ImGui::End();
	}

void SimulationController::pushStatistic(const ExecutionStatistic& statistic, float time_delta)
{
	m_statistic.m_framerate[STATISTIC_INDEX] = (uint16_t)(1.0f / time_delta);

#ifdef MEASURE_TIME
	m_statistic.m_rtree_creation_time[STATISTIC_INDEX] = statistic.m_rtree_creation_time;
	m_statistic.m_find_collisions_candidates_time[STATISTIC_INDEX] = statistic.m_find_collisions_candidates_time;
	m_statistic.m_check_collisions_candidates_time[STATISTIC_INDEX] = statistic.m_check_collisions_candidates_time;
	m_statistic.m_collisions_constraints_graph_time[STATISTIC_INDEX] = statistic.m_collisions_constraints_graph_time;
	m_statistic.m_user_constraints_graph_time[STATISTIC_INDEX] = statistic.m_user_constraints_graph_time;
	m_statistic.m_evaluate_constraints_time[STATISTIC_INDEX] = statistic.m_evaluate_constraints_time;
	m_statistic.m_evaluate_friction_time[STATISTIC_INDEX] = statistic.m_evaluate_friction_time;
#endif

	m_statistic.m_internal_constraints_count = statistic.m_internal_constraints_count;
	m_statistic.m_phantom_constraints_count = statistic.m_phantom_constraints_count;
	m_statistic.m_collision_constraints_count[STATISTIC_INDEX] = statistic.m_collision_constraints_count;
	m_statistic.m_user_defined_constraints_count = statistic.m_user_defined_constraints_count;

	++STATISTIC_INDEX;
	STATISTIC_INDEX = STATISTIC_INDEX % STATISTIC_SIZE;
}