#pragma once
#include "simulation_model.h"
#include "simulation_view.h"
#include "defines.h"

enum class SimulationState : uint8_t
{
	ADD_CLOTH_AND_COLLIDERS,
	ADD_CONSTRAINTS,
	CONFIGURE_SIMULATION,
	SIMULATION_RUN,
	SIMULATION_PAUSED
};

enum class MovementDirection : uint8_t
{
	FRONT,
	BACK,
	LEFT,
	RIGHT,
	UP,
	DOWN
};

class SimulationController
{
public:
	SimulationController() = delete;

	SimulationController(const SimulationController&) = delete;

	explicit SimulationController(SimulationModel* model, SimulationView* view);

	void newFrame();

	void simulationStep(float time_delta);

	void showInterface();

	void render() const;

	void moveCamera(MovementDirection direction, bool fast);

	void rotateCamera(float x_offset, float y_offset)
	{
		if (fabsf(x_offset) < FLT_EPSILON && fabsf(y_offset) < FLT_EPSILON)
		{
			return;
		}

		DrawParameters& draw_parameters = m_view->getDrawParameters();

		draw_parameters.m_camera_yaw += m_settings.m_mouse_sensitivity * x_offset;
		draw_parameters.m_camera_pitch += m_settings.m_mouse_sensitivity * y_offset;
		draw_parameters.m_camera_pitch =
			std::clamp(draw_parameters.m_camera_pitch, -MathUtils::PI * 0.5f + FLT_EPSILON, MathUtils::PI * 0.5f - FLT_EPSILON);
	}

	_NODISCARD SimulationState getState() const
	{
		return m_state;
	}

private:
	_NODISCARD bool loadCloth();

	_NODISCARD bool loadCollider();

	_NODISCARD bool loadClothCandidate();

	_NODISCARD bool loadColliderCandidate();

	void transformCoords(const glm::vec3& rotations, const glm::vec3& size, const glm::vec3& translation, std::vector<glm::vec3>& coords) const;

	_NODISCARD float getAverageEdgeLength(const std::vector<glm::vec3>& coords, const std::vector<glm::uvec3>& indices) const;

	_NODISCARD bool readGeometryFromFile(std::vector<glm::vec3>& coords, std::vector<glm::uvec3>& indices, const std::string& file_path) const;

	std::vector<std::vector<KeysUtils::ConstraintKey>> addFixedVertex(const std::vector<FixedVertexTask>& tasks);

	std::vector<std::vector<KeysUtils::ConstraintKey>> addFixedAngle(const std::vector<FixedAngleTask>& tasks);

	std::vector<std::vector<KeysUtils::ConstraintKey>> addSewing(const std::vector<SewTask>& tasks);

	void showDrawSettingsWindow();

	void showClothAndCollidersWindow();

	void showConstraintsWindow();

	void showSimulationParamsWindow();

	void showRunningSimulationWindow();

	void clampDrawParameters()
	{
		DrawParameters& draw_parameters = m_view->getDrawParameters();

		draw_parameters.m_cloth_specular_intensity = std::clamp(draw_parameters.m_cloth_specular_intensity, 0.0f, 1.0f);
		draw_parameters.m_collider_specular_intensity = std::clamp(draw_parameters.m_collider_specular_intensity, 0.0f, 1.0f);
		draw_parameters.m_candidate_specular_intensity = std::clamp(draw_parameters.m_candidate_specular_intensity, 0.0f, 1.0f);
		draw_parameters.m_camera_fov = std::clamp(draw_parameters.m_camera_fov, 0.0f, 180.0f);
		draw_parameters.m_points_size = std::clamp(draw_parameters.m_points_size, 0.0f, 50.f);
	}

	void clampClothAndColliderParameters()
	{
		m_new_cloth_parameters.m_material_props.m_thickness = std::clamp(m_new_cloth_parameters.m_material_props.m_thickness, FLT_EPSILON, 10.0f);
		m_new_cloth_parameters.m_material_props.m_friction_coeff = std::clamp(m_new_cloth_parameters.m_material_props.m_friction_coeff, FLT_EPSILON, 1.0f);
		m_new_cloth_parameters.m_material_props.m_density = std::clamp(m_new_cloth_parameters.m_material_props.m_density, FLT_EPSILON, 1000.0f);
		m_new_cloth_parameters.m_material_props.m_bending_stiffness = std::clamp(m_new_cloth_parameters.m_material_props.m_bending_stiffness, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_young_welt = std::clamp(m_new_cloth_parameters.m_material_props.m_young_welt, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_young_warp = std::clamp(m_new_cloth_parameters.m_material_props.m_young_warp, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_poisson_welt = std::clamp(m_new_cloth_parameters.m_material_props.m_poisson_welt, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_poisson_warp = std::clamp(m_new_cloth_parameters.m_material_props.m_poisson_warp, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_shear_modulus = std::clamp(m_new_cloth_parameters.m_material_props.m_shear_modulus, FLT_EPSILON, 100000.0f);
		m_new_cloth_parameters.m_material_props.m_stretch_stiffness = std::clamp(m_new_cloth_parameters.m_material_props.m_stretch_stiffness, FLT_EPSILON, 100000.0f);

		m_new_collider_parameters.m_friction_coeff = std::clamp(m_new_collider_parameters.m_friction_coeff, FLT_EPSILON, 1.0f);
	}

	void clampConstraintsParameters()
	{
		const int max_vertex = m_model->getCloth().getRealVerticesCount() - 1;

		m_new_fixed_vertex.m_vertex = std::clamp(m_new_fixed_vertex.m_vertex, 0, max_vertex);
		m_new_fixed_angle.m_vertex_a = std::clamp(m_new_fixed_angle.m_vertex_a, 0, max_vertex);
		m_new_fixed_angle.m_vertex_b = std::clamp(m_new_fixed_angle.m_vertex_b, 0, max_vertex);
		m_new_fixed_angle.m_stiffness = std::clamp(m_new_fixed_angle.m_stiffness, FLT_EPSILON, 100000.0f);
		m_new_sew_task.m_vertex_a = std::clamp(m_new_sew_task.m_vertex_a, 0, max_vertex);
		m_new_sew_task.m_vertex_b = std::clamp(m_new_sew_task.m_vertex_b, 0, max_vertex);
		m_new_sew_task.m_sewing_stiffness = std::clamp(m_new_sew_task.m_sewing_stiffness, FLT_EPSILON, 100000.0f);
		m_new_sew_task.m_distance_to_join_vertices = std::clamp(m_new_sew_task.m_distance_to_join_vertices, FLT_EPSILON, 100000.0f);
	}

	void clampSimulationParameters()
	{
		ModelSettings& model_settings = m_model->getSettings();

		model_settings.m_speed_damping_coefficient = std::clamp(model_settings.m_speed_damping_coefficient, 0.0f, 1.0f);
		model_settings.m_gravity = std::clamp(model_settings.m_gravity, -1000.0f, 1000.0f);
		model_settings.m_max_collision_radius_for_cloth = std::clamp(model_settings.m_max_collision_radius_for_cloth, 0.0f, 10.0f);
		model_settings.m_max_collision_radius_for_colliders = std::clamp(model_settings.m_max_collision_radius_for_colliders, 0.0f, 10.0f);
		model_settings.m_r_tree_min = std::clamp(model_settings.m_r_tree_min, 1, 50);
		model_settings.m_r_tree_max = std::clamp(model_settings.m_r_tree_max, model_settings.m_r_tree_min * 2, 100);
		model_settings.m_internal_constraints_threads_count = std::clamp(model_settings.m_internal_constraints_threads_count, 1, 32);
		model_settings.m_other_constraints_threads_count = std::clamp(model_settings.m_other_constraints_threads_count, 1, 32);
		model_settings.m_check_collisions_threads_count = std::clamp(model_settings.m_check_collisions_threads_count, 1, 32);
		model_settings.m_preferred_partitions_count = std::clamp(model_settings.m_preferred_partitions_count, 0, 30);
		model_settings.m_iterations_count = std::clamp(model_settings.m_iterations_count, 0, 100);
		m_settings.m_time_step = std::clamp(m_settings.m_time_step, 0.0f, 1.0f);
		m_settings.m_pause_between_frames = std::clamp(m_settings.m_pause_between_frames, 0.0f, 10.0f);
	}

	void showStatisticWindow();

	void pushStatistic(const ExecutionStatistic& statistic, float time_delta);

private:
	SimulationState m_state = SimulationState::ADD_CLOTH_AND_COLLIDERS;
	uint64_t m_simulation_frame_number = 0;
	float m_last_frame_time = 0.001f;

	SimulationModel* m_model = nullptr;
	SimulationView* m_view = nullptr;

	std::vector<std::string> m_found_files;
	std::vector<const char*> m_found_files_ptrs;

	bool m_add_cloth_tab_selected = true;

	struct
	{
		const std::string m_file_extention = ".smf";

		bool m_use_real_time = false;
		float m_time_step = 1.0f / 100.0f;
		float m_pause_between_frames = 0.0f;

		const int m_first_frames_count = 5;
		const float m_first_frames_time_step = 0.001f;

		const float m_camera_linear_speed = 150.0f;
		const float m_mouse_sensitivity = 0.01f;
	}
	m_settings;

	struct
	{
		glm::vec3 m_size = { 1.0f, 1.0f, 1.0f };
		glm::vec3 m_center_coords = { 0.0f, 0.0f, 0.0f };
		glm::vec3 m_rotations = { 0.0f, 0.0f, 0.0f };
		int m_file_index = 0;
		MaterialProperties m_material_props;
		int m_layer_number = 0;
		bool m_use_realistic_stretch = true;
		bool m_use_realistic_bending = true;
	}
	m_new_cloth_parameters;

	struct
	{
		glm::vec3 m_size = { 1.0f, 1.0f, 1.0f };
		glm::vec3 m_center_coords = { 0.0f, 0.0f, 0.0f };
		glm::vec3 m_rotations = { 0.0f, 0.0f, 0.0f };
		int m_file_index = 0;
		float m_friction_coeff = 0.5f;
	}
	m_new_collider_parameters;

	FixedVertexTask m_new_fixed_vertex;
	FixedAngleTask m_new_fixed_angle;
	SewTask m_new_sew_task;

	struct
	{
		std::vector<glm::vec3> m_coords;
		std::vector<glm::uvec3> m_indices;
		float m_average_edge_length = 0.0f;
	}
	m_candidate_object;

	struct
	{
		// frames per second
		std::vector<uint16_t> m_framerate;

#ifdef MEASURE_TIME
		// miliseconds
		std::vector<uint16_t> m_rtree_creation_time;
		std::vector<uint16_t> m_find_collisions_candidates_time;
		std::vector<uint16_t> m_check_collisions_candidates_time;
		std::vector<uint16_t> m_collisions_constraints_graph_time;
		std::vector<uint16_t> m_user_constraints_graph_time;
		std::vector<uint16_t> m_evaluate_constraints_time;
		std::vector<uint16_t> m_evaluate_friction_time;
#endif

		// counts
		uint32_t m_internal_constraints_count = 0u;
		uint32_t m_phantom_constraints_count = 0u;
		std::vector<uint32_t> m_collision_constraints_count;
		uint32_t m_user_defined_constraints_count = 0u;
	}
	m_statistic;
};