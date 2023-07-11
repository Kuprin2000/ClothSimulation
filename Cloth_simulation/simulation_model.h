#pragma once
#include "r_tree.h"
#include "constraints_graph.h"
#include "collider.h"
#include "defines.h"

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
#include <chrono>
#endif

struct FixedVertexTask
{
	int m_vertex = 0;
	bool m_use_current_position = true;
	glm::vec3 m_coords = { 0.0f, 0.0f, 0.0f };
};

struct FixedAngleTask
{
	int m_vertex_a = 0;
	int m_vertex_b = 0;
	float m_angle = 0.0f;
	float m_stiffness = 0.0f;
};

struct SewTask
{
	int m_vertex_a = 0;
	int m_vertex_b = 0;
	float m_sewing_stiffness = 0.0f;
	float m_distance_to_join_vertices = 0.0f;
};

struct ExecutionStatistic
{
#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	// miliseconds
	uint16_t m_evaluate_forces_time = 0u;
	uint16_t m_rtree_creation_time = 0u;
	uint16_t m_find_collisions_candidates_time = 0u;
	uint16_t m_check_collisions_candidates_time = 0u;
	uint16_t m_collisions_constraints_graph_time = 0u;
	uint16_t m_user_constraints_graph_time = 0u;
	uint16_t m_evaluate_constraints_time = 0u;
	uint16_t m_update_positions_and_speeds_time = 0u;
	uint16_t m_update_normals_time = 0u;
#endif

	// counts
	uint32_t m_internal_constraints_count = 0u;
	uint32_t m_phantom_constraints_count = 0u;
	uint32_t m_collision_constraints_count = 0u;
	uint32_t m_user_defined_constraints_count = 0u;
};

struct ChangedParameters
{
	bool m_need_recreate_r_tree = false;
	bool m_need_recreate_internal_constraints_graphs = false;
	bool m_some_params_changed = false;
};

class SimulationModel
{
public:
	SimulationModel() :
		m_cloth({}, {}, {}, MaterialProperties(), 0, false, false),
		m_cloth_constraints_graph(ConstraintsGraphSettings(m_settings.m_preferred_partitions_count, (THREADS_COUNT != 1), true)), m_collider({}, {}, 0.0f),
		m_r_tree(m_settings.m_r_tree_min, m_settings.m_r_tree_max), m_collisions_constraints_graph(ConstraintsGraphSettings(INT_MAX, (THREADS_COUNT != 1), false)),
		m_user_defined_constraints_graph(ConstraintsGraphSettings(INT_MAX, (THREADS_COUNT != 1), false))
	{}

	void notifySettingsUpdated(ChangedParameters parameters)
	{
		if (parameters.m_need_recreate_r_tree)
		{
			m_r_tree = RTree(m_settings.m_r_tree_min, m_settings.m_r_tree_max);
		}

		if (parameters.m_need_recreate_internal_constraints_graphs)
		{
			m_cloth_constraints_graph =
				ConstraintsGraph(ConstraintsGraphSettings(m_settings.m_preferred_partitions_count, (THREADS_COUNT != 1), true));
		}
	}

	_NODISCARD ModelSettings& getSettings()
	{
		return m_settings;
	}

	SimulationModel(const SimulationModel&) = delete;

	void addClothParts(const std::vector<Cloth>& parts)
	{
		if (parts.empty())
		{
			return;
		}

		m_cloth.pushClothes(parts);
	}

	_NODISCARD bool isEmpty()
	{
		return !m_cloth.getPartsCount();
	}

	void addColliders(const std::vector<Collider>& colliders)
	{
		if (colliders.empty())
		{
			return;
		}

		m_collider.pushColliders(colliders);
	}

	_NODISCARD std::vector<std::vector<KeysUtils::ConstraintKey>> addFixedVertex(const std::vector<FixedVertexTask>& tasks);

	_NODISCARD std::vector<std::vector<KeysUtils::ConstraintKey>> addFixedAngle(const std::vector<FixedAngleTask>& tasks);

	_NODISCARD std::vector<std::vector<KeysUtils::ConstraintKey>> addSewing(const std::vector<SewTask>& tasks);

	void prepareForSimulation()
	{
		m_cloth_constraints_graph.setConstraints(m_cloth.getInternalConstraints(), m_cloth.getRealVerticesCount());

#pragma omp single
		{
			m_cloth.setPhantomVertices(m_cloth_constraints_graph.getReplacedVertices(), m_cloth_constraints_graph.getPhantomVerticesCount());
		}
	}

	_NODISCARD const ExecutionStatistic& simulationStep(float time_delta);

	_NODISCARD const Cloth& getCloth() const
	{
		return m_cloth;
	}

	_NODISCARD const Collider& getCollider() const
	{
		return m_collider;
	}

private:
	_NODISCARD float evaluateExternalForces(float time_delta);

	void generateRTree(float max_movement)
	{
		m_r_tree.clear();
		m_r_tree.insertMovingTriangles(m_cloth.getCoords(), m_cloth.getTestCoords(),
			m_cloth.getIndices(), 0, RTree::ObjectType::CLOTH, std::min(max_movement, m_settings.m_max_collision_radius_for_cloth));
		m_r_tree.insertTriangles(m_collider.getCoords(), m_collider.getIndices(), 0, RTree::ObjectType::COLLIDER, 0.0f);
	}

	void findCollisionsCandidates();

	void createSelfCollisionCandidates(int triangle_a, int triangle_b, ConstraintsBuffers& buffer);

	void createColliderCollisionCandidate(int cloth_triangle_id, int collider_triangle_id, ConstraintsBuffers& buffer);

	void checkCollisionsCandidates(float max_movement);

	void replaceCandidatesWithRealCollisions();

	void evaluateConstraints(float time_delta);

	void evaluateInternalConstraints(float alpha_correction_coeff, int iteration);

	void evaluateCollisionConstraints(float alpha_correction_coeff, int iteration);

	void evaluateUserConstraints(float alpha_correction_coeff, int iteration);

	void evaluateFriction(float time_delta);

	void updatePositionsAndSpeeds(float time_delta);

private:
	ModelSettings m_settings;
	bool m_user_defined_constraints_changed = true;

	Cloth m_cloth;
	ConstraintsGraph m_cloth_constraints_graph;

	Collider m_collider;
	RTree m_r_tree;

	// AlignedVector<int> m_collisions_candidates[CONSTRAINT_TYPES_COUNT];
	ConstraintsBuffers m_collisions_constraints;
	ConstraintsGraph m_collisions_constraints_graph;

	ConstraintsBuffers m_user_defined_constraints;
	ConstraintsGraph m_user_defined_constraints_graph;
	std::map<KeysUtils::ConstraintKey, std::vector<int>> m_user_defined_constraints_search_map;

	ExecutionStatistic m_statistic;

	float m_max_movement = 0.0f;

	std::array<ConstraintsBuffers, THREADS_COUNT> m_tmp_collisions_buffers;

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	std::chrono::steady_clock::time_point m_start_time = std::chrono::high_resolution_clock::now();
	std::chrono::steady_clock::time_point m_end_time = std::chrono::high_resolution_clock::now();
#endif
};