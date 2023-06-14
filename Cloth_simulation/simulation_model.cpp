// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "simulation_model.h"
#include "check_collisions.h"
#include "evaluate_constraints.h"
#include "evaluate_friction.h"

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationModel::addFixedVertex(const std::vector<FixedVertexTask>& tasks)
{
	if (tasks.empty())
	{
		return {};
	}

	std::vector<std::vector<KeysUtils::ConstraintKey>> keys;
	keys.reserve(tasks.size());
	int constraint_id = m_user_defined_constraints.getConstraintsCount();

	for (const auto& task : tasks)
	{
		// FIXED_POSITION: 1 - one vertex
		// FIXED_POSITION: 3 - position of a vertex
		if (task.m_use_current_position)
		{
			const glm::vec3& vertex_coords = m_cloth.getCoords(task.m_vertex);
			m_user_defined_constraints.pushConstraint(ConstraintType::FIXED_POSITION, { (uint32_t)task.m_vertex }, { vertex_coords[0], vertex_coords[1], vertex_coords[2] }, false);
		}
		else
		{
			m_user_defined_constraints.pushConstraint(ConstraintType::FIXED_POSITION, { (uint32_t)task.m_vertex }, { task.m_coords[0], task.m_coords[1], task.m_coords[2] }, false);
		}

		keys.emplace_back(KeysUtils::generateConstraintKeys({ (uint32_t)task.m_vertex }, ConstraintType::FIXED_POSITION));
		KeysUtils::insertKeysToMap(keys.back(), constraint_id, m_user_defined_constraints_search_map);

		++constraint_id;
	}

	m_user_defined_constraints_changed = true;

	return keys;
}

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationModel::addFixedAngle(const std::vector<FixedAngleTask>& tasks)
{
	if (tasks.empty())
	{
		return {};
	}

	std::vector<std::vector<KeysUtils::ConstraintKey>> keys;
	keys.reserve(tasks.size());

	int constraint_id = m_user_defined_constraints.getConstraintsCount();
	std::array<uint32_t, 2> tmp_vertices;

	ConstraintsBuffers& internal_constraints = m_cloth.getInternalConstraints();
	const std::map<KeysUtils::ConstraintKey, std::vector<int>>& internal_constraints_map = m_cloth.getConstraintSearchMap();
	ConstraintType original_constraint_type;

	int original_constraints_count = 0;
	int original_constraint_id = 0;

	std::vector<uint32_t> original_constraint_vertices(2, 0u);
	for (const auto& task : tasks)
	{
		tmp_vertices = { (uint32_t)task.m_vertex_a, (uint32_t)task.m_vertex_b };

		keys.emplace_back(KeysUtils::generateConstraintKeys({ tmp_vertices[0], tmp_vertices[1] }, ConstraintType::FIXED_ANGLE));

		// find original bending constraint affecting the same edge
		original_constraints_count = 0;

		KeysUtils::ConstraintKey original_constraint_key = KeysUtils::twoVerticesToKey(tmp_vertices[0], tmp_vertices[1], ConstraintType::BEND);
		if (internal_constraints_map.contains(original_constraint_key))
		{
			const std::vector<int>& bending_constraints = internal_constraints_map.at(original_constraint_key);
			original_constraints_count += (int)bending_constraints.size();
			original_constraint_id = bending_constraints[0];
			original_constraint_type = ConstraintType::BEND;
		}

		original_constraint_key = KeysUtils::twoVerticesToKey(tmp_vertices[0], tmp_vertices[1], ConstraintType::REALISTIC_BEND);
		if (internal_constraints_map.contains(original_constraint_key))
		{
			const std::vector<int>& realistic_bending_constraints = internal_constraints_map.at(original_constraint_key);
			original_constraints_count += (int)realistic_bending_constraints.size();
			original_constraint_id = realistic_bending_constraints[0];
			original_constraint_type = ConstraintType::REALISTIC_BEND;
		}

		if (original_constraints_count != 1)
		{
			throw std::exception("Error while looking  for original bending constraint");
		}
		internal_constraints.disableConstraint(original_constraint_id);

		// get vertices original constraint affected to
		const uint32_t* start_ptr = internal_constraints.getConstraintVertices(original_constraint_id);
		original_constraint_vertices = { start_ptr, start_ptr + CONSTRAINT_UINT_COUNT[(int)original_constraint_type] };
		m_cloth.replacePhantomsWithOriginals(original_constraint_vertices);
		if (original_constraint_type == ConstraintType::REALISTIC_BEND)
		{
			std::swap(original_constraint_vertices[0], original_constraint_vertices[1]);
		}

		// create new constraint
		// FIXED_ANGLE: 4 - two triangles with common edge
		// FIXED_ANGLE: 1 - optimal angle between triangles, 1 - opposite stiffness, 1 - lambda for XPBD
		m_user_defined_constraints.pushConstraint(ConstraintType::FIXED_ANGLE, original_constraint_vertices, { task.m_angle, 1.0f / task.m_stiffness, 0.0f }, false);
		KeysUtils::insertKeysToMap(keys.back(), constraint_id, m_user_defined_constraints_search_map);

		++constraint_id;
	}

	m_user_defined_constraints_changed = true;

	return keys;
}

std::vector<std::vector<KeysUtils::ConstraintKey>> SimulationModel::addSewing(const std::vector<SewTask>& tasks)
{
	if (tasks.empty())
	{
		return {};
	}

	std::vector<std::vector<KeysUtils::ConstraintKey>> keys;
	keys.reserve(tasks.size());
	int constraint_id = m_user_defined_constraints.getConstraintsCount();
	std::vector<uint32_t> constraint_vertices(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::SEW_VERTICES]);

	for (const auto& task : tasks)
	{
		constraint_vertices = { (uint32_t)task.m_vertex_a, (uint32_t)task.m_vertex_b };

		m_user_defined_constraints.pushConstraint(ConstraintType::SEW_VERTICES, constraint_vertices, { 1.0f / task.m_sewing_stiffness, task.m_distance_to_join_vertices, 0.0f }, false);
		keys.emplace_back(KeysUtils::generateConstraintKeys(constraint_vertices, ConstraintType::SEW_VERTICES));
		KeysUtils::insertKeysToMap(keys.back(), constraint_id, m_user_defined_constraints_search_map);

		++constraint_id;
	}

	m_user_defined_constraints_changed = true;

	return keys;
}

const ExecutionStatistic& SimulationModel::simulationStep(float time_delta)
{

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_start_time = std::chrono::high_resolution_clock::now();
#endif

	m_max_movement = evaluateExternalForces(time_delta);

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_evaluate_forces_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	generateRTree(m_max_movement);

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_rtree_creation_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	findCollisionsCandidates();

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_find_collisions_candidates_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	checkCollisionsCandidates(m_max_movement);

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_check_collisions_candidates_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	m_collisions_constraints_graph.setConstraints(m_collisions_constraints, m_cloth.getRealVerticesCount());

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_collisions_constraints_graph_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	if (m_user_defined_constraints_changed)
	{
		m_user_defined_constraints_graph.setConstraints(m_user_defined_constraints, m_cloth.getRealVerticesCount());
		m_user_defined_constraints_changed = false;
	}

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_user_constraints_graph_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	evaluateConstraints(time_delta);

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_evaluate_constraints_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	updatePositionsAndSpeeds(time_delta);

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_update_positions_and_speeds_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
	m_start_time = m_end_time;
#endif

	m_cloth.updateNormals();

#if defined(MEASURE_TIME) || defined(PERFORMANCE_TEST)
	m_end_time = std::chrono::high_resolution_clock::now();
	m_statistic.m_update_normals_time =
		(uint16_t)std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time).count();
#endif

	m_statistic.m_internal_constraints_count = (uint32_t)m_cloth.getInternalConstraintsCount();
	m_statistic.m_phantom_constraints_count = (uint32_t)m_cloth_constraints_graph.getPhantomConstraintsCount();
	m_statistic.m_collision_constraints_count = (uint32_t)m_collisions_constraints_graph.getNodesCount();
	m_statistic.m_user_defined_constraints_count = (uint32_t)m_user_defined_constraints.getConstraintsCount();

	return m_statistic;
}

float SimulationModel::evaluateExternalForces(float time_delta)
{
	const AlignedVector<glm::vec3>& coords = m_cloth.getCoords();
	AlignedVector<glm::vec3>& test_coords = m_cloth.getTestCoords();
	const AlignedVector<glm::vec3> speeds = m_cloth.getSpeeds();

	const float gravity_movement = -time_delta * time_delta * m_settings.m_gravity;

	glm::vec3 movement = { 0.0f, 0.0f, 0.0f };
	float max_movement = 0.0f;

	for (int i = 0; i < m_cloth.getRealVerticesCount(); ++i)
	{
		movement = time_delta * speeds[i];
		movement[1] += gravity_movement;
		test_coords[i] = coords[i] + movement;

		max_movement = std::max({ max_movement, fabsf(movement[0]),fabsf(movement[1]),fabsf(movement[2]) });
	}

	m_cloth.syncPhantomVertices();

	return max_movement;
}

void SimulationModel::findCollisionsCandidates()
{
	m_collisions_constraints.clear();
	//for (auto& elem : m_collisions_candidates)
	//{
	//	elem.clear();
	//}

	// std::array<int, 2> self_collision_key = { 0, 0 };
	// std::set<std::array<int, 2>> done_self_collisions;

	const RTree::RTreeNodes& r_tree_nodes = m_r_tree.getNodes();
	const std::vector<int>& cloth_triangle_nodes = m_r_tree.getClothPrimitivesNodes();

	// we will test all cloth triangles
	for (auto cloth_triangle_node : cloth_triangle_nodes)
	{
		const int current_triangle_id = r_tree_nodes.getPrimitiveID(cloth_triangle_node);
		const std::vector<int> collided_triangles = m_r_tree.findCollisionsWithBndBox(r_tree_nodes.getBndBox(cloth_triangle_node));

		for (const auto collided_triangle_node : collided_triangles)
		{
			const int collided_triangle_id = r_tree_nodes.getPrimitiveID(collided_triangle_node);
			if (collided_triangle_id == current_triangle_id)
			{
				continue;
			}

			if (r_tree_nodes.getObjectType(collided_triangle_node) == RTree::ObjectType::CLOTH)
			{
				// self_collision_key = { std::min<int>(cloth_triangle_node, collided_triangle_node), std::max<int>(cloth_triangle_node, collided_triangle_node) };
				// if (done_self_collisions.contains(self_collision_key))
				// {
					// continue;
				// }

				// done_self_collisions.insert(self_collision_key);
				createSelfCollisionCandidates(current_triangle_id, collided_triangle_id);
			}
			else
			{
				createColliderCollisionCandidate(current_triangle_id, collided_triangle_id);
			}
		}
	}
}

void SimulationModel::createSelfCollisionCandidates(int triangle_a, int triangle_b)
{
	// get data about triangles
	glm::uvec3 triangle_a_indices = m_cloth.getIndices(triangle_a);
	glm::uvec3 triangle_b_indices = m_cloth.getIndices(triangle_b);

	for (int i = 0; i < 3; ++i)
	{
		triangle_a_indices[i] = m_cloth.getHostOrOriginalVertex(triangle_a_indices[i]);
		triangle_b_indices[i] = m_cloth.getHostOrOriginalVertex(triangle_b_indices[i]);
	}

	if (triangle_a_indices[0] == triangle_b_indices[0] || triangle_a_indices[0] == triangle_b_indices[1] || triangle_a_indices[0] == triangle_b_indices[2] ||
		triangle_a_indices[1] == triangle_b_indices[0] || triangle_a_indices[1] == triangle_b_indices[1] || triangle_a_indices[1] == triangle_b_indices[2] ||
		triangle_a_indices[2] == triangle_b_indices[0] || triangle_a_indices[2] == triangle_b_indices[1] || triangle_a_indices[2] == triangle_b_indices[2])
	{
		return;
	}

	const PrimitivesOwnershipUtils::TrianglePrimitivesOwnership triangle_a_primitives_ownership =
		m_cloth.getTrianglePrimitivesOwnership(triangle_a);
	const PrimitivesOwnershipUtils::TrianglePrimitivesOwnership triangle_b_primitives_ownership =
		m_cloth.getTrianglePrimitivesOwnership(triangle_b);

	// here we try to create 6 vertex-triangle collisions candidates
	constexpr int data_array_size = CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE];
	for (int vertex_id = 0; vertex_id < 3; ++vertex_id)
	{
		// SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE: 1 - vertex, 1 - triangle, 2 - zeros
		if (PrimitivesOwnershipUtils::triangleOwnsTheVertex(vertex_id, triangle_a_primitives_ownership))
		{
			// m_collisions_candidates[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE].push_back(m_collisions_constraints.getConstraintsCount());
			m_collisions_constraints.pushConstraint(ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE,
				{ (uint32_t)triangle_a_indices[vertex_id], (uint32_t)triangle_b, 0u, 0u }, std::vector<float>(data_array_size, 0.0f), true);
		}

		if (PrimitivesOwnershipUtils::triangleOwnsTheVertex(vertex_id, triangle_b_primitives_ownership))
		{
			// m_collisions_candidates[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE].push_back(m_collisions_constraints.getConstraintsCount());
			m_collisions_constraints.pushConstraint(ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE,
				{ (uint32_t)triangle_b_indices[vertex_id],(uint32_t)triangle_a, 0u, 0u }, std::vector<float>(data_array_size, 0.0f), true);
		}
	}

	// here we try to create 9 edge-edge constraints
	constexpr int data_array_size_2 = CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE];
	for (uint8_t triangle_a_edge_index = 0; triangle_a_edge_index < 3; ++triangle_a_edge_index)
	{
		for (uint8_t triangle_b_edge_index = 0; triangle_b_edge_index < 3; ++triangle_b_edge_index)
		{
			if (PrimitivesOwnershipUtils::triangleOwnsTheEdge(triangle_a_edge_index, triangle_a_primitives_ownership) &&
				PrimitivesOwnershipUtils::triangleOwnsTheEdge(triangle_b_edge_index, triangle_b_primitives_ownership))
			{
				// m_collisions_candidates[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE].push_back(m_collisions_constraints.getConstraintsCount());
				m_collisions_constraints.pushConstraint(
					ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE,
					{ (uint32_t)triangle_a, (uint32_t)triangle_a_edge_index, (uint32_t)triangle_b, (uint32_t)triangle_b_edge_index },
					std::vector<float>(data_array_size_2, 0.0f), true);
			}
		}
	}
}

void SimulationModel::createColliderCollisionCandidate(int cloth_triangle_id, int collider_triangle_id)
{
	glm::uvec3 cloth_triangle_indices = m_cloth.getIndices(cloth_triangle_id);
	for (int i = 0; i < 3; ++i)
	{
		cloth_triangle_indices[i] = m_cloth.getHostOrOriginalVertex(cloth_triangle_indices[i]);
	}

	const glm::uvec3& collider_triangle_indices = m_collider.getIndices(collider_triangle_id);

	const PrimitivesOwnershipUtils::TrianglePrimitivesOwnership cloth_triangle_primitives_ownership =
		m_cloth.getTrianglePrimitivesOwnership(cloth_triangle_id);
	const PrimitivesOwnershipUtils::TrianglePrimitivesOwnership collider_triangle_primitives_ownership =
		m_collider.getTrianglePrimitivesOwnership(collider_triangle_id);

	// here we try to create 3 vertex-triangle collisions
	constexpr int data_array_size = CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE];
	for (int vertex_id = 0; vertex_id < 3; ++vertex_id)
	{
		if (PrimitivesOwnershipUtils::triangleOwnsTheVertex(vertex_id, cloth_triangle_primitives_ownership))
		{
			// m_collisions_candidates[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE].push_back(m_collisions_constraints.getConstraintsCount());
			m_collisions_constraints.pushConstraint(ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE,
				{ cloth_triangle_indices[0], (uint32_t)collider_triangle_id }, std::vector<float>(data_array_size, 0.0f), true);
		}
	}

	// here we try to create 9 edge-edge constraints
	constexpr int data_array_size_2 = CONSTRAINT_FLOAT_COUNT[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE];
	for (int cloth_triangle_edge = 0; cloth_triangle_edge < 3; ++cloth_triangle_edge)
	{
		for (int object_triangle_edge = 0; object_triangle_edge < 3; ++object_triangle_edge)
		{
			if (PrimitivesOwnershipUtils::triangleOwnsTheEdge(cloth_triangle_edge, cloth_triangle_primitives_ownership) &&
				PrimitivesOwnershipUtils::triangleOwnsTheEdge(object_triangle_edge, collider_triangle_primitives_ownership))
			{
				// m_collisions_candidates[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE].push_back(m_collisions_constraints.getConstraintsCount());
				m_collisions_constraints.pushConstraint(ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE,
					{ (uint32_t)cloth_triangle_id, (uint32_t)cloth_triangle_edge, (uint32_t)collider_triangle_id, (uint32_t)object_triangle_edge }, std::vector<float>(data_array_size_2, 0.0f), true);
			}
		}
	}
}

void SimulationModel::checkCollisionsCandidates(float max_movement)
{
	const float self_collision_critical_distance = std::min(max_movement, m_settings.m_max_collision_radius_for_cloth);
	const float collider_collision_critical_distance = std::min(max_movement, m_settings.m_max_collision_radius_for_colliders);

	// we can use 
	// candidates = &m_collisions_candidates[(size_t)ConstraintType::...];
	// to get tasks of specific type and remove switch, but it is not good for CPU version

#pragma omp parallel for
	for (int i = 0; i < m_collisions_constraints.getConstraintsCount(); ++i)
	{
		switch (m_collisions_constraints.getConstraintType(i))
		{
		case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE:
		case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE:
			CheckCollisionsMultithreaded::checkSelfVertexTriangleCollision(m_cloth, m_collisions_constraints, self_collision_critical_distance, i);
			break;
		case ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE:
			CheckCollisionsMultithreaded::checkSelfEdgeEdgeCollision(m_cloth, m_collisions_constraints, self_collision_critical_distance, i);
			break;
		case ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE:
			CheckCollisionsMultithreaded::checkColliderVertexTriangleCollision(m_cloth, m_collisions_constraints, m_collider, collider_collision_critical_distance, i);
			break;
		case ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE:
			CheckCollisionsMultithreaded::checkColliderEdgeEdgeCollision(m_cloth, m_collisions_constraints, m_collider, collider_collision_critical_distance, i);
			break;
		default:
			throw std::exception("Wrong constraint type!");
			break;
		}
	}

	replaceCandidatesWithRealCollisions();
}

void SimulationModel::replaceCandidatesWithRealCollisions()
{
	const std::array<int, CONSTRAINT_TYPES_COUNT>& old_stored_constraints_count = m_collisions_constraints.getStoredConstraintsCounters();

	std::array<int, CONSTRAINT_TYPES_COUNT> new_stored_constraints_count = { 0 };
	new_stored_constraints_count[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION] =
		old_stored_constraints_count[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_CANDIDATE];
	new_stored_constraints_count[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS] =
		old_stored_constraints_count[(size_t)ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS_CANDIDATE];
	new_stored_constraints_count[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION] =
		old_stored_constraints_count[(size_t)ConstraintType::SELF_EDGE_EDGE_COLLISION_CANDIDATE];
	new_stored_constraints_count[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION] =
		old_stored_constraints_count[(size_t)ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION_CANDIDATE];
	new_stored_constraints_count[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION] =
		old_stored_constraints_count[(size_t)ConstraintType::COLLIDER_EDGE_EDGE_COLLISION_CANDIDATE];

	m_collisions_constraints.setStoredConstraintsCounters(new_stored_constraints_count);
}

void SimulationModel::evaluateConstraints(float time_delta)
{
#pragma omp parallel
	{
		const float alpha_correction_coeff = 1.0f / (time_delta * time_delta);

		for (int iteration = 0; iteration < m_settings.m_iterations_count; ++iteration)
		{
			evaluateInternalConstraints(alpha_correction_coeff, iteration);

#pragma omp single
			{
				m_cloth.syncOriginalsVertices();
			}

			evaluateCollisionConstraints(alpha_correction_coeff, iteration);

			evaluateUserConstraints(alpha_correction_coeff, iteration);

#pragma omp single
			{
				m_cloth.syncPhantomVertices();
			}
		}

		evaluateFriction(time_delta);
	}
}

void SimulationModel::evaluateInternalConstraints(float alpha_correction_coeff, int iteration)
{
	const int thread_id = omp_get_thread_num();

	const std::vector<int>* tasks = nullptr;
	const ConstraintsBuffers& buffer = m_cloth.getInternalConstraints();
	const TasksMap& tasks_map = m_cloth_constraints_graph.getTasksMap();

	for (int partition = 0; partition < tasks_map.getPartitionsCount() - 1; ++partition)
	{
		tasks = &tasks_map.getTasks(partition);
		for (int i = thread_id; i < tasks->size(); i += THREADS_COUNT)
		{
			switch (buffer.getConstraintType((*tasks)[i]))
			{
			case ConstraintType::STRETCH:
				EvaluateConstraintsMultithread::evaluateStretch(m_cloth, alpha_correction_coeff, iteration, (*tasks)[i]);
				break;
			case ConstraintType::REALISTIC_STRETCH:
				EvaluateConstraintsMultithread::evaluateRealisticStretch(m_cloth, alpha_correction_coeff, iteration, (*tasks)[i]);
				break;
			case ConstraintType::BEND:
				if (!buffer.getIsDisabled((*tasks)[i]))
				{
					EvaluateConstraintsMultithread::evaluateBending(m_cloth, alpha_correction_coeff, iteration, (*tasks)[i]);
				}
				break;
			case ConstraintType::REALISTIC_BEND:
				if (!buffer.getIsDisabled((*tasks)[i]))
				{
					EvaluateConstraintsMultithread::evaluateRealisticBending(m_cloth, alpha_correction_coeff, iteration, (*tasks)[i]);
				}
				break;
			default:
				throw std::exception("Wrong constraint type");
			}
		}

#pragma omp barrier
	}

#pragma omp single
	{
		tasks = &tasks_map.getTasks(tasks_map.getPartitionsCount() - 1);
		for (int i = 0; i < tasks->size(); ++i)
		{
			EvaluateConstraintsMultithread::evaluatePhantom(m_cloth, m_cloth_constraints_graph.getPhantomConstraints(), (*tasks)[i]);
		}
	}
}

void SimulationModel::evaluateCollisionConstraints(float alpha_correction_coeff, int iteration)
{
	const int thread_id = omp_get_thread_num();

	const std::vector<int>* tasks = nullptr;
	const ConstraintsBuffers& buffer = m_collisions_constraints;
	const TasksMap& tasks_map = m_collisions_constraints_graph.getTasksMap();

	for (int partition = 0; partition < tasks_map.getPartitionsCount(); ++partition)
	{
		tasks = &tasks_map.getTasks(partition);
		for (int i = thread_id; i < tasks->size(); i += THREADS_COUNT)
		{
			switch (buffer.getConstraintType((*tasks)[i]))
			{
			case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION:
				EvaluateConstraintsMultithread::evaluateSelfVertexTriangleCollision(m_cloth, m_collisions_constraints, (*tasks)[i]);
				break;
			case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS:
				EvaluateConstraintsMultithread::evaluateSelfVertexTriangleCollisionBetweenLayers(m_cloth, m_collisions_constraints, (*tasks)[i]);
				break;
			case ConstraintType::SELF_EDGE_EDGE_COLLISION:
				EvaluateConstraintsMultithread::evaluateSelfEdgeEdgeCollision(m_cloth, m_collisions_constraints, (*tasks)[i]);
				break;
			case ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION:
				EvaluateConstraintsMultithread::evaluateColliderVertexTriangleCollision(m_cloth, m_collider, m_collisions_constraints, (*tasks)[i]);
				break;
			case ConstraintType::COLLIDER_EDGE_EDGE_COLLISION:
				EvaluateConstraintsMultithread::evaluateColliderEdgeEdgeCollision(m_cloth, m_collider, m_collisions_constraints, (*tasks)[i]);
				break;
			default:
				throw std::exception("Wrong constraint type");
				break;
			}
		}

#pragma omp barrier
	}
}

void SimulationModel::evaluateUserConstraints(float alpha_correction_coeff, int iteration)
{
	const int thread_id = omp_get_thread_num();

	const std::vector<int>* tasks = nullptr;
	const ConstraintsBuffers& buffer = m_user_defined_constraints;
	const TasksMap& tasks_map = m_user_defined_constraints_graph.getTasksMap();

	for (int partition = 0; partition < tasks_map.getPartitionsCount(); ++partition)
	{
		tasks = &tasks_map.getTasks(partition);
		for (int i = thread_id; i < tasks->size(); i += THREADS_COUNT)
		{
			switch (buffer.getConstraintType((*tasks)[i]))
			{
			case ConstraintType::SEW_VERTICES:
				EvaluateConstraintsMultithread::evaluateSewing(m_cloth, m_user_defined_constraints, alpha_correction_coeff, i, (*tasks)[i]);
				break;
			case ConstraintType::FIXED_POSITION:
				EvaluateConstraintsMultithread::evaluateFixedPosition(m_cloth, m_user_defined_constraints, (*tasks)[i]);
				break;
			case ConstraintType::FIXED_ANGLE:
				EvaluateConstraintsMultithread::evaluateFixedAngle(m_cloth, m_user_defined_constraints, alpha_correction_coeff, i, (*tasks)[i]);
				break;
			default:
				throw std::exception("Wrong constraint type");
				break;
			}
		}

#pragma omp barrier
	}
}

void SimulationModel::evaluateFriction(float time_delta)
{
	const int thread_id = omp_get_thread_num();

	const std::vector<int>* tasks = nullptr;
	const ConstraintsBuffers& buffer = m_collisions_constraints;
	const TasksMap& tasks_map = m_collisions_constraints_graph.getTasksMap();

	for (int partition = 0; partition < tasks_map.getPartitionsCount(); ++partition)
	{
		tasks = &tasks_map.getTasks(partition);
		for (int i = thread_id; i < tasks->size(); i += THREADS_COUNT)
		{
			switch (buffer.getConstraintType((*tasks)[i]))
			{
			case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION:
			case ConstraintType::SELF_VERTEX_TRIANGLE_COLLISION_BETWEEN_LAYERS:
				EvaluateFrictionMultithreaded::evaluateSelfFriction(m_cloth, buffer, (*tasks)[i], time_delta);
				break;
			case ConstraintType::COLLIDER_VERTEX_TRIANGLE_COLLISION:
				EvaluateFrictionMultithreaded::evaluateColliderFriction(m_cloth, m_collider, buffer, (*tasks)[i], time_delta);
				break;
			case ConstraintType::SELF_EDGE_EDGE_COLLISION:
			case ConstraintType::COLLIDER_EDGE_EDGE_COLLISION:
				break;
			default:
				throw std::exception("Wrong constraint type");
				break;
			}
		}

#pragma omp barrier
	}
}

void SimulationModel::updatePositionsAndSpeeds(float time_delta)
{
	// firstly we work only with real cloth vertices
	const int real_vertices_count = m_cloth.getRealVerticesCount();

	// update vertices speeds
	const float speeds_coefficient = m_settings.m_speed_damping_coefficient / time_delta;
	AlignedVector<glm::vec3>& coords = m_cloth.getCoords();
	AlignedVector<glm::vec3>& test_coords = m_cloth.getTestCoords();
	AlignedVector<glm::vec3>& speeds = m_cloth.getSpeeds();

	m_cloth.syncSlaveVertices();

	for (int i = 0; i < real_vertices_count; ++i)
	{
		speeds[i] = speeds_coefficient * (test_coords[i] - coords[i]);
	}

	std::swap(coords, test_coords);
}
