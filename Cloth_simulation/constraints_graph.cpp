// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "constraints_graph.h"
#include "bool_vector.h"
#include <cmath>
#include <algorithm>

void ConstraintsGraph::setConstraints(ConstraintsBuffers& constraints, int vertices_count)
{
#pragma omp single
	{
		clear();
		m_nodes.generateNodes(constraints);
		m_original_vertices_count = vertices_count;

		if (m_settings.m_need_create_partitions)
		{
			createEdges();

			// simplify graph

			if (m_settings.m_need_insert_phantoms)
			{
				const std::vector<MathUtils::FastSet> cliques = bronKerboschDegeneracy();
				if (!cliques.empty() && (cliques[0].size() <= CONSTRAINT_UINT_COUNT[(int)ConstraintType::PHANTOM_VERTICES]))
				{
					replaceVertices(cliques);
					generatePhantomConstraintsBuffers();

					clearEdges();
					createEdges();
				}
			}
		}
	}

	if (m_settings.m_need_create_partitions)
	{
		colorGraph();
	}

#pragma omp single
	{
		//for (int i = 0; i < m_nodes.getNodesCount(); ++i)
		//{
		//	const int node_color = m_nodes.getColor(i);
		//	const std::vector<int>& neighbors = m_nodes.getNeighbors(i).getValues();
		//	for (auto neighbour : neighbors)
		//	{
		//		if (node_color == m_nodes.getColor(neighbour) && node_color != NO_COLOR)
		//		{
		//			throw std::exception("Bad color");
		//		}
		//	}
		//}

		createMap();
	}
}

ConstraintsBuffers& ConstraintsGraph::getPhantomConstraints()
{
	return m_phantom_constraints;
}

void ConstraintsGraph::clearEdges()
{
	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		m_nodes.clearNeighbors(i);
	}
}

void ConstraintsGraph::createEdges()
{
	std::vector<std::vector<int>> nodes_affecting_the_vertex(m_original_vertices_count + m_phantom_vertices_count);
	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		const uint32_t* current_node_vertices = m_nodes.getEnabledConstraintVertices(i);

		for (int j = 0; j < CONSTRAINT_UINT_COUNT[m_nodes.getEnabledConstraintType(i)]; ++j)
		{
			nodes_affecting_the_vertex[current_node_vertices[j]].push_back(i);
		}
	}

	for (const auto& nodes_group : nodes_affecting_the_vertex)
	{
		for (const auto node : nodes_group)
		{
			m_nodes.addNeighbors(nodes_group, node);
			m_nodes.deleteNeighbour(node, node);
		}
	}
}

std::vector<MathUtils::FastSet> ConstraintsGraph::bronKerboschDegeneracy()
{
	// you can read about this algorithm in "Accelerating the Bron-Kerbosch
	// Algorithm for Maximal Clique Enumeration Using GPUs"

	const std::vector<int> sorted_nodes = sortDegeneracy();
	MathUtils::FastSet P(sorted_nodes, m_nodes.getNodesCount());
	MathUtils::FastSet X(m_nodes.getNodesCount());
	MathUtils::FastSet singleton(m_nodes.getNodesCount());
	std::vector<MathUtils::FastSet> result;

	for (const auto node : sorted_nodes)
	{
		singleton = { node };
		P.erase(node);

		bronKerboschPivot(singleton, MathUtils::intersectSets(P, m_nodes.getNeighbors(node)),
			MathUtils::intersectSets(X, m_nodes.getNeighbors(node)), result);

		X.insert(node);
	}

	std::sort(result.begin(), result.end(),
		[](const MathUtils::FastSet& a, const MathUtils::FastSet& b) -> bool
		{
			return a.size() > b.size();
		});

	return result;
}

void ConstraintsGraph::bronKerboschPivot(const MathUtils::FastSet& clique, MathUtils::FastSet candidates,
	MathUtils::FastSet duplicates, std::vector<MathUtils::FastSet>& results) const
{
	// you can read about this algorithm in "Accelerating the Bron-Kerbosch
	// Algorithm for Maximal Clique Enumeration Using GPUs"

	const MathUtils::FastSet united_sets = MathUtils::uniteSets(candidates, duplicates);
	if (!united_sets.size())
	{
		if (clique.size() > m_settings.m_preferred_partitions_count)
		{
			results.push_back(clique);
		}
		return;
	}

	MathUtils::FastSet intersected_sets(m_nodes.getNodesCount());
	int pivot = 0;
	int max_size = INT_MIN;

	const std::vector<int>& united_sets_values = united_sets.getValues();
	for (auto iter = united_sets_values.begin(); iter != united_sets_values.end(); ++iter)
	{
		intersected_sets = MathUtils::intersectSets(candidates, m_nodes.getNeighbors(*iter));
		if ((int)intersected_sets.size() > max_size)
		{
			max_size = (int)intersected_sets.size();
			pivot = *iter;
		}
	}

	MathUtils::FastSet subtracted_sets = MathUtils::subtractSet(candidates, m_nodes.getNeighbors(pivot));

	MathUtils::FastSet singleton(m_nodes.getNodesCount());
	while (subtracted_sets.size())
	{
		singleton.clear();
		singleton.insert(subtracted_sets.getValues()[0]);
		const MathUtils::FastSet& current_node_neighbors = m_nodes.getNeighbors(subtracted_sets.getValues()[0]);

		bronKerboschPivot(MathUtils::uniteSets(clique, singleton), MathUtils::intersectSets(candidates, current_node_neighbors),
			MathUtils::intersectSets(duplicates, current_node_neighbors), results);

		candidates = MathUtils::subtractSet(candidates, singleton);
		duplicates = MathUtils::uniteSets(duplicates, singleton);
		subtracted_sets = MathUtils::subtractSet(candidates, m_nodes.getNeighbors(pivot));
	}
}

std::vector<int> ConstraintsGraph::sortDegeneracy()
{
	// you can read about this algorithm on Wikipedia in "Algorithms" section
	// https://en.wikipedia.org/wiki/Degeneracy_(graph_theory)
	// or in "Smallest-last ordering and clustering and graph coloring algorithms"

	std::vector<int> result(m_nodes.getNodesCount());
	int insert_index = (int)result.size() - 1;
	int node_neighbors_count = 0;
	int max_neighbor_count = INT_MIN;

	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		m_nodes.setIsOrdered(false, i);

		node_neighbors_count = m_nodes.getNeighborsCount(i);
		m_nodes.setCounter(node_neighbors_count, i);

		max_neighbor_count = std::max(max_neighbor_count, node_neighbors_count);
	}

	std::vector<MathUtils::FastSet> buckets(max_neighbor_count + 1);
	for (int i = 0; i < buckets.size(); ++i)
	{
		buckets[i] = MathUtils::FastSet(m_nodes.getNodesCount());
	}

	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		buckets[m_nodes.getNeighborsCount(i)].insert(i);
	}

	MathUtils::FastSet* current_basket = nullptr;
	int current_node = 0;

	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		for (auto& basket : buckets)
		{
			if (basket.size())
			{
				current_basket = &basket;
				break;
			}
		}

		current_node = current_basket->getValues()[0];
		current_basket->erase(current_node);

		result[insert_index] = current_node;
		--insert_index;
		m_nodes.setIsOrdered(true, current_node);

		const std::vector<int>& current_node_neighbors = m_nodes.getNeighbors(current_node).getValues();
		for (const auto neighbor : current_node_neighbors)
		{
			if (m_nodes.getIsOrdered(neighbor))
			{
				continue;
			}

			current_basket = &buckets[m_nodes.getCounter(neighbor)];
			current_basket->erase(neighbor);

			m_nodes.decreaseCounter(neighbor);
			buckets[m_nodes.getCounter(neighbor)].insert(neighbor);
		}
	}

	return result;
}

int ConstraintsGraph::findCommonVertex(const MathUtils::FastSet& clique) const
{
	const uint32_t* current_node_vertices = m_nodes.getEnabledConstraintVertices(clique.getValues()[0]);
	int current_node_vertices_count = CONSTRAINT_UINT_COUNT[m_nodes.getEnabledConstraintType(clique.getValues()[0])];
	MathUtils::FastSet common_vertices({ current_node_vertices, current_node_vertices + current_node_vertices_count }, m_original_vertices_count);

	const std::vector<int>& nodes = clique.getValues();
	for (auto node : nodes)
	{
		current_node_vertices = m_nodes.getEnabledConstraintVertices(node);
		current_node_vertices_count = CONSTRAINT_UINT_COUNT[m_nodes.getEnabledConstraintType(node)];

		const MathUtils::FastSet tmp_set({ current_node_vertices, current_node_vertices + current_node_vertices_count }, m_original_vertices_count);
		common_vertices = MathUtils::intersectSets(common_vertices, tmp_set);
	}

	return !common_vertices.size() ? -1 : common_vertices.getValues()[0];
}

void ConstraintsGraph::replaceVertices(const std::vector<MathUtils::FastSet>& cliques)
{
	int vertex_to_replace = 0;
	int phantom_vertex = m_original_vertices_count;
	uint32_t* node_vertices = nullptr;

	for (const auto& clique : cliques)
	{
		vertex_to_replace = findCommonVertex(clique);
		if (vertex_to_replace == -1)
		{
			continue;
		}

		m_replaced_vertices.emplace_back((uint32_t)vertex_to_replace, (uint32_t)phantom_vertex, (uint8_t)clique.size());

		const std::vector<int>& nodes = clique.getValues();
		for (auto node : nodes)
		{
			node_vertices = m_nodes.getEnabledConstraintVertices(node);
			for (int i = 0; i < CONSTRAINT_UINT_COUNT[(int)m_nodes.getEnabledConstraintType(node)]; ++i)
			{
				if (node_vertices[i] == vertex_to_replace)
				{
					node_vertices[i] = phantom_vertex;
					++phantom_vertex;
					++m_phantom_vertices_count;
					break;
				}
			}
		}
	}
}

void ConstraintsGraph::generatePhantomConstraintsBuffers()
{
	std::array<int, CONSTRAINT_TYPES_COUNT> constraints_per_type = { 0 };
	constraints_per_type[(size_t)ConstraintType::PHANTOM_VERTICES] = (int)m_replaced_vertices.size();
	m_phantom_constraints = ConstraintsBuffers::reserveBuffers(constraints_per_type);

	std::vector<uint32_t> constraint_vertices;
	constraint_vertices.reserve(CONSTRAINT_UINT_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES]);

	for (const auto& phantom_constraint : m_replaced_vertices)
	{
		constraint_vertices.resize(phantom_constraint.m_phantoms_count);
		for (int i = 0; i < constraint_vertices.size(); ++i)
		{
			constraint_vertices[i] = phantom_constraint.m_first_phantom_index + i;
		}

		m_phantom_constraints.pushConstraint(ConstraintType::PHANTOM_VERTICES,
			constraint_vertices, { (float)phantom_constraint.m_phantoms_count }, false);
	}
}

void ConstraintsGraph::assignColors(const std::vector<int>& conflicts, int max_degree, int thread_id)
{
	for (int i = thread_id; i < conflicts.size(); i += THREADS_COUNT)
	{
		// we keep node colored to NO_COLOR to show that is can be put to any partition
		const int node = conflicts[i];
		if (!m_nodes.getNeighborsCount(node))
		{
			continue;
		}

		BoolVector::BoolVector color_is_forbidden;
		color_is_forbidden.resize(max_degree);
		int neighbour_color = 0;
		const std::vector<int>& node_neighbors = m_nodes.getNeighbors(node).getValues();

		for (const auto neighbor : node_neighbors)
		{
			neighbour_color = m_nodes.getColor(neighbor);
			if (neighbour_color != NO_COLOR)
			{
				color_is_forbidden.setValue(true, neighbour_color);
			}
		}

		int node_color = 0;
		while (node_color < (int)color_is_forbidden.size() && color_is_forbidden[node_color])
		{
			++node_color;
		}

		m_nodes.setColor((int16_t)node_color, node);
	}

#pragma omp barrier
}

void ConstraintsGraph::detectConflicts(const std::vector<int>& conflicts, std::array<MathUtils::FastSet, THREADS_COUNT>& new_conflicts, int thread_id) const
{
	for (int i = thread_id; i < conflicts.size(); i += THREADS_COUNT)
	{
		const int node = conflicts[i];
		const std::vector<int>& node_neighbour = m_nodes.getNeighbors(node).getValues();
		for (auto neighbour : node_neighbour)
		{
			if (m_nodes.getColor(node) == m_nodes.getColor(neighbour) && neighbour < node)
			{
				new_conflicts[thread_id].insert(node);
			}
		}
	}

#pragma omp barrier
}

void ConstraintsGraph::colorGraph()
{
#pragma omp single
	{
		for (int i = 0; i < m_nodes.getNodesCount(); ++i)
		{
			m_conflicts.push_back(i);
			m_max_degree = std::max(m_max_degree, m_nodes.getNeighborsCount(i));
		}

		for (auto& elem : m_new_conflicts)
		{
			elem.updateMaxValue(m_nodes.getNodesCount());
		}
	}

	const int thread_id = omp_get_thread_num();

	while (!m_no_conflicts)
	{
		assignColors(m_conflicts, m_max_degree, thread_id);
		detectConflicts(m_conflicts, m_new_conflicts, thread_id);

#pragma omp single
		{
			MathUtils::FastSet united_conflicts(m_original_vertices_count + m_phantom_vertices_count);
			for (auto& elem : m_new_conflicts)
			{
				united_conflicts = MathUtils::uniteSets(united_conflicts, elem);
				elem.clear();
			}

			m_conflicts = united_conflicts.getValues();
			m_no_conflicts = m_conflicts.empty();
		}
	}

#pragma omp barrier
#pragma omp single
	{
		m_colors_count = 0;
		for (int i = 0; i < m_nodes.getNodesCount(); ++i)
		{
			m_colors_count = std::max(m_colors_count, (int)m_nodes.getColor(i));
		}

		++m_colors_count;

		m_conflicts.clear();

		m_no_conflicts = false;
		m_max_degree = 0;
	}
}

void ConstraintsGraph::createMap()
{
	// one more partition for phantom constraints
	m_tasks_map.reserve(m_colors_count + 1);

	std::array<std::vector<int>, CONSTRAINT_TYPES_COUNT> independent_tasks = {};

	// original constraints
	int16_t color = 0;
	uint8_t type = 0u;
	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		color = m_settings.m_need_create_partitions ? m_nodes.getColor(i) : 0;
		type = m_nodes.getEnabledConstraintType(i);

		if (color != NO_COLOR)
		{
			m_tasks_map.pushTask(m_nodes.getConstraintID(i), color);
		}
		else
		{
			independent_tasks[type].push_back(i);
		}
	}

	// if there are independent tasks we can push them in any
	// partition we want. Let's try to make partitions uniform
	if (!independent_tasks.empty())
	{
		makeMapUniform(independent_tasks);
	}

	// phantom constraints go to the last partition
	for (int i = 0; i < m_replaced_vertices.size(); ++i)
	{
		m_tasks_map.pushTask(m_nodes.getConstraintID(i), m_colors_count);
	}
}

void ConstraintsGraph::makeMapUniform(const std::array<std::vector<int>, CONSTRAINT_TYPES_COUNT>& independent_tasks)
{
	// calculate independent constraints of each type;
	int independent_tasks_count_per_type[CONSTRAINT_TYPES_COUNT] = { 0 };
	int independent_tasks_count = 0;
	for (int i = 0; i < CONSTRAINT_TYPES_COUNT; ++i)
	{
		independent_tasks_count_per_type[i] = (int)independent_tasks[i].size();
		independent_tasks_count += independent_tasks_count_per_type[i];
	}

	// calculate how many tasks of each type partitions should have
	const std::array<int, CONSTRAINT_TYPES_COUNT> constraints_per_type = m_nodes.countEnabledConstraintsPerType();
	int preferred_tasks_count_per_type[CONSTRAINT_TYPES_COUNT] = { 0 };
	for (int i = 0; i < CONSTRAINT_TYPES_COUNT; ++i)
	{
		preferred_tasks_count_per_type[i] = (int)ceilf((float)constraints_per_type[i] / (float)m_colors_count);
	}

	// put some independent tasks in each partition to reach preferred number of tasks
	for (int partition = 0; partition < m_colors_count; ++partition)
	{
		if (!independent_tasks_count)
		{
			break;
		}

		for (int type = 0; type < CONSTRAINT_TYPES_COUNT; ++type)
		{
			int tasks_to_add = preferred_tasks_count_per_type[type] - m_tasks_map.getTasksCount(partition);
			if (tasks_to_add <= 0 || !independent_tasks_count_per_type[type])
			{
				continue;
			}

			tasks_to_add = std::min(tasks_to_add, independent_tasks_count_per_type[type]);
			for (int k = 0; k < tasks_to_add; ++k)
			{
				m_tasks_map.pushTask(m_nodes.getConstraintID(independent_tasks[(size_t)type][(size_t)tasks_to_add - (size_t)k - 1u]), partition);
			}

			preferred_tasks_count_per_type[type] -= tasks_to_add;
			independent_tasks_count -= tasks_to_add;
		}
	}
}