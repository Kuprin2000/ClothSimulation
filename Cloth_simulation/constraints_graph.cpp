// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "constraints_graph.h"
#include <cmath>
#include <algorithm>

void ConstraintsGraph::setConstraints(ConstraintsBuffers& constraints, int vertices_count)
{
	clear();
	m_nodes.generateNodes(constraints);
	m_original_vertices_count = vertices_count;

	if (m_settings.m_need_create_partitions)
	{
		createEdges();

		// simplify graph

		if (!m_nodes.getNodesCount())
		{
			return;
		}

		if (m_settings.m_need_insert_phantoms)
		{
			const std::vector<std::set<int>> cliques = bronKerboschDegeneracy();
			if (!cliques.empty() && (cliques[0].size() <= CONSTRAINT_VERTICES_COUNT[(int)ConstraintType::PHANTOM_VERTICES]))
			{
				replaceVertices(cliques);
				generatePhantomConstraintsBuffers();

				clearEdges();
				createEdges();
			}
		}

		colorGraph();
	}

	createMap();
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
		const uint32_t* current_node_vertices = m_nodes.getVertices(i);

		for (int j = 0; j < CONSTRAINT_VERTICES_COUNT[m_nodes.getType(i)]; ++j)
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

std::vector<std::set<int>> ConstraintsGraph::bronKerboschDegeneracy()
{
	// you can read about this algorithm in "Accelerating the Bron-Kerbosch
	// Algorithm for Maximal Clique Enumeration Using GPUs"

	const std::vector<int> sorted_nodes = sortDegeneracy();
	std::set<int> P = { sorted_nodes.begin(), sorted_nodes.end() };
	std::set<int> X = {};
	std::set<int> singleton = {};
	std::vector<std::set<int>> result = {};

	for (const auto node : sorted_nodes)
	{
		singleton = { node };
		P.erase(node);

		bronKerboschPivot(singleton, MathUtils::intersectSets(&P, &m_nodes.getNeighbors(node)),
			MathUtils::intersectSets(&X, &m_nodes.getNeighbors(node)), result);

		X.insert(node);
	}

	std::sort(result.begin(), result.end(),
		[](const std::set<int>& a, const std::set<int>& b) -> bool
		{
			return a.size() > b.size();
		});

	return result;
}

void ConstraintsGraph::bronKerboschPivot(const std::set<int>& clique, std::set<int> candidates,
	std::set<int> duplicates, std::vector<std::set<int>>& results) const
{
	// you can read about this algorithm in "Accelerating the Bron-Kerbosch
	// Algorithm for Maximal Clique Enumeration Using GPUs"

	const std::set<int> united_sets = MathUtils::uniteSets(&candidates, &duplicates);
	if (united_sets.empty())
	{
		if (clique.size() > m_settings.m_preferred_partitions_count)
		{
			results.push_back(clique);
		}
		return;
	}

	std::set<int> intersected_sets = {};
	int pivot = 0;
	int max_size = INT_MIN;

	for (auto iter = united_sets.begin(); iter != united_sets.end(); ++iter)
	{
		intersected_sets = MathUtils::intersectSets(&candidates, &m_nodes.getNeighbors(*iter));
		if ((int)intersected_sets.size() > max_size)
		{
			max_size = (int)intersected_sets.size();
			pivot = *iter;
		}
	}

	std::set<int> subtracted_sets = MathUtils::subtractSet(candidates, m_nodes.getNeighbors(pivot));

	std::set<int> singleton = {};
	while (!subtracted_sets.empty())
	{
		singleton = { *subtracted_sets.begin() };
		const std::set<int>& current_node_neighbors = m_nodes.getNeighbors(*subtracted_sets.begin());

		bronKerboschPivot(MathUtils::uniteSets(&clique, &singleton), MathUtils::intersectSets(&candidates, &current_node_neighbors),
			MathUtils::intersectSets(&duplicates, &current_node_neighbors), results);

		candidates = MathUtils::subtractSet(candidates, singleton);
		duplicates = MathUtils::uniteSets(&duplicates, &singleton);
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

		node_neighbors_count = (int)m_nodes.getNeighbors(i).size();
		m_nodes.setCounter(node_neighbors_count, i);

		max_neighbor_count = std::max(max_neighbor_count, node_neighbors_count);
	}

	std::vector<std::set<int>> buckets(max_neighbor_count + 1);

	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		buckets[(int)m_nodes.getNeighbors(i).size()].insert(i);
	}

	std::set<int>* current_basket = nullptr;
	int current_node = 0;

	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		for (auto& basket : buckets)
		{
			if (!basket.empty())
			{
				current_basket = &basket;
				break;
			}
		}

		current_node = *current_basket->begin();
		current_basket->erase(current_node);

		result[insert_index] = current_node;
		--insert_index;
		m_nodes.setIsOrdered(true, current_node);

		const std::set<int>& current_node_neighbors = m_nodes.getNeighbors(current_node);
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

int ConstraintsGraph::findCommonVertex(const std::set<int>& clique) const
{
	const uint32_t* current_node_vertices = m_nodes.getVertices(*clique.begin());
	int current_node_vertices_count = CONSTRAINT_VERTICES_COUNT[m_nodes.getType(*clique.begin())];
	std::set<uint32_t> common_vertices = { current_node_vertices, current_node_vertices + current_node_vertices_count };

	for (auto node : clique)
	{
		current_node_vertices = m_nodes.getVertices(node);
		current_node_vertices_count = CONSTRAINT_VERTICES_COUNT[m_nodes.getType(node)];

		const std::set<uint32_t> tmp_set = { current_node_vertices, current_node_vertices + current_node_vertices_count };
		common_vertices = MathUtils::intersectSets(&common_vertices, &tmp_set);
	}

	return common_vertices.empty() ? -1 : *common_vertices.begin();
}

void ConstraintsGraph::replaceVertices(const std::vector<std::set<int>>& cliques)
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

		for (auto node : clique)
		{
			node_vertices = m_nodes.getVertices(node);
			for (int i = 0; i < CONSTRAINT_VERTICES_COUNT[(int)m_nodes.getType(node)]; ++i)
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
	constraint_vertices.reserve(CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES]);

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

void ConstraintsGraph::colorGraph()
{
	// we use greedy coloring algorithm. You can read about it in
	// "Scalable Partitioning for Parallel Position Based Dynamics"

	const std::vector<int> sorted_nodes = sortDegeneracy();
	std::vector<bool> color_is_forbidden;
	int neighbour_color = 0;
	int node_color = 0;

	m_colors_count = 1;
	for (auto node : sorted_nodes)
	{
		// we color node to NO_COLOR to show that is can be put to any partition
		if (m_nodes.getNeighbors(node).empty())
		{
			m_nodes.setColor(NO_COLOR, node);
			continue;
		}

		for (int i = 0; i < (int)color_is_forbidden.size(); ++i)
		{
			color_is_forbidden[i] = false;
		}

		const std::set<int>& node_neighbors = m_nodes.getNeighbors(node);
		for (const auto neighbor : node_neighbors)
		{
			neighbour_color = m_nodes.getColor(neighbor);
			color_is_forbidden.resize(std::max((neighbour_color == NO_COLOR) ? 0 : (neighbour_color + 1), (int)color_is_forbidden.size()));

			if (neighbour_color != NO_COLOR)
			{
				color_is_forbidden[neighbour_color] = true;
			}
		}

		node_color = 0;
		while (node_color < (int)color_is_forbidden.size() && color_is_forbidden[node_color])
		{
			++node_color;
		}

		m_nodes.setColor((uint8_t)node_color, node);

		m_colors_count = std::max(m_colors_count, node_color + 1);
	}
}

void ConstraintsGraph::createMap()
{
	// one more partition for phantom constraints
	m_tasks_map.reserve(m_colors_count + 1);

	std::array<std::vector<int>, CONSTRAINT_TYPES_COUNT> independent_tasks = {};

	// original constraints
	int8_t color = 0;
	uint8_t type = 0u;
	for (int i = 0; i < m_nodes.getNodesCount(); ++i)
	{
		color = m_settings.m_need_create_partitions ? m_nodes.getColor(i) : 0;
		type = m_nodes.getType(i);

		if (color != NO_COLOR)
		{
			m_tasks_map.pushTask(m_nodes.getConstraintID(i), color, m_nodes.getType(i));
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
		m_tasks_map.pushTask(m_nodes.getConstraintID(i), m_colors_count, (size_t)ConstraintType::PHANTOM_VERTICES);
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
	const std::array<int, CONSTRAINT_TYPES_COUNT> constraints_per_type = m_nodes.countNodesPerType();
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
			int tasks_to_add = preferred_tasks_count_per_type[type] - m_tasks_map.getTasksCount(partition, type);
			if (tasks_to_add <= 0 || !independent_tasks_count_per_type[type])
			{
				continue;
			}

			tasks_to_add = std::min(tasks_to_add, independent_tasks_count_per_type[type]);
			for (int k = 0; k < tasks_to_add; ++k)
			{
				m_tasks_map.pushTask(m_nodes.getConstraintID(independent_tasks[(size_t)type][(size_t)tasks_to_add - (size_t)k - 1u]), partition, type);
			}

			preferred_tasks_count_per_type[type] -= tasks_to_add;
			independent_tasks_count -= tasks_to_add;
		}
	}
}