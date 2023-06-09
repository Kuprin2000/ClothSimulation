#pragma once
#include "settings_structures.h"
#include "cloth.h"
#include "defines.h"
#include "bool_vector.h"

static const int16_t NO_COLOR = INT16_MIN;

class ConstraintGraphNodes
{
public:
	void generateNodes(ConstraintsBuffers& constraints)
	{
		for (int i = 0; i < constraints.getConstraintsCount(); ++i)
		{
			if (!constraints.getIsDisabled(i))
			{
				m_enabled_constraints.push_back(i);
			}
		}

		m_constraints_buffer = &constraints;

		m_constraints_neighbors.resize(m_enabled_constraints.size());
		for (int i = 0; i < m_constraints_neighbors.size(); ++i)
		{
			m_constraints_neighbors[i].updateMaxValue((int)m_enabled_constraints.size());
		}

		m_constraints_color.resize(m_enabled_constraints.size(), NO_COLOR);
		m_constraints_counter.resize(m_enabled_constraints.size(), 0);
		m_constraints_is_ordered.resize(m_enabled_constraints.size());
	}

	_NODISCARD int getNodesCount() const
	{
		return (int)m_enabled_constraints.size();
	}

	_NODISCARD int getConstraintID(int index) const
	{
		return m_enabled_constraints[index];
	}

	_NODISCARD uint8_t getEnabledConstraintType(int index) const
	{
		return (uint8_t)m_constraints_buffer->getConstraintType(m_enabled_constraints[index]);
	}

	_NODISCARD std::array<int, CONSTRAINT_TYPES_COUNT> countEnabledConstraintsPerType() const
	{
		std::array<int, CONSTRAINT_TYPES_COUNT> result = { 0 };
		for (int i = 0; i < m_enabled_constraints.size(); ++i)
		{
			++result[(uint8_t)m_constraints_buffer->getConstraintType(m_enabled_constraints[i])];
		}

		return result;
	}

	_NODISCARD int16_t getColor(int index) const
	{
		return m_constraints_color[index];
	}

	_NODISCARD int getCounter(int index) const
	{
		return m_constraints_counter[index];
	}

	_NODISCARD bool getIsOrdered(int index) const
	{
		return m_constraints_is_ordered[index];
	}

	void setColor(int16_t new_color, int index)
	{
		m_constraints_color[index] = new_color;
	}

	void setCounter(int new_counter, int index)
	{
		m_constraints_counter[index] = new_counter;
	}

	void setIsOrdered(bool new_value, int index)
	{
		m_constraints_is_ordered.setValue(new_value, index);
	}

	void decreaseCounter(int index)
	{
		--m_constraints_counter[index];
	}

	void addNeighbors(const std::vector<int>& new_neighbors, int index)
	{
		m_constraints_neighbors[index].insert(new_neighbors);
	}

	void deleteNeighbour(int neighbour_to_delete, int index)
	{
		m_constraints_neighbors[index].erase(neighbour_to_delete);
	}

	void clearNeighbors(int index)
	{
		m_constraints_neighbors[index].clear();
	}

	_NODISCARD const uint32_t* getEnabledConstraintVertices(int index) const
	{
		return m_constraints_buffer->getConstraintVertices(m_enabled_constraints[index]);
	}

	_NODISCARD uint32_t* getEnabledConstraintVertices(int index)
	{
		return m_constraints_buffer->getConstraintVertices(m_enabled_constraints[index]);
	}

	_NODISCARD const MathUtils::FastSet& getNeighbors(int index) const
	{
		return m_constraints_neighbors[index];
	}

	_NODISCARD int getNeighborsCount(int index) const
	{
		return (int)m_constraints_neighbors[index].size();
	}

	void clear()
	{
		m_enabled_constraints.clear();
		m_constraints_buffer = nullptr;
		for (int i = 0; i < m_constraints_neighbors.size(); ++i)
		{
			m_constraints_neighbors[i].clear();
		}
		m_constraints_color.clear();
		m_constraints_counter.clear();
		m_constraints_is_ordered.clear();
	}

private:
	std::vector<int> m_enabled_constraints;
	ConstraintsBuffers* m_constraints_buffer = nullptr;
	std::vector<MathUtils::FastSet> m_constraints_neighbors;
	std::vector<int16_t> m_constraints_color;
	std::vector<int> m_constraints_counter;
	BoolVector::BoolVector m_constraints_is_ordered;
};

class TasksMap
{
public:
	void reserve(int partitions_count)
	{
		clear();
		m_data.resize((size_t)partitions_count, {});
	}

	void pushTask(int task_id, int partition_id)
	{
		m_data[(size_t)partition_id].push_back(task_id);
	}

	_NODISCARD const std::vector<int>& getTasks(int partition_id) const
	{
		return m_data[(size_t)partition_id];
	}

	_NODISCARD int getPartitionsCount() const
	{
		return (int)m_data.size();
	}

	_NODISCARD int getTasksCount(int partition_id) const
	{
		return (int)m_data[(size_t)partition_id].size();
	}

	void clear()
	{
		m_data.clear();
	}

private:
	std::vector<std::vector<int>> m_data;
};

class ConstraintsGraph
{
public:
	ConstraintsGraph() = delete;

	ConstraintsGraph(const ConstraintsGraph&) = delete;

	ConstraintsGraph(ConstraintsGraph&&) = default;

	ConstraintsGraph& operator=(ConstraintsGraph&&) = default;

	ConstraintsGraph(const ConstraintsGraphSettings& settings) : m_settings(settings)
	{
	}

	void setConstraints(ConstraintsBuffers& constraints, int vertices_count);

	_NODISCARD int getPhantomVerticesCount() const
	{
		return m_phantom_vertices_count;
	}

	_NODISCARD int getNodesCount() const
	{
		return m_nodes.getNodesCount();
	}

	_NODISCARD const std::vector<PhantomVerticesDescriptor>& getReplacedVertices() const
	{
		return m_replaced_vertices;
	}

	_NODISCARD int getPhantomConstraintsCount() const
	{
		return (int)m_replaced_vertices.size();
	}

	_NODISCARD ConstraintsBuffers& getPhantomConstraints();

	_NODISCARD int getColorCount() const
	{
		return m_colors_count;
	}

	_NODISCARD const TasksMap& getTasksMap() const
	{
		return m_tasks_map;
	}

	void clear()
	{
		m_nodes.clear();
		m_original_vertices_count = 0;
		m_phantom_vertices_count = 0;
		m_replaced_vertices.clear();
		m_phantom_constraints.clear();
		m_tasks_map.clear();
		m_colors_count = 1;
	}

private:
	void clearEdges();

	void createEdges();

	_NODISCARD std::vector<MathUtils::FastSet> bronKerboschDegeneracy();

	void bronKerboschPivot(const MathUtils::FastSet& clique, MathUtils::FastSet candidates,
		MathUtils::FastSet duplicates, std::vector<MathUtils::FastSet>& results) const;

	_NODISCARD std::vector<int> sortDegeneracy();

	_NODISCARD int findCommonVertex(const MathUtils::FastSet& clique) const;

	void replaceVertices(const std::vector<MathUtils::FastSet>& cliques);

	void generatePhantomConstraintsBuffers();

	void assignColors(const std::vector<int>& conflicts, int max_degree, int thread_id);

	void detectConflicts(const std::vector<int>& conflicts, std::array<MathUtils::FastSet, THREADS_COUNT>& tmp_conflicts, int thread_id) const;

	void colorGraph();

	void createMap();

	void makeMapUniform(const std::array<std::vector<int>, CONSTRAINT_TYPES_COUNT>& independent_tasks);

private:
	ConstraintsGraphSettings m_settings;

	ConstraintGraphNodes m_nodes;

	int m_original_vertices_count = 0;
	int m_phantom_vertices_count = 0;

	std::vector<PhantomVerticesDescriptor> m_replaced_vertices;
	ConstraintsBuffers m_phantom_constraints;

	TasksMap m_tasks_map;

	int m_colors_count = 1;
};