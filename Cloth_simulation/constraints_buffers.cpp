// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "constraints_buffers.h"
#include <algorithm>

ConstraintsBuffers ConstraintsBuffers::reserveBuffers(const std::array<int, CONSTRAINT_TYPES_COUNT>& counted_constraints)
{
	// count size of buffers we have to create
	int total_constraints_count = 0;
	int total_vertices_count = 0;
	int total_data_count = 0;
	for (int type = 0; type < CONSTRAINT_TYPES_COUNT; ++type)
	{
		total_constraints_count += counted_constraints[type];
		total_vertices_count += counted_constraints[type] * CONSTRAINT_VERTICES_COUNT[type];
		total_data_count += counted_constraints[type] * CONSTRAINT_DATA_COUNT[type];
	}

	ConstraintsBuffers result;

	// create buffers
	result.m_stored_constraints_per_type.fill(0);
	result.m_constraints_types.reserve(total_constraints_count);
	result.m_vertices_for_constraints_offsets.reserve(total_constraints_count);
	result.m_vertices_for_constraints.reserve(total_vertices_count);
	result.m_data_for_constraints_offsets.reserve(total_constraints_count);
	result.m_data_for_constraints.reserve(total_data_count);
	result.m_is_disabled.reserve(total_constraints_count);

	return result;
}

ConstraintsBuffers ConstraintsBuffers::reserveBuffers(const std::vector<ConstraintType>& constraints_types)
{
	std::array<int, CONSTRAINT_TYPES_COUNT> constraints_counters = { 0 };

	for (const auto value : constraints_types)
	{
		++constraints_counters[(int)value];
	}

	// count size of buffers we have to create
	int total_constraints_count = 0;
	int total_vertices_count = 0;
	int total_data_count = 0;
	for (int type = 0; type < CONSTRAINT_TYPES_COUNT; ++type)
	{
		total_constraints_count += constraints_counters[type];
		total_vertices_count += constraints_counters[type] * CONSTRAINT_VERTICES_COUNT[type];
		total_data_count += constraints_counters[type] * CONSTRAINT_DATA_COUNT[type];
	}

	return reserveBuffers(constraints_counters);
}

void ConstraintsBuffers::pushBuffer(const ConstraintsBuffers& buffers_to_push, int value_to_add_to_vertices)
{
	m_constraints_types.insert(m_constraints_types.end(), buffers_to_push.m_constraints_types.begin(),
		buffers_to_push.m_constraints_types.end());

	m_vertices_for_constraints_offsets.insert(m_vertices_for_constraints_offsets.end(),
		buffers_to_push.m_vertices_for_constraints_offsets.begin(), buffers_to_push.m_vertices_for_constraints_offsets.end());
	std::for_each(m_vertices_for_constraints_offsets.begin() + m_constraints_count, m_vertices_for_constraints_offsets.end(),
		[this](uint32_t& value)
		{
			value += (uint32_t)m_vertices_for_constraints.size();
		});

	const int old_vertices_count = (int)m_vertices_for_constraints.size();
	m_vertices_for_constraints.insert(m_vertices_for_constraints.end(),
		buffers_to_push.m_vertices_for_constraints.begin(), buffers_to_push.m_vertices_for_constraints.end());
	std::for_each(m_vertices_for_constraints.begin() + old_vertices_count, m_vertices_for_constraints.end(),
		[value_to_add_to_vertices](uint32_t& value)
		{
			value += value_to_add_to_vertices;
		});

	m_data_for_constraints_offsets.insert(m_data_for_constraints_offsets.end(), buffers_to_push.m_data_for_constraints_offsets.begin(),
		buffers_to_push.m_data_for_constraints_offsets.end());
	std::for_each(m_data_for_constraints_offsets.begin() + m_constraints_count, m_data_for_constraints_offsets.end(),
		[this](uint32_t& value)
		{
			value += (uint32_t)m_data_for_constraints.size();
		});

	m_data_for_constraints.insert(m_data_for_constraints.end(), buffers_to_push.m_data_for_constraints.begin(),
		buffers_to_push.m_data_for_constraints.end());

	m_is_disabled.insert(m_is_disabled.end(), buffers_to_push.m_is_disabled.begin(), buffers_to_push.m_is_disabled.end());

	for (int i = 0; i < CONSTRAINT_TYPES_COUNT; ++i)
	{
		m_stored_constraints_per_type[i] += buffers_to_push.m_stored_constraints_per_type[i];
	}

	m_constraints_count += buffers_to_push.m_constraints_count;
}