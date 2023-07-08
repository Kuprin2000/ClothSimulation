#pragma once
#include <array>
#include <vector>
#include "keys_utils.h"
#include "aligned_vector.h"

class ConstraintsBuffers
{
public:
	ConstraintsBuffers() = default;

	ConstraintsBuffers(const ConstraintsBuffers&) = delete;

	ConstraintsBuffers(ConstraintsBuffers&&) = default;

	ConstraintsBuffers& operator=(ConstraintsBuffers&&) = default;

	static _NODISCARD ConstraintsBuffers reserveBuffers(const std::array<int, CONSTRAINT_TYPES_COUNT>& counted_constraints)
	{
		uint32_t total_constraints_count = 0;
		uint32_t total_bytes_count = 0;

		for (int type = 0; type < CONSTRAINT_TYPES_COUNT; ++type)
		{
			total_constraints_count += counted_constraints[type];
			total_bytes_count += counted_constraints[type] * CONSTRAINT_BYTES_COUNT[type];
		}

		ConstraintsBuffers result;

		// create buffers
		result.m_stored_constraints_per_type.fill(0);
		result.m_constraints_types.reserve(total_constraints_count);
		result.m_is_disabled.reserve(total_constraints_count);
		result.m_bytes_offsets.reserve(total_constraints_count);
		result.m_data_blocks.reserve(total_bytes_count);

		return result;
	}

	static _NODISCARD ConstraintsBuffers reserveBuffers(const std::vector<ConstraintType>& constraints_types)
	{
		std::array<int, CONSTRAINT_TYPES_COUNT> constraints_counters = { 0 };

		for (const auto value : constraints_types)
		{
			++constraints_counters[(int)value];
		}

		return reserveBuffers(constraints_counters);
	}

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>& constraint_vertices,
		const std::vector<float>& constraint_data, bool is_disabled)
	{
		++m_constraints_count;
		++m_stored_constraints_per_type[(size_t)constraint_type];
		m_constraints_types.push_back((uint8_t)constraint_type);
		m_is_disabled.push_back((uint8_t)is_disabled);

		const uint32_t uint_data_size = CONSTRAINT_UINT_COUNT[(size_t)constraint_type] * sizeof(uint32_t);
		// made for phantom constraints
		const uint32_t real_uint_data_size = (uint32_t)constraint_vertices.size() * sizeof(uint32_t);
		const uint32_t float_data_size = (uint32_t)CONSTRAINT_FLOAT_COUNT[(size_t)constraint_type] * sizeof(float);
		const uint32_t reserved_size = (uint32_t)CONSTRAINT_BYTES_COUNT[(size_t)constraint_type] - uint_data_size - float_data_size;

		m_bytes_offsets.push_back(m_occupied_bytes_count);
		m_data_blocks.resize(m_data_blocks.size() + uint_data_size + float_data_size + reserved_size);
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_vertices.data(), real_uint_data_size);
		m_occupied_bytes_count += uint_data_size;
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_data.data(), float_data_size);
		m_occupied_bytes_count += float_data_size + reserved_size;
	}

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>& constraint_vertices,
		const std::vector<float>&& constraint_data, bool is_disabled)
	{
		++m_constraints_count;
		++m_stored_constraints_per_type[(size_t)constraint_type];
		m_constraints_types.push_back((uint8_t)constraint_type);
		m_is_disabled.push_back((uint8_t)is_disabled);

		const uint32_t uint_data_size = CONSTRAINT_UINT_COUNT[(size_t)constraint_type] * sizeof(uint32_t);
		// made for phantom constraints
		const uint32_t real_uint_data_size = (uint32_t)constraint_vertices.size() * sizeof(uint32_t);
		const uint32_t float_data_size = (uint32_t)CONSTRAINT_FLOAT_COUNT[(size_t)constraint_type] * sizeof(float);
		const uint32_t reserved_size = (uint32_t)CONSTRAINT_BYTES_COUNT[(size_t)constraint_type] - uint_data_size - float_data_size;

		m_bytes_offsets.push_back(m_occupied_bytes_count);
		m_data_blocks.resize(m_data_blocks.size() + uint_data_size + float_data_size + reserved_size);
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_vertices.data(), real_uint_data_size);
		m_occupied_bytes_count += uint_data_size;
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_data.data(), float_data_size);
		m_occupied_bytes_count += float_data_size + reserved_size;
	}

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>&& constraint_vertices,
		const std::vector<float>&& constraint_data, bool is_disabled)
	{
		++m_constraints_count;
		++m_stored_constraints_per_type[(size_t)constraint_type];
		m_constraints_types.push_back((uint8_t)constraint_type);
		m_is_disabled.push_back((uint8_t)is_disabled);

		const uint32_t uint_data_size = CONSTRAINT_UINT_COUNT[(size_t)constraint_type] * sizeof(uint32_t);
		// made for phantom constraints
		const uint32_t real_uint_data_size = (uint32_t)constraint_vertices.size() * sizeof(uint32_t);
		const uint32_t float_data_size = (uint32_t)CONSTRAINT_FLOAT_COUNT[(size_t)constraint_type] * sizeof(float);
		const uint32_t reserved_size = (uint32_t)CONSTRAINT_BYTES_COUNT[(size_t)constraint_type] - uint_data_size - float_data_size;

		m_bytes_offsets.push_back(m_occupied_bytes_count);
		m_data_blocks.resize(m_data_blocks.size() + uint_data_size + float_data_size + reserved_size);
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_vertices.data(), real_uint_data_size);
		m_occupied_bytes_count += uint_data_size;
		memcpy(m_data_blocks.data() + m_occupied_bytes_count, constraint_data.data(), float_data_size);
		m_occupied_bytes_count += float_data_size + reserved_size;
	}

	void pushBuffer(const ConstraintsBuffers& buffers_to_push, int value_to_add_to_vertices)
	{
		const int old_constraints_count = m_constraints_count;
		const uint32_t old_offset = m_occupied_bytes_count;

		m_constraints_count += buffers_to_push.m_constraints_count;
		for (int i = 0; i < CONSTRAINT_TYPES_COUNT; ++i)
		{
			m_stored_constraints_per_type[i] += buffers_to_push.m_stored_constraints_per_type[i];
		}

		m_constraints_types.insert(
			m_constraints_types.end(), buffers_to_push.m_constraints_types.begin(), buffers_to_push.m_constraints_types.end());

		m_is_disabled.insert(m_is_disabled.begin(), buffers_to_push.m_is_disabled.begin(), buffers_to_push.m_is_disabled.end());

		m_occupied_bytes_count += buffers_to_push.m_occupied_bytes_count;
		m_bytes_offsets.insert(m_bytes_offsets.end(), buffers_to_push.m_bytes_offsets.begin(), buffers_to_push.m_bytes_offsets.end());
		for (int i = old_constraints_count; i < m_constraints_count; ++i)
		{
			m_bytes_offsets[i] += old_offset;
		}

		m_data_blocks.insert(m_data_blocks.end(), buffers_to_push.m_data_blocks.begin(), buffers_to_push.m_data_blocks.end());
		for (int i = old_constraints_count; i < m_constraints_count; ++i)
		{
			const int vertices_count = CONSTRAINT_UINT_COUNT[m_constraints_types[i]];
			uint32_t* vertex = (uint32_t*)(m_data_blocks.data() + m_bytes_offsets[i]);
			for (int j = 0; j < vertices_count; ++j)
			{
				*vertex += value_to_add_to_vertices;
				++vertex;
			}
		}
	}

	_NODISCARD const std::array<int, CONSTRAINT_TYPES_COUNT>& getStoredConstraintsCounters() const
	{
		return m_stored_constraints_per_type;
	}

	void setStoredConstraintsCounters(const std::array<int, CONSTRAINT_TYPES_COUNT>& new_counters)
	{
		m_stored_constraints_per_type = new_counters;
	}

	_NODISCARD int getConstraintsCount() const
	{
		return m_constraints_count;
	}

	_NODISCARD ConstraintType getConstraintType(int constraint_id) const
	{
		return (ConstraintType)m_constraints_types[constraint_id];
	}

	void setConstraintType(ConstraintType new_type, int constraint_id)
	{
		m_constraints_types[constraint_id] = (uint8_t)new_type;
	}

	_NODISCARD const AlignedVector::AlignedVector<uint8_t>& getConstraintTypes() const
	{
		return m_constraints_types;
	}

	_NODISCARD bool getIsDisabled(int constraint_id) const
	{
		return m_is_disabled[constraint_id];
	}

	_NODISCARD const AlignedVector::AlignedVector<uint8_t>& getIsDisabled() const
	{
		return m_is_disabled;
	}

	void enableConstraint(int constraint_id)
	{
		m_is_disabled[constraint_id] = 0u;
	}

	void disableConstraint(int constraint_id)
	{
		m_is_disabled[constraint_id] = 1u;
	}

	_NODISCARD uint8_t* getDataBlock(int index)
	{
		return &m_data_blocks[m_bytes_offsets[index]];
	}

	_NODISCARD const uint8_t* getDataBlock(int index) const
	{
		return &m_data_blocks[m_bytes_offsets[index]];
	}

	_NODISCARD uint32_t* getConstraintVertices(int index)
	{
		return (uint32_t*)&m_data_blocks[m_bytes_offsets[index]];
	}

	_NODISCARD const uint32_t* getConstraintVertices(int index) const
	{
		return (uint32_t*)&m_data_blocks[m_bytes_offsets[index]];
	}


	void clear()
	{
		m_constraints_count = 0;
		m_stored_constraints_per_type = { 0 };
		m_constraints_types.clear();
		m_is_disabled.clear();
		m_occupied_bytes_count = 0u;
		m_bytes_offsets.clear();
		m_data_blocks.clear();
	}

private:
	int m_constraints_count = 0;
	std::array<int, CONSTRAINT_TYPES_COUNT> m_stored_constraints_per_type = { 0 };
	AlignedVector::AlignedVector<uint8_t> m_constraints_types;

	AlignedVector::AlignedVector<uint8_t> m_is_disabled;

	uint32_t m_occupied_bytes_count = 0u;
	AlignedVector::AlignedVector<uint32_t> m_bytes_offsets;
	AlignedVector::AlignedVector<uint8_t> m_data_blocks;
};