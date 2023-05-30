#pragma once
#include <array>
#include <vector>
#include "keys_utils.h"

class ConstraintsBuffers
{
public:
	ConstraintsBuffers() = default;

	ConstraintsBuffers(const ConstraintsBuffers&) = delete;

	ConstraintsBuffers(ConstraintsBuffers&&) = default;

	ConstraintsBuffers& operator=(ConstraintsBuffers&&) = default;

	static _NODISCARD ConstraintsBuffers reserveBuffers(const std::array<int, CONSTRAINT_TYPES_COUNT>& counted_constraints);

	static _NODISCARD ConstraintsBuffers reserveBuffers(const std::vector<ConstraintType>& constraints_types);

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>& constraint_vertices,
		const std::vector<float>& constraint_data, bool is_disabled)
	{
		m_constraints_types.push_back((uint8_t)constraint_type);

		const size_t constraint_vertices_count = (constraint_type == ConstraintType::PHANTOM_VERTICES) ?
			CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES] : (int)constraint_vertices.size();
		m_vertices_for_constraints_offsets.push_back((uint32_t)m_vertices_for_constraints.size());
		m_vertices_for_constraints.insert(m_vertices_for_constraints.end(), constraint_vertices.begin(), constraint_vertices.end());
		m_vertices_for_constraints.resize(m_vertices_for_constraints.size() + constraint_vertices_count, 0u);

		m_data_for_constraints_offsets.push_back((uint32_t)m_data_for_constraints.size());
		m_data_for_constraints.insert(m_data_for_constraints.end(), constraint_data.begin(), constraint_data.end());

		m_is_disabled.push_back(is_disabled);

		++m_stored_constraints_per_type[(size_t)constraint_type];
		++m_constraints_count;
	}

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>& constraint_vertices,
		const std::vector<float>&& constraint_data, bool is_disabled)
	{
		m_constraints_types.push_back((uint8_t)constraint_type);

		const size_t constraint_vertices_count = (constraint_type == ConstraintType::PHANTOM_VERTICES) ?
			CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES] : (int)constraint_vertices.size();
		m_vertices_for_constraints_offsets.push_back((uint32_t)m_vertices_for_constraints.size());
		m_vertices_for_constraints.insert(m_vertices_for_constraints.end(), constraint_vertices.begin(), constraint_vertices.end());
		m_vertices_for_constraints.resize(m_vertices_for_constraints.size() + constraint_vertices_count, 0u);

		m_data_for_constraints_offsets.push_back((uint32_t)m_data_for_constraints.size());
		m_data_for_constraints.insert(m_data_for_constraints.end(), constraint_data.begin(), constraint_data.end());

		m_is_disabled.push_back(is_disabled);

		++m_stored_constraints_per_type[(size_t)constraint_type];
		++m_constraints_count;
	}

	void pushConstraint(ConstraintType constraint_type, const std::vector<uint32_t>&& constraint_vertices,
		const std::vector<float>&& constraint_data, bool is_disabled)
	{
		m_constraints_types.push_back((uint8_t)constraint_type);

		const size_t constraint_vertices_count = (constraint_type == ConstraintType::PHANTOM_VERTICES) ?
			CONSTRAINT_VERTICES_COUNT[(size_t)ConstraintType::PHANTOM_VERTICES] : (int)constraint_vertices.size();
		m_vertices_for_constraints_offsets.push_back((uint32_t)m_vertices_for_constraints.size());
		m_vertices_for_constraints.insert(m_vertices_for_constraints.end(), constraint_vertices.begin(), constraint_vertices.end());
		m_vertices_for_constraints.resize(m_vertices_for_constraints.size() + constraint_vertices_count, 0u);

		m_data_for_constraints_offsets.push_back((uint32_t)m_data_for_constraints.size());
		m_data_for_constraints.insert(m_data_for_constraints.end(), constraint_data.begin(), constraint_data.end());

		m_is_disabled.push_back(is_disabled);

		++m_stored_constraints_per_type[(size_t)constraint_type];
		++m_constraints_count;
	}

	void pushBuffer(const ConstraintsBuffers& buffers_to_push, int value_to_add_to_vertices);

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

	void setConstraintType(int constraint_id, ConstraintType new_type)
	{
		m_constraints_types[constraint_id] = (uint8_t)new_type;
	}

	_NODISCARD uint32_t* getConstraintVertices(int constraint_id)
	{
		return m_vertices_for_constraints.data() + m_vertices_for_constraints_offsets[constraint_id];
	}

	_NODISCARD const uint32_t* getConstraintVertices(int constraint_id) const
	{
		return m_vertices_for_constraints.data() + m_vertices_for_constraints_offsets[constraint_id];
	}

	void setConstraintVertices(const std::vector<uint32_t>& new_vertices, int constraint_id)
	{
		std::memcpy(m_vertices_for_constraints.data() + m_vertices_for_constraints_offsets[constraint_id],
			new_vertices.data(), new_vertices.size() * sizeof(uint32_t));
	}

	_NODISCARD float* getConstraintData(int constraint_id)
	{
		return m_data_for_constraints.data() + m_data_for_constraints_offsets[constraint_id];
	}

	_NODISCARD const float* getConstraintData(int constraint_id) const
	{
		return m_data_for_constraints.data() + m_data_for_constraints_offsets[constraint_id];
	}

	_NODISCARD bool getIsDisabled(int constraint_id) const
	{
		return m_is_disabled[constraint_id];
	}

	void enableConstraint(int constraint_id)
	{
		m_is_disabled[constraint_id] = false;
	}

	void disableConstraint(int constraint_id)
	{
		m_is_disabled[constraint_id] = true;
	}

	void clear()
	{
		m_stored_constraints_per_type.fill(0);
		m_constraints_count = 0;
		m_constraints_types.clear();
		m_vertices_for_constraints_offsets.clear();
		m_vertices_for_constraints.clear();
		m_data_for_constraints_offsets.clear();
		m_data_for_constraints.clear();
		m_is_disabled.clear();
	}

	_NODISCARD const std::vector<uint8_t>& getConstraintTypes() const
	{
		return m_constraints_types;
	}

	_NODISCARD const std::vector<uint32_t>& getVerticesOffsets() const
	{
		return m_vertices_for_constraints_offsets;
	}

	_NODISCARD std::vector<uint32_t>& getVertices()
	{
		return m_vertices_for_constraints;
	}

	_NODISCARD const std::vector<uint32_t>& getDataOffsets() const
	{
		return m_data_for_constraints_offsets;
	}

	_NODISCARD const std::vector<float>& getData() const
	{
		return m_data_for_constraints;
	}

	_NODISCARD const std::vector<bool>& getIsDisabled() const
	{
		return m_is_disabled;
	}

private:
	std::array<int, CONSTRAINT_TYPES_COUNT> m_stored_constraints_per_type = { 0 };

	int m_constraints_count = 0;

	std::vector<uint8_t> m_constraints_types;

	std::vector<uint32_t> m_vertices_for_constraints_offsets;
	std::vector<uint32_t> m_vertices_for_constraints;

	std::vector<uint32_t> m_data_for_constraints_offsets;
	std::vector<float> m_data_for_constraints;

	std::vector<bool> m_is_disabled;
};