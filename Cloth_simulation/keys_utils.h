#pragma once
#include <map>
#include <array>
#include <algorithm>
#include "cloth_constraints.h"

namespace KeysUtils
{
	using ConstraintKey = std::array<uint32_t, 3>;

	_NODISCARD inline ConstraintKey twoVerticesToKey(uint32_t a, uint32_t b, ConstraintType constraint_type)
	{
		return { std::min<uint32_t>(a, b), std::max<uint32_t>(a, b), (uint32_t)constraint_type };
	}

	_NODISCARD inline std::vector<ConstraintKey> generateConstraintKeys(const std::vector<uint32_t>& vertices, ConstraintType constraint_type)
	{
		switch (constraint_key_generation_function[int(constraint_type)])
		{
		case ConstraintKeyGenerationFunction::TWO_VERTICES:
			return {
				twoVerticesToKey(vertices[0], vertices[1], constraint_type) };
			break;
		case ConstraintKeyGenerationFunction::THREE_VERTICES:
			return {
				twoVerticesToKey(vertices[0], vertices[1], constraint_type),
				twoVerticesToKey(vertices[0], vertices[2], constraint_type),
				twoVerticesToKey(vertices[1], vertices[2], constraint_type) };
			break;
		case ConstraintKeyGenerationFunction::ONE_VERTEX:
			return {
				{vertices[0], UINT32_MAX, (uint32_t)constraint_type} };
			break;
		default:
			throw std::exception("Can't generate the key!");
		}
	}

	inline void insertKeysToMap(const std::vector<ConstraintKey>& keys, int constraint_id, std::map<ConstraintKey, std::vector<int>>& search_map)
	{
		for (const auto& key : keys)
		{
			if (!search_map.contains(key))
			{
				search_map[key] = { constraint_id };
			}
			else
			{
				search_map[key].push_back(constraint_id);
			}
		}
	}
}
