#pragma once
#include <vector>

// Utils to copy data

namespace CopyUtils
{
	template <class T>
	inline void setSubvector(const T* src, T* dst, int offset, int size)
	{
		std::memcpy(dst + offset, src, size * sizeof(T));
	}

	template <class T>
	inline _NODISCARD std::vector<T> getSubvector(const std::vector<T>& src, int start, int end)
	{
		return std::vector<T>(src.begin() + start, src.begin() + end);
	}
}