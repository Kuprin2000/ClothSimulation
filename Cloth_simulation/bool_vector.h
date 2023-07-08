#pragma once
#include "aligned_vector.h"
#include <vector>

namespace BoolVector
{
	class BoolVector
	{
	public:
		void reserve(size_t size)
		{
			m_data.reserve(size / 8u + 1u);
		}

		void resize(size_t size)
		{
			m_data.resize(size / 8u + 1u);
			m_stored_values = size;
		}

		void push_back(bool value)
		{
			const size_t index = m_stored_values % 8;
			if (index && value)
			{
				m_data[m_stored_values / 8].m_uint |= (value << index);
			}
			else if (!(index))
			{
				BoolValues new_elem;
				new_elem.m_values.m_0 = value;
				m_data.push_back(new_elem);
			}

			++m_stored_values;
		}

		void clear()
		{
			m_data.clear();
			m_stored_values = 0u;
		}

		_NODISCARD bool operator[](size_t index) const
		{
			return (m_data[index / 8].m_uint >> (index % 8)) & 0x1;
		}

		void setValue(bool value, int index)
		{
			uint8_t new_value = UINT8_MAX;
			new_value -= 1 << (index % 8);
			new_value &= m_data[index / 8].m_uint;
			new_value += uint8_t(value) << (index % 8);
			m_data[index / 8].m_uint = new_value;
		}

		void fill(bool value)
		{
			BoolValues internal_value;
			internal_value.m_uint = value ? UINT8_MAX : 0u; 
			std::fill(m_data.begin(), m_data.end(), internal_value);
		}

		_NODISCARD size_t size() const
		{
			return m_stored_values;
		}

	private:
		union BoolValues
		{
			struct
			{
				uint8_t m_0 : 1;
				uint8_t m_1 : 1;
				uint8_t m_2 : 1;
				uint8_t m_3 : 1;
				uint8_t m_4 : 1;
				uint8_t m_5 : 1;
				uint8_t m_6 : 1;
				uint8_t m_7 : 1;
			} m_values;

			uint8_t m_uint = 0u;
		};

		AlignedVector::AlignedVector<BoolValues> m_data;
		size_t m_stored_values = 0u;
	};
}