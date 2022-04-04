/**
 * Authors:
 *      Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Maintained by: Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Last Update: 08/02/2018
 * Github: https://github.com/jimmyberg/DigitalFilters
 * License: GNU
 * Brief: Abstract class for moving filters. Moving filter are real time filter used for applications where
 *        continuous filtering is necessary as it can be part of an control
 *        system.
 */

#pragma once

#include <stdexcept>
#include <cmath>
#include <dsor_utils/data_structures/circular_buffer.hpp>

namespace tps {
	template<typename T>
	constexpr T pow(T input, unsigned int power){ return (power == 0 ? 1 : input * (power <= 1 ? 1 : tps::pow(input, power-1))); }
}

/**
 * @brief      Abstract base class for digital moving filters.
 * 
 * @tparam     Type  Floating point type used.
 */
template<typename Type>
class DigitalFilter {

    public:

        virtual Type update(Type newValue) = 0;
        virtual Type getOutput() = 0;
};