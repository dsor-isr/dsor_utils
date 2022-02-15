/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 *      Joao Quintas (jquintas@gmail.com)
 *      Joao Cruz (joao.pedro.cruz@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: math.hpp
 * Brief: Defines all functions related to general math functions that can be used anywhere
 */
#pragma once

#include <algorithm>

namespace DSOR {

/**
 * @brief Returns the sign of the number
 * 
 * @param v A number
 * @return 1 if value is positive
 *         0 if value is 0
 *        -1 if value is negative
 */
template <typename T>
inline int sign(T v) {
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

/**
 * @brief A function that saturates 2 values linearly
 * @param value A number to saturate
 * @param min The minimum value
 * @param max The maximum value
 * @return A value such that value in [min, max]
 */
template <typename T>
inline T saturation(T value, T min, T max) {
    return std::max(std::min(value, max), min);
}

/**
 *@brief A function to check if two numbers are equal (int, float, double, etc)
 * @param a a number to compare
 * @param b another number to compare
 * @param tolerance tolerance of comparison
 * @return A boolean for true if equal within tolerance, or otherwise
 */
template <typename T>
inline bool approximatelyEquals(T a, T b, T tolerance=1e-6) {
    if (b == 0.0) return a == 0.0; // preventing division by zero
    return std::abs(1.0 - a / b) < tolerance;
}
}