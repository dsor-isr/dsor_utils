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
#include <Eigen/Dense>

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
 * @brief A function that saturates 2 values
 * @param value A number to saturate
 * @param min The minimum value
 * @param max The maximum value
 * @return A value such that value in [min, max]
 */
template <typename T>
inline T saturation(T value, T min, T max) {
    return std::max(std::min(value, max), min);
}

}