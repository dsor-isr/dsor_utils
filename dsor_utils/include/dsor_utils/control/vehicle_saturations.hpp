/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 10/02/2022
 * License: MIT
 * File: vehicle_saturations.hpp 
 * Brief: Defines functions used for saturating the vehicle inputs to the system
 * 
 */
#pragma once

namespace DSOR {

/**
 * @brief Method to saturate the control in depth
 * @param depth_command The input command in depth
 * @param altitude_min The minimum altitude (positive distance from the vehicle to the bottom in m)
 * @param measured_depth The measured depth of the vehicle
 * @param measured_altitude The measured altitude of the vehicle
 * @return The saturated control of depth such that the vehicle does not surpass the minimum altitude 
*/
template <typename T>
inline T saturateControlDepthUsingMinAltitude(T depth_command, T altitude_min, T measured_depth, T measured_altitude) {
    return std::min(depth_command, measured_depth + measured_altitude - altitude_min);
}

/**
 * @brief Method to saturate the control in altitude
 * @param altitude_command The input command in altitude
 * @param altitude_min The minimum altitude to keep from the ground
 * @return The output command of the altitude saturated such that the vehicle does not surpass the minimum altitude
 */
template <typename T>
inline T saturateControlAltitudeUsingMinAltitude(T altitude_command, T altitude_min) {
    return std::max(altitude_command, altitude_min);
}

/**
 * @brief Method to saturate the control in depth
 * @param depth_command The input command in depth
 * @param max_depth The max depth
 * @return The output command of the depth saturated such that the vehicle does not surpass the maximum depth
 */
template <typename T>
inline T saturateControlDepthUsingMaxDepth(T depth_command, T max_depth) {
    return std::min(depth_command, max_depth);
}

/** 
 * @brief Method to saturate the control in altitude
 * @param altitude_command The input command in altitude
 * @param measured_depth The measured depth of the vehicle
 * @param measured_altitude The measured altitude of the vehicle
 * @param max_depth The maximum depth allowed
 * @return The output command of the altitude saturated such that the vehicle does not surpass the maximum depth
 */
template <typename T>
inline T saturateControlAltitudeUsingMaxDepth(T altitude_command, T measured_depth, T measured_altitude, T max_depth) {
    return std::max(altitude_command, measured_depth + measured_altitude - max_depth);
}

}