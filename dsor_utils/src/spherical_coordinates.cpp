/**
 * Authors:
 *      Joao Quintas (jquintas@gmail.com)
 *      Joao Cruz (joao.pedro.cruz@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: spherical_coordinates.cpp 
 * Brief: Defines all functions related to spherical coordinates conversions
 */
#include "dsor_utils/math.hpp"
#include "dsor_utils/rotations.hpp"
#include "dsor_utils/frames.hpp"
#include "dsor_utils/control/vehicle_saturations.hpp"
#include "dsor_utils/spherical_coordinates.hpp"
// Note: math.hpp, rotations.hpp and frames.hpp are only included so that the header files are indexed by catkin (to be improved)

/**
 * @brief Convert from spherical to cartesian coordinates. Used mainly with usbl fixes
 * 
 * @param bearing Horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
 * @param elevation Angle measured between the horizontal and the vehicle line of sight to the object
 * @param range Distance to the object
 * @return Eigen Vector with cartesian coordinates 
 */
template <typename T>
Eigen::Matrix<T, 3, 1> DSOR::spherical_to_cartesian(T bearing, T elevation, T range) {

    Eigen::Matrix<T, 3, 1> pos_cartesian;
	
    pos_cartesian << sin(bearing) * range * cos(elevation),
                     cos(bearing) * range * cos(elevation),
                     range * sin(elevation);

    return pos_cartesian;
}