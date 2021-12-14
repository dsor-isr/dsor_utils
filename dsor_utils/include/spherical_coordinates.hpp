/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 *      Joao Quintas (jquintas@gmail.com)
 *      Joao Cruz (joao.pedro.cruz@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: spherical_coordinates.hpp
 * Brief: Defines all functions related to general math functions that can be used anywhere
 */
#pragma once
#include <Eigen/Dense>

namespace DSOR {
    
    /**
	 * @brief Convert from spherical to cartesian coordinates. Used mainly with usbl fixes
	 * 
	 * @param bearing Horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
	 * @param elevation Angle measured between the horizontal and the vehicle line of sight to the object
	 * @param range Distance to the object
	 * @return Eigen Vector with cartesian coordinates 
	 */
    template <typename T>
	Eigen::Matrix<T, 3, 1> spherical_to_cartesian(T bearing, T elevation, T range);
}