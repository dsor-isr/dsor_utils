#include "math.hpp"
#include "rotations.hpp"
#include "spherical_coordinates.hpp"
// Note: math.hpp and rotations.hpp are only included so that the header files are indexed by catkin (to be improved)

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