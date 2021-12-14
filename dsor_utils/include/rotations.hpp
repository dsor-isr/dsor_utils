/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 *      Joao Quintas (jquintas@gmail.com)
 *      Joao Cruz (joao.pedro.cruz@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: rotations.hpp 
 * Brief: Defines all functions related to angle wrapping, rotation matrices, 
 * euler angles, convertion to quaternions, etc.
 */
#pragma once

#include <Eigen/Core>
#include <cmath>

namespace DSOR {

/**
 * @brief Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention
 * This function is from: https://github.com/mavlink/mavros/issues/444
 * and the logic is also available at: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
 * 
 * NOTE: The Eigen standard way of doing it is not used because for the order YPR the output range would be:
 * [Eigen EulerAngles implementation] yaw, pitch, roll in the ranges [0:pi]x[-pi:pi]x[-pi:pi]
 * @param q An eigen quaternion
 * @return A Vector<T, 3> with the [roll, pitch, yaw] obtained according to Z-Y-X convention
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> quaternion_to_euler(const Eigen::Quaternion<T> &q) {

    Eigen::Matrix<T, 3, 1> rpy;

    /* Compute roll */
    rpy.x() = std::atan2(2. * (q.w() * q.x() + q.y() * q.z()), 1. - 2. * (q.x() * q.x() + q.y()*q.y()));
    double sin_pitch = 2. * (q.w()*q.y() - q.z()*q.x());
    sin_pitch = sin_pitch >  1.0 ?  1.0 : sin_pitch;
    sin_pitch = sin_pitch < -1.0 ? -1.0 : sin_pitch;

    /* Compute pitch */
    rpy.y() = std::asin(sin_pitch);

    /* Compute yaw */
    rpy.z() = std::atan2(2. * (q.w()*q.z() + q.x()*q.y()), 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
    return rpy;
}

/**
 * @brief Converts a vector of [roll, pitch, yaw] according to Z-Y-X convention
 * into a quaternion
 * @param v An eigen vector of either floats or doubles
 * @return An Eigen Quaternion
 */
template <typename T>
inline Eigen::Quaternion<T> euler_to_quaternion(const Eigen::Matrix<T, 3, 1> &v) {
    
    // Create the Eigen quaternion
    Eigen::Quaternion<T> orientation;

    // Obtain the orientation according to Z-Y-X convention
    orientation = Eigen::AngleAxis<T>(v(2), Eigen::Matrix<T, 3, 1>::UnitZ()) *
            Eigen::AngleAxis<T>(v(1), Eigen::Matrix<T, 3, 1>::UnitY()) *
            Eigen::AngleAxis<T>(v(0), Eigen::Matrix<T, 3, 1>::UnitX());

    return orientation;
}

/**
 * @brief Wrap angle between [0, 2PI] 
 * 
 * @param angle angle in radians
 * @return The wraped angle
 */
template <typename T>
inline T wrapTo2pi(T angle) {

    T wrapped_angle = std::fmod(angle, 2 * M_PI);

    if(wrapped_angle < 0) 
        wrapped_angle += 2 * M_PI;
    return wrapped_angle;
}

/**
 * @brief Wrap angle between [-PI, PI] 
 * 
 * @param angle angle in radians
 * @return The wraped angle
 */
template <typename T>
inline T wrapTopi(T angle) {

    T wrapped_angle = std::fmod(angle + M_PI, 2 * M_PI);

    if (wrapped_angle < 0)
        wrapped_angle += 2 * M_PI;

    return wrapped_angle - M_PI;
}

/**
 * @brief Method to calculate the diference between angles correctly even if they wrap between -pi and pi
 * 
 * @param a angle 1 in radians 
 * @param b angle 2 in radians
 * @return The minimum difference between the two angles 
 */
template <typename T>
inline T angleDiff(T a, T b) {
    double aux = std::fmod(a - b + M_PI, 2 * M_PI);
    if (aux < 0) aux += (2 * M_PI);
    aux = aux - M_PI;
    return aux;
}


/**
 * @brief Compute the skew-symmetric matrix from a vector 3x1
 * @param v A vector with 3 elements
 * @return A 3x3 skew-symmetric matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> computeSkewSymmetric(const Eigen::Matrix<T, 3, 1> &v) {

    Eigen::Matrix<T, 3, 3> skew_symmetric;
    skew_symmetric <<  0.0, -v(2),  v(1),
                      v(2),   0.0, -v(0),
                     -v(1),  v(0),   0.0;

    return skew_symmetric;
}

/**
 * @brief Compute the rotation matrix that converts angular velocities expressed in the body frame
 * to angular velocities expressed in the inertial frame (according to Z-Y-X convention)
 * @param v A vector with 3 elements (roll, pitch, yaw)
 * @return A 3x3 rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> rotationAngularBodyToInertial(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> transformation_matrix;
    transformation_matrix << 1, sin(v(0)) * tan(v(1)), cos(v(0)) * tan(v(1)),
                             0, cos(v(0)), -sin(v(0)),
                             0, sin(v(0)) / cos(v(1)), cos(v(0)) / cos(v(1));
    return transformation_matrix;
}

/**
 * @brief Method that returns a rotation matrix from body frame to inertial frame, assuming a Z-Y-X convention
 * @param v A vector with euler angles (roll, pith, yaw) according to Z-Y-X convention
 * @return A 3x3 rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> rotationBodyToInertial(const Eigen::Matrix<T, 3, 1> &v) {

    Eigen::Matrix<T, 3, 3> rot_matrix;

    rot_matrix << cos(v(2)) * cos(v(1)), (-sin(v(2)) * cos(v(0))) + (cos(v(2)) * sin(v(1)) * sin(v(0))), (sin(v(2)) * sin(v(0))) + (cos(v(2)) * cos(v(0)) * sin(v(1))),
                  sin(v(2)) * cos(v(1)), (cos(v(2)) * cos(v(0))) + (sin(v(0)) * sin(v(1)) * sin(v(2))), (-cos(v(2)) * sin(v(0))) + (sin(v(1)) * sin(v(2)) * cos(v(0))),
                 -sin(v(1)), cos(v(1)) * sin(v(0)), cos(v(1)) * cos(v(0));
    return rot_matrix;
}

}