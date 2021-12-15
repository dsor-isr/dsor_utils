/**
 * Authors:
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: Marcelo Fialho Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Last Update: 14/12/2021
 * License: MIT
 * File: frames.hpp 
 * Brief: Defines all functions related to conversions between ENU do NED frames and vice-versa
 * 
 * NOTE: Most of this code is adapted from mavros
 * https://github.com/mavlink/mavros/blob/master/mavros/src/lib/ftf_frame_conversions.cpp
 * which had as authors Nuno Marques (n.marques21@hotmail.com) and Eddy Scott (scott.edward@aurora.aero)
 */
#pragma once

#include <Eigen/Dense>
#include "rotations.hpp"

namespace DSOR{

/**
* @brief Static quaternion needed for rotating between ENU and NED frames
* NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
* ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
*/
static const Eigen::Quaterniond NED_ENU_Q = euler_to_quaternion(Eigen::Vector3d(M_PI, 0.0, M_PI_2));

/**
 * @brief Static quaternion needed for rotating between aircraft and base_link frames
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
 * Fto Forward, Left, Up (base_link) frames.
 */
static const Eigen::Quaterniond VEHICLE_BASELINK_Q = euler_to_quaternion(Eigen::Vector3d(M_PI, 0.0, 0.0));

template <typename T>
Eigen::Quaternion<T> transform_orientation(const Eigen::Quaternion<T> &q, const StaticTF transform)
{
	// Transform the attitude representation from frame to frame.
	// The proof for this transform can be seen
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		return NED_ENU_Q * q;

	case StaticTF::VEHICLE_TO_BASELINK:
	case StaticTF::BASELINK_TO_VEHICLE:
		return q * VEHICLE_BASELINK_Q;

	case StaticTF::ABSOLUTE_FRAME_VEHICLE_TO_BASELINK:
	case StaticTF::ABSOLUTE_FRAME_BASELINK_TO_VEHICLE:
		return VEHICLE_BASELINK_Q * q;

	default:
		ROS_FATAL("unsupported StaticTF mode");
		return q;
	}
}

/**
 * @brief Transform from attitude represented WRT NED frame to attitude
 *		  represented WRT ENU frame
 */
template<class T>
inline T transform_orientation_ned_enu(const T &in) {
	return detail::transform_orientation(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform from attitude represented WRT ENU frame to
 *		  attitude represented WRT NED frame
 */
template<class T>
inline T transform_orientation_enu_ned(const T &in) {
	return detail::transform_orientation(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame
 */
template<class T>
inline T transform_orientation_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame
 */
template<class T>
inline T transform_orientation_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame, treating aircraft frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame, treating baselink frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in NED to ENU frame.
 */
template<class T>
inline T transform_frame_ned_enu(const T &in) {
	return detail::transform_static_frame(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU to NED frame.
 *
 */
template<class T>
inline T transform_frame_enu_ned(const T &in) {
	return detail::transform_static_frame(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform data expressed in Aircraft frame to Baselink frame.
 *
 */
template<class T>
inline T transform_frame_aircraft_baselink(const T &in) {
	return detail::transform_static_frame(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform data expressed in Baselink frame to Aircraft frame.
 *
 */
template<class T>
inline T transform_frame_baselink_aircraft(const T &in) {
	return detail::transform_static_frame(in, StaticTF::BASELINK_TO_AIRCRAFT);

/**
 * Method that converts a quaternion expressed in a NED frame to a quaternion expressed in ENU
 * NOTE: This implementation is based on this explanation
 * https://stackoverflow.com/questions/18818102/convert-quaternion-representing-rotation-from-one-coordinate-system-to-another.
 * This is valid for conversions made for quaternions expressed in the inertial frame. Be carefull if your
 * quaternion is expressed in the body frame, because the orientation NED and ENU in body frame can be a bit tricky.
 * @param q_ENU An eigen quaternion expressed in ENU
 * @return An eigen quaternion expressed in NED
 */
template<typename T>
Eigen::Quaternion<T> quaternion_ENU_to_NED(const Eigen::Quaternion<T> &q_ENU) {


    Eigen::Quaternion<T> q_NED;
    //TODO

    return q_NED;
}

/**
 * Method that converts a quaternion expressed in a ENU frame to a quaternion expressed in NED
 * NOTE: This implementation is based on this explanation
 * https://stackoverflow.com/questions/18818102/convert-quaternion-representing-rotation-from-one-coordinate-system-to-another.
 * This is valid for conversions made for quaternions expressed in the inertial frame. Be carefull if your
 * quaternion is expressed in the body frame, because the orientation NED and ENU in body frame can be a bit tricky.
 * @param q_NED An eigen quaternion expressed in NED
 * @return An eigen quaternion expressed in ENU
 */
template<typename T>
Eigen::Quaternion<T> quaternion_NED_to_ENU(const Eigen::Quaternion<T> &q_NED) {

    Eigen::Quaternion<T> q_ENU;
    //TODO

    return q_ENU;
}

}