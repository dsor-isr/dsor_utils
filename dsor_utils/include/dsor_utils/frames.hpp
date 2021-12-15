/**
 * Authors:
 * 		André Potes (andre.potes@gmail.com)
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
 * @brief Static quaternion to convert a rotation expressed in ENU to a rotation expressed in NED (Z->Y->X convention) on
 * 			the inertial frame
 * Rotate PI/2 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis
 * 
 * NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on
 * 			the inertial frame
 */
template <typename T>
static const Eigen::Quaternion<T> ENU_NED_INERTIAL_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, M_PI_2));

/**
 * @brief Static quaternion to convert a rotation expressed in ENU body frame (ROS base_link) to
 * 			 a rotation expressed in NED body frame  (Z->Y->X convention)
 * Rotate 0 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis
 * 
 * NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on
 * 			the body frame
*/
template <typename T>
static const Eigen::Quaternion<T> ENU_NED_BODY_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));

/**
 * @brief Static quaternion needed for rotating vectors in body frames between ENU and NED
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * Fto Forward, Left, Up (body frame in ENU).
 */
template <typename T>
static const Eigen::Quaternion<T> BODY_ENU_NED_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));

/**
 * @brief Static affine matrix to roate vectors ENU (or NED) -> NED (or ENU) expressed in body frame
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * Fto Forward, Left, Up (body frame in ENU).
 */
template <typename T>
static const Eigen::Transform<T, 3, Eigen::Affine> BODY_ENU_NED_TF = Eigen::Transform<T, 3, Eigen::Affine>(BODY_ENU_NED_Q);

/**
 * @brief Use reflections instead of rotations for NED <-> ENU transformation
 * to avoid NaN/Inf floating point pollution across different axes
 * since in NED <-> ENU the axes are perfectly aligned.
 */
static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1, 0, 2));
template <typename T>
static const Eigen::DiagonalMatrix<T, 3> NED_ENU_REFLECTION_Z(1, 1, -1);


/**
 * @brief Transform a rotation (as a quaternion) from body expressed in ENU (or NED) to inertial frame 
 * 			to a similar rotation (as quaternion) from body expressed in NED (or ENU) to inertial frame.
 *
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference. 
 * 
 * @param q quaternion representing a rotation: body frame ENU (or NED) -> inertial frame (in arbitrary convention)
 * @return quaternion represeting a rotation: body frame NED (or ENU) -> inertial frame (in arbitrary convention)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_body_rotation(const Eigen::Quaternion<T> &q) {
	return q * ENU_NED_BODY_Q;
}

/**
 * @brief Transform a rotation (as a quaternion) from body to inertial frame expressed in ENU (or NED)
 * 			to a similar rotation (as quaternion) from body to inertial frame expressed in NED (or ENU)
 * 
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference.
 * 
 * @param q quaternion representing a rotation: body frame (in arbitrary convention) -> inertial frame ENU (or NED)
 * @return quaternion represeting a rotation: body frame (in arbitrary convention) -> inertial frame NED (or ENU)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_inertial_rotation(const Eigen::Quaternion<T> &q) {
	return ENU_NED_INERTIAL_Q * q;
}


/**
 * @brief Transform a rotation (as a quaternion) from body (ENU or NED) to inertial frame (ENU or NED)
 * 			to a similar rotation (as quaternion) from body (NED or ENU) to inertial frame (NED or ENU)
 * 
 * NOTE: Check http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/
 * 	for more details behind these type of transformations towards obtaining rotations in different
 * 	frames of reference.
 * 
 * @param q quaternion representing a rotation: body frame (ENU or NED) -> inertial frame (ENU or NED)
 * @return quaternion representing a rotation: body frame (NED or ENU) -> inertial frame (NED or ENU)
 */
template <typename T>
inline Eigen::Quaternion<T> rot_body_to_inertial(const Eigen::Quaternion<T> &q) {
	return rot_inertial_rotation( rot_body_rotation( q ) );
}


/**
 * @brief Transform vector in ENU (or NED) to NED (or ENU), expressed in body-frame.
 * 	+PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED)
 * 	Fto Forward, Left, Up (body frame in ENU).
 * 
 * @param vec Vector expressed in body-frame (ENU or NED)
 * @return Vector expressed in body-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> transform_vect_body_enu_ned(const Eigen::Matrix<T,3,1> &vec) {
	return BODY_ENU_NED_TF * vec;
}

/**
 * @brief Transform vector in ENU (or NED) to NED (or ENU), expressed in inertial-frame.
 *  ENU <---> NED - Invert the Z axis and switch the XY axis
 * 
 * @param vec Vector expressed in inertial-frame (ENU or NED)
 * @return Vector expressed in inertial-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 1> transform_vect_inertial_enu_ned(const Eigen::Matrix<T,3,1> &vec) {
	return NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * vec);
}


/**
 * @brief Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in body-frame.
 * 	
 * NOTE: Check https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance for a detailed
 * 	explanation of the actual conversion proof for covariance matrices
 * 
 * @param cov_in Covariance matrix expressed in body-frame (ENU or NED)
 * @return Covariance matrix expressed in body-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> transform_cov3_body_enu_ned(const Eigen::Matrix<T, 3, 3> &cov_in) {
	return cov_in * BODY_ENU_NED_Q;
}



/**
 * @brief Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in inertial-frame.
 * 
 * NOTE: Check https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance for a detailed
 * 	explanation of the actual conversion proof for covariance matrices
 * 
 * @param cov_in Covariance matrix expressed in inertial-frame (ENU or NED)
 * @return Covariance matrix expressed in inertial-frame (NED or ENU)
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> transform_cov3_inertial_enu_ned(const Eigen::Matrix<T, 3, 3> &cov_in) {
	Eigen::Matrix<T, 3, 3> cov_out;

	cov_out = NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * cov_in * NED_ENU_REFLECTION_Z.transpose() ) *
        NED_ENU_REFLECTION_XY.transpose();
    
	return cov_out;
}


}