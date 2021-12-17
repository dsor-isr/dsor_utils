#include "dsor_utils/rotations.hpp"
#include <iostream>
#include <Eigen/Dense>
// Bring in gtest
#include <gtest/gtest.h>

#define APPROX_PRECISION 0.00001

// Call EXPECT_* and/or ASSERT_* macros as needed

/* ==================== Test unit for converting quaternion to euler =============================== */

TEST(TestSuite, QuatToEuler) {
    // test conversion from quaternion (x,y,z,w) to euler angles (roll, pitch, yaw) in
    // the ZYX convention

    Eigen::Quaternion<double> q_double;
    q_double.x() = 0.0364663;
    q_double.y() = 0.7259515;
    q_double.z() = -0.5696747;
    q_double.w() = 0.3835823;
     
    Eigen::Quaternion<float> q_float;
    q_float.x() = 0.0364663;
    q_float.y() = 0.7259515;
    q_float.z() = -0.5696747;
    q_float.w() = 0.3835823;

    Eigen::Quaternion<int> q_int;
    q_int.x() = 0;
    q_int.y() = 0;
    q_int.z() = 0;
    q_int.w() = 1;

    Eigen::Matrix<double, 3, 1> angles_double(-1.64159, 0.641593, -2.64159);
    Eigen::Matrix<float, 3, 1> angles_float(-1.64159, 0.641593, -2.64159);
    Eigen::Matrix<int, 3, 1> angles_int(0, 0, 0);
    
    // assert true due to approximation errors in double and float conversions
    ASSERT_TRUE( DSOR::quaternion_to_euler<double>(q_double).isApprox(angles_double, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::quaternion_to_euler<float>(q_float).isApprox(angles_float, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::quaternion_to_euler<int>(q_int).isApprox(angles_int, APPROX_PRECISION) );
}

/* ============================================================================================ */

/* ==================== Test unit for converting euler to quaternion =============================== */

TEST(TestSuite, EulerToQuat) {
    // test conversion from quaternion (x,y,z,w) to euler angles (roll, pitch, yaw) in
    // the ZYX convention

    Eigen::Quaternion<double> q_double;
    q_double.x() = 0.79157;
    q_double.y() = 0.0763388;
    q_double.z() = 0.605307;
    q_double.w() = -0.034531;
     
    Eigen::Quaternion<float> q_float;
    q_float.x() = 0.397474;
    q_float.y() = 0.453882;
    q_float.z() = 0.783772;
    q_float.w() = -0.147335;

    Eigen::Matrix<double, 3, 1> angles_double(3, -1.3, 0.3);
    Eigen::Matrix<float, 3, 1> angles_float(-2.0, 4, -0.2);
        
    // assert true due to approximation errors in double and float conversions
    ASSERT_TRUE( DSOR::euler_to_quaternion<double>(angles_double).isApprox(q_double, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::euler_to_quaternion<float>(angles_float).isApprox(q_float, APPROX_PRECISION) );
}

/* ============================================================================================ */

/* ==================== Test unit for obtain yaw angle from a quaternion =============================== */

TEST(TestSuite, YawFromQuat) {
    // test obtaining a yaw angle form a quaternion

    Eigen::Quaternion<double> q_double;
    q_double.x() = 0.79157;
    q_double.y() = 0.0763388;
    q_double.z() = 0.605307;
    q_double.w() = -0.034531;
     
    Eigen::Quaternion<float> q_float;
    q_float.x() = 0.397474;
    q_float.y() = 0.453882;
    q_float.z() = 0.783772;
    q_float.w() = -0.147335;

    double yaw_angle_double = DSOR::quaternion_to_euler<double>(q_double).z();
    float yaw_angle_float = DSOR::quaternion_to_euler<float>(q_float).z();

    // assert true due to approximation errors in double and float conversions
    ASSERT_TRUE( std::abs(DSOR::yaw_from_quaternion<double>(q_double) - yaw_angle_double) < APPROX_PRECISION );
    ASSERT_TRUE( std::abs(DSOR::yaw_from_quaternion<float>(q_float) - yaw_angle_float) < APPROX_PRECISION );
}

/* ============================================================================================ */

/* ================= Test unit for wrap an angle betwen 0 and 2 PI radians  =================== */

TEST(TestSuite, WrapToTwoPi) {
    // test wrapping an angle between 0 and 2 pi radians

    std::vector<double> angles{-2*M_PI, -M_PI-0.1, -M_PI, -2.8, 3.1, M_PI, M_PI+1, 2*M_PI};
    std::vector<double> angles_wrapped{0, 3.041592, 3.141592, 3.48318, 3.1, 3.141592, 4.141592, 6.283185}

/* ============================================================================================ */


int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}