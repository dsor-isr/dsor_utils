/**
 * Authors:
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: 
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)                
 * Last Update: 17/12/2021
 * License: MIT
 * File: test_rotations.cpp 
 * Brief: Tests all the functions declared in rotations.hpp
 */
#include "dsor_utils/rotations.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>

#define APPROX_PRECISION 0.0001

// Call EXPECT_* and/or ASSERT_* macros as needed

/**
 * @brief Test function: Eigen::Matrix<T, 3, 1> quaternion_to_euler(const Eigen::Quaternion<T> &q)
 * Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention
 */
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

/**
 * @brief Test function: Eigen::Quaternion<T> euler_to_quaternion(const Eigen::Matrix<T, 3, 1> &v)
 * Function to convert a vector of euler angles according to Z-Y-X convention into a quaternion
 */
TEST(TestSuite, EulerToQuat) {
    // test conversion from euler angle (roll, pitch, yaw) to quaternion (x,y,z,w) in
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

/**
 * @brief Test function: T yaw_from_quaternion(const Eigen::Quaternion<T> &q) 
 * Gets the yaw angle from a quaternion (assumed a Z-Y-X rotation)
 */
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

/**
 * @brief Test unit:  wrapTo2pi(T angle)
 *  Wrap angle between [0, 2PI] 
 */
TEST(TestSuite, WrapToTwoPi) {
    // test wrapping an angle between 0 and 2 pi radians

    std::vector<double> angles_double{-M_PI-0.1, -M_PI, -2.8, 3.1, M_PI, M_PI+1};
    std::vector<float> angles_float{-M_PI-0.1, -M_PI, -2.8, 3.1, M_PI, M_PI+1};
    std::vector<float> angles_wrapped_int{0, 3, 4, 3, 4, 6};

    std::vector<double> angles_wrapped_double{3.04159, 3.14159, 3.48319, 3.1, 3.14159, 4.141592};
    std::vector<float> angles_wrapped_float{3.04159, 3.14159, 3.48319, 3.1, 3.14159, 4.141592};
    std::vector<int> angles_int{static_cast<int>(-2*M_PI), 
                                static_cast<int>(-M_PI-0.1), 
                                static_cast<int>(-2.8), 
                                static_cast<int>(3.1), 
                                static_cast<int>(M_PI+1),
                                static_cast<int>(2*M_PI)
                                };

    for (unsigned int i = 0; i < angles_double.size(); i++) {
        ASSERT_TRUE( std::abs(DSOR::wrapTo2pi<double>(angles_double[i]) - angles_wrapped_double[i]) < APPROX_PRECISION );
        ASSERT_TRUE( std::abs(DSOR::wrapTo2pi<float>(angles_float[i]) - angles_wrapped_float[i]) < APPROX_PRECISION );
        ASSERT_TRUE( std::abs(DSOR::wrapTo2pi<int>(angles_int[i]) - angles_wrapped_int[i]) < APPROX_PRECISION );
    }
}


/**
 * @brief Test function:  T wrapTopi(T angle)
 * that wraps angles between -pi and pi
 * The tests were developed for floats, doubles and int types
 */
TEST(TestSuite, WrapToPi) {

    // Test the function for type float, double and int
    std::vector<float> input_f{-2*M_PI, -M_PI-0.1, -2.8, 3.1, M_PI+1, 2*M_PI};
    std::vector<double> input_d{-2*M_PI, -M_PI-0.1, -2.8, 3.1, M_PI+1, 2*M_PI};
    std::vector<int> input_i{static_cast<int>(-2*M_PI), 
                             static_cast<int>(-M_PI-0.1), 
                             static_cast<int>(-2.8), 
                             static_cast<int>(3.1), 
                             static_cast<int>(M_PI+1),
                             static_cast<int>(2*M_PI)
                             };
    
    std::vector<float> expected_wrapped_f{0, 3.04159265358979, -2.8000000, 3.1000000, -2.14159265358979, 0};
    std::vector<double> expected_wrapped_d{0, 3.04159265358979, -2.8000000, 3.1000000, -2.14159265358979, 0};
    std::vector<int> expected_wrapped_i{0, -3, -2, 3, -2, 0};
    
    for (unsigned int i=0; i < input_d.size(); i++) {
        ASSERT_TRUE(std::abs(DSOR::wrapTopi<float>(input_f[i]) - expected_wrapped_f[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::wrapTopi<double>(input_d[i]) - expected_wrapped_d[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::wrapTopi<int>(input_i[i]) - expected_wrapped_i[i]) < APPROX_PRECISION);
    }
}

/**
 * @brief Test function:  T radToDeg(T angle)
 * that converts angles in radians to angles in degrees
 */
TEST(TestSuite, radToDeg) {
    
    // Input values to the function
    std::vector<int> input_i{0, 0, 0, 1, 4, 6, 6, 0, -6, 3, 0};
    std::vector<float> input_f{0.0, 0.174533, 0.349066, 1.78024, 4.53786, 6.28319, 6.98132, -0.174533, -6.98132, 0.0, 0.0};
    std::vector<double> input_d{0.0, 0.174533, 0.349066, 1.78024, 4.53786, 6.28319, 6.98132, -0.174533, -6.98132, 0.0, 0.0};

    // Output values to the function
    std::vector<int> expected_i{0, 0, 0, 57, 229, 343, 343, 0, -343};
    std::vector<double> expected_f{0, 10.00000429, 20.00000857, 102.0002385, 260.000226, 360.0002689, 400.0001714, -10.00000429, -400.0001714};
    std::vector<float> expected_d{0, 10.00000429, 20.00000857, 102.0002385, 260.000226, 360.0002689, 400.0001714, -10.00000429, -400.0001714};
    
    // Test the result for int, floats and doubles
    for (unsigned int i = 0; i < expected_i.size(); i++) {
        ASSERT_TRUE(std::abs(DSOR::radToDeg<int>(input_i[i]) - expected_i[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::radToDeg<float>(input_f[i]) - expected_f[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::radToDeg<double>(input_d[i]) - expected_d[i]) < APPROX_PRECISION);
    }
}

/**
 * @brief Test function:  T degToRad(T angle)
 * that converts angles in radians to angles in degrees
 */
TEST(TestSuite, degToRad) {
    
    // Inputs values to the function
    std::vector<int> input_i{0, 10, 20, 99, 102, 260, 360, 400, -10, -400};
    std::vector<float> input_f{0.0, 10.0, 20.0, 99.0, 102.0, 260.0, 360.0, 400.0, -10.0, -400.0};
    std::vector<double> input_d{0.0, 10.0, 20.0, 99.0, 102.0, 260.0, 360.0, 400.0, -10.0, -400.0};

    // Output values to the function
    std::vector<int> expected_i{0, 0, 0, 1, 1, 4, 6, 6, 0, -6};
    std::vector<float> expected_f{0.0, 0.174533, 0.349066, 1.72788, 1.78024, 4.53786, 6.28319, 6.98132, -0.174533, -6.98132};
    std::vector<double> expected_d{0.0, 0.174533, 0.349066, 1.72788, 1.78024, 4.53786, 6.28319, 6.98132, -0.174533, -6.98132};

    // Test the result for int, floats and doubles
    for (unsigned int i = 0; i < expected_i.size(); i++) {
        std::cout << std::abs(DSOR::degToRad<int>(input_i[i]) - expected_i[i]) << std::endl;
        ASSERT_TRUE(std::abs(DSOR::degToRad<int>(input_i[i]) - expected_i[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::degToRad<float>(input_f[i]) - expected_f[i]) < APPROX_PRECISION);
        ASSERT_TRUE(std::abs(DSOR::degToRad<double>(input_d[i]) - expected_d[i]) < APPROX_PRECISION);
    }
}

/**
 * @brief Test function:  T angleDiff(T a, T b) that
 * calculate the diference between angles 
 */
TEST(TestSuite, angleDiff) {

    std::vector<double> input_a_d{-2*M_PI, -1.0, 0, 2.4, M_PI, 2*M_PI};
    std::vector<float> input_a_f{-2*M_PI, -1.0, 0, 2.4, M_PI, 2*M_PI};
    std::vector<int> input_a_i{static_cast<int>(-2*M_PI), 
                             static_cast<int>(-1.0), 
                             static_cast<int>(0.0), 
                             static_cast<int>(2.4), 
                             static_cast<int>(M_PI),
                             static_cast<int>(2*M_PI)
                             };
    
    std::vector<double> input_b_d{2*M_PI, 2.0, 0.0, 0.0, -M_PI, 3.0};
    std::vector<float> input_b_f{2*M_PI, 2.0, 0.0, 0.0, -M_PI, 3.0};
    std::vector<int> input_b_i{static_cast<int>(2*M_PI),
                            static_cast<int>(2.0),
                            static_cast<int>(0.0),
                            static_cast<int>(0.0),
                            static_cast<int>(-M_PI),
                            static_cast<int>(3.0)
                            };

    std::vector<double> expect_d{0, -3, 0, 2.4, 0, -3};
    std::vector<float> expect_f{0, -3, 0, 2.4, 0, -3};
    std::vector<int> expect_i{0, -3, 0, 2, 0, 3};

    for (unsigned int i = 0; i < input_a_d.size(); i++) {
        ASSERT_TRUE( std::abs( DSOR::angleDiff<double>(input_a_d[i], input_b_d[i]) - expect_d[i] ) < APPROX_PRECISION );
        ASSERT_TRUE( std::abs( DSOR::angleDiff<float>(input_a_f[i], input_b_f[i]) - expect_f[i] ) < APPROX_PRECISION );
        ASSERT_TRUE( std::abs( DSOR::angleDiff<int>(input_a_i[i], input_b_i[i]) - expect_i[i] ) < APPROX_PRECISION );
    }
}

/**
 * @brief Test function:  Eigen::Matrix<T, 3, 3> computeSkewSymmetric(const Eigen::Matrix<T, 3, 1> &v)
 * Computes the 3D skew-symmetric matrix given a vector of values
 */
TEST(TestSuite, SkewSymmetric3) {
    Eigen::Vector3d input_d(0.2, 9.3, 1.5);
    Eigen::Vector3f input_f(0.2, 9.3, 1.5);
    Eigen::Vector3i input_i(20, 9, 1);

    Eigen::Matrix<double, 3, 3> expect_d;
    Eigen::Matrix<float, 3, 3> expect_f;
    Eigen::Matrix<int, 3, 3> expect_i;

    expect_d << 0.0, -1.5, 9.3,
                1.5, 0.0, -0.2,
                -9.3, 0.2, 0.0;

    expect_f << 0.0, -1.5, 9.3,
                1.5, 0.0, -0.2,
                -9.3, 0.2, 0.0;

    expect_i << 0, -1, 9,
                1, 0, -20,
                -9, 20, 0;

    ASSERT_TRUE( DSOR::computeSkewSymmetric3<double>(input_d).isApprox(expect_d, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::computeSkewSymmetric3<float>(input_f).isApprox(expect_f, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::computeSkewSymmetric3<int>(input_i).isApprox(expect_i, APPROX_PRECISION) );    

}   

/**
 * @brief Test function:  Eigen::Matrix<T, 2, 2> computeSkewSymmetric(T c)
 * Compute the 2x2 skew-symmetric matrix from a constant (int, float or double)
 */
TEST(TestSuite, SkewSymmetric2) {
    
    double input_d = 1.5;
    float input_f = 1.5;
    int input_i = 1;

    Eigen::Matrix<double, 2, 2> expect_d;
    Eigen::Matrix<float, 2, 2> expect_f;
    Eigen::Matrix<int, 2, 2> expect_i;

    expect_d << 0.0, -1.5,
                1.5, 0.0;
    expect_f << 0.0, -1.5,
                1.5, 0.0;
    expect_i << 0, -1,
                1, 0;

    ASSERT_TRUE( DSOR::computeSkewSymmetric2<double>(input_d).isApprox(expect_d, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::computeSkewSymmetric2<float>(input_f).isApprox(expect_f, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::computeSkewSymmetric2<int>(input_i).isApprox(expect_i, APPROX_PRECISION) );
}
/**
 * @brief Test function: inline Eigen::Matrix<T, 3, 3> rotationAngularBodyToInertial(const Eigen::Matrix<T, 3, 1> &v)
 * Compute the 3x3 rotation of angular body velocity to inertial angular velocity
 */
TEST(TestSuite, rotationAngularBodyToInertial) {
    
    Eigen::Vector3i angle_i;
    Eigen::Vector3f angle_f;
    Eigen::Vector3d angle_d;
    
    // Define the input angles
    angle_i << 0, 1, 1;
    angle_f << 0.349, 1.3, 1.3;
    angle_d << 0.349, 1.3, 1.3;

    // Expected matrices
    Eigen::Matrix3i m_i;
    Eigen::Matrix3f m_f;
    Eigen::Matrix3d m_d;
    
    m_i << 1, 0, 1,
        0, 1, 0,
        0, 0, 1;

    m_f << 1, 1.2317687, 3.3849502, 
        0, 0.9397151, -0.3419,
        0, 1.2783542, 3.5129692;

    m_d << 1, 1.2317687, 3.3849502, 
        0, 0.9397151, -0.3419583, 
        0, 1.2783542, 3.5129692;
    
    ASSERT_TRUE( DSOR::rotationAngularBodyToInertial<double>(angle_d).isApprox(m_d, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::rotationAngularBodyToInertial<float>(angle_f).isApprox(m_f, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::rotationAngularBodyToInertial<int>(angle_i).isApprox(m_i, APPROX_PRECISION) );   
}

/**
 * @brief Test function: Eigen::Matrix<T, 3, 3> rotationBodyToInertial(const Eigen::Matrix<T, 3, 1> &v) 
 * Converts a rotation matrix from body to inertial
 */
TEST(TestSuite, rotationBodyInertial) {
    Eigen::Vector3f input_f;
    Eigen::Vector3d input_d;
    
    // Define the input angles (rad)
    input_f << 1, 2, 3;
    input_d << 1, 2, 3;
    
    Eigen::Matrix<double, 3, 3> expected_d;
    Eigen::Matrix<float, 3, 3> expected_f;

    expected_d << 0.4119822, -0.8337377, -0.3676305,
                -0.0587266, -0.4269176,  0.9023816,
                -0.9092974, -0.3501755, -0.2248451;

    expected_f << 0.4119822, -0.8337377, -0.3676305,
                -0.0587266, -0.4269176,  0.9023816,
                -0.9092974, -0.3501755, -0.2248451;

    ASSERT_TRUE( DSOR::rotationBodyToInertial<double>(input_d).isApprox(expected_d, APPROX_PRECISION) );
    ASSERT_TRUE( DSOR::rotationBodyToInertial<float>(input_f).isApprox(expected_f, APPROX_PRECISION) );
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}