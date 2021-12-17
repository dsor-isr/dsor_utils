/**
 * Authors:
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: 
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)                
 * Last Update: 17/12/2021
 * License: MIT
 * File: test_math.cpp 
 * Brief: Tests all the functions declared in math.hpp
 */
#include "dsor_utils/math.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>

/* ========================== Test unit for sign function =============================== */

/**
 * @brief Test function: inline int sign(T v)
 * only for positive values
 */
TEST(TestSuite, SignPositive) {

    // test positive numbers 
    ASSERT_EQ( DSOR::sign<int>(10) , 1);
    ASSERT_EQ( DSOR::sign<double>( (double)20.0 ) , 1 );
    ASSERT_EQ( DSOR::sign<float>( (float)15.0 ) , 1 );
}

/**
 * @brief Test function: inline int sign(T v)
 * only for zero
 */
TEST(TestSuite, SignNegative) {
    // test negative numbers 
    ASSERT_EQ( DSOR::sign<int>(-10) , -1);
    ASSERT_EQ( DSOR::sign<double>( (double)-20.0 ) , -1 );
    ASSERT_EQ( DSOR::sign<float>( (float)-15.0 ) , -1 );
}

/**
 * @brief Test function: inline int sign(T v)
 * only for positive values
 */
TEST(TestSuite, SignZero) {

    // test zero 
    ASSERT_EQ( DSOR::sign<int>(0) , 0);
    ASSERT_EQ( DSOR::sign<double>( (double)0.0 ) , 0 );
    ASSERT_EQ( DSOR::sign<float>( (float)0.0 ) , 0 );
}

/* ============================================================================================ */

/* ========================== Test unit for saturation function =============================== */

/**
 * @brief Test function: inline T saturation(T value, T min, T max)
 * only for positive saturated values
 */
TEST(TestSuite, SaturatedPositive) {
    
    // test saturated positive value
    ASSERT_EQ( DSOR::saturation<int>(15, -12, 10) , 10);
    ASSERT_EQ( DSOR::saturation<double>( (double)15.0, (double)-12.0, (double)10.0) , (double)10.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)15.0, (float)-12.0, (float)10.0) , (float)10.0);

}   

/**
 * @brief Test function: inline T saturation(T value, T min, T max)
 * only for negative saturated values
 */
TEST(TestSuite, SaturatedNegative) {
    // test saturated negative value
    ASSERT_EQ( DSOR::saturation<int>(-15, -12, 10) , -12);
    ASSERT_EQ( DSOR::saturation<double>( (double)-15.0, (double)-12.0, (double)10.0) , (double)-12.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)-15.0, (float)-12.0, (float)10.0) , (float)-12.0); 
}

/**
 * @brief Test function: inline T saturation(T value, T min, T max)
 * only for not saturated values
 */
TEST(TestSuite, NotSatured) {
    // test a value that is not saturated
    ASSERT_EQ( DSOR::saturation<int>(5, -10, 10) , 5);
    ASSERT_EQ( DSOR::saturation<double>( (double)5.0, (double)-12.0, (double)10.0) , (double)5.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)5.0, (float)-12.0, (float)10.0) , (float)5.0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}