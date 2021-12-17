#include "dsor_utils/math.hpp"
#include <Eigen/Dense>
// Bring in gtest
#include <gtest/gtest.h>

// Call EXPECT_* and/or ASSERT_* macros as needed

/* ========================== Test unit for sign function =============================== */


TEST(TestSuite, SignPositive) {

    // test positive numbers 
    ASSERT_EQ( DSOR::sign<int>(10) , 1);
    ASSERT_EQ( DSOR::sign<double>( (double)20.0 ) , 1 );
    ASSERT_EQ( DSOR::sign<float>( (float)15.0 ) , 1 );
}

TEST(TestSuite, SignNegative) {
    // test negative numbers 
    ASSERT_EQ( DSOR::sign<int>(-10) , -1);
    ASSERT_EQ( DSOR::sign<double>( (double)-20.0 ) , -1 );
    ASSERT_EQ( DSOR::sign<float>( (float)-15.0 ) , -1 );
}

TEST(TestSuite, SignZero) {

    // test zero 
    ASSERT_EQ( DSOR::sign<int>(0) , 0);
    ASSERT_EQ( DSOR::sign<double>( (double)0.0 ) , 0 );
    ASSERT_EQ( DSOR::sign<float>( (float)0.0 ) , 0 );
}

/* ============================================================================================ */

/* ========================== Test unit for saturation function =============================== */

TEST(TestSuite, SaturatedPositive) {
    
    // test saturated positive value
    ASSERT_EQ( DSOR::saturation<int>(15, -12, 10) , 10);
    ASSERT_EQ( DSOR::saturation<double>( (double)15.0, (double)-12.0, (double)10.0) , (double)10.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)15.0, (float)-12.0, (float)10.0) , (float)10.0);

}   

TEST(TestSuite, SaturatedNegative) {
    // test saturated negative value
    ASSERT_EQ( DSOR::saturation<int>(-15, -12, 10) , -12);
    ASSERT_EQ( DSOR::saturation<double>( (double)-15.0, (double)-12.0, (double)10.0) , (double)-12.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)-15.0, (float)-12.0, (float)10.0) , (float)-12.0); 
}

TEST(TestSuite, NotSatured) {
    // test a value that is not saturated
    ASSERT_EQ( DSOR::saturation<int>(5, -10, 10) , 5);
    ASSERT_EQ( DSOR::saturation<double>( (double)5.0, (double)-12.0, (double)10.0) , (double)5.0);
    ASSERT_EQ( DSOR::saturation<float>( (float)5.0, (float)-12.0, (float)10.0) , (float)5.0);
}

/* ============================================================================================ */



int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}