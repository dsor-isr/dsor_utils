#include "dsor_utils/math.hpp"
#include <Eigen/Dense>
// Bring in gtest
#include <gtest/gtest.h>


// // calling EXPECT_* and/or ASSERT_* macros as needed
TEST(TestSuite, Sign) {

    // test positive numbers 
    ASSERT_EQ( DSOR::sign<int>(10) , 1);
    ASSERT_EQ( DSOR::sign<double>( (double)20.0 ) , 1 );
    ASSERT_EQ( DSOR::sign<float>( (float)15.0 ) , 1 );
    
    // test zero 
    ASSERT_EQ( DSOR::sign<int>(0) , 0);
    ASSERT_EQ( DSOR::sign<double>( (double)0.0 ) , 0 );
    ASSERT_EQ( DSOR::sign<float>( (float)0.0 ) , 0 );

    // test negative numbers 
    ASSERT_EQ( DSOR::sign<int>(-10) , -1);
    ASSERT_EQ( DSOR::sign<double>( (double)-20.0 ) , -1 );
    ASSERT_EQ( DSOR::sign<float>( (float)-15.0 ) , -1 );

}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}