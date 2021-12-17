#include "dsor_utils/spherical_coordinates.hpp"
#include <Eigen/Dense>
// Bring in gtest
#include <gtest/gtest.h>


// Declare a test
TEST(TestSuite, testCase5) {
    ASSERT_DOUBLE_EQ(2, 2);
}

// Declare another test 
// calling EXPECT_* and/or ASSERT_* macros as needed
TEST(TestSuite, testCase6) {
    
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}