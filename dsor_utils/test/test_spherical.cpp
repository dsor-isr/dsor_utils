/**
 * Authors:
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * Maintained by: 
 *      Andre Potes (andre.potes@tecnico.ulisboa.pt)
 *      Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)                
 * Last Update: 17/12/2021
 * License: MIT
 * File: test_spherical.cpp 
 * Brief: Tests all the functions declared in spherical_coordinates.hpp
 */
#include "dsor_utils/spherical_coordinates.hpp"
#include <gtest/gtest.h>

/**
 * @brief Test function: Eigen::Matrix<T, 3, 1> DSOR::spherical_to_cartesian(T bearing, T elevation, T range)
 */
TEST(TestSuite, testCase5) {
    // TODO
}

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}