#pragma once

#include <Eigen/Dense>

namespace DSOR{

/**
 * @brief     State class - used to save the state of a vehicle (using SNAME convention)
 * @author    Eduardo Cunha
 * @author    Andre Potes
 * @author    Marcelo Jacinto
 * @version   1.0.0
 * @date      2022/05/26
 * @copyright MIT
 */
struct VehicleState{

public:
    /**
     * The position of the vehicle expressed in the inertial frame
     * eta1=[x,y,z]^T
     */
    Eigen::Vector3d eta1{0.0, 0.0, 0.0};

    /**
     * The orientation of the vehicle expressed in the inertial frame, using euler angles
     * eta2=[roll, pitch, yaw]^T
     */
    Eigen::Vector3d eta2{0.0, 0.0, 0.0};

    /**
     * The body velocity of the vehicle v1=[u,v,w]^T
     */
    Eigen::Vector3d v1{0.0, 0.0, 0.0};

    /**
     * The body angular velocity of the vehicle v2=[p,q,r]^T
     */
    Eigen::Vector3d v2{0.0, 0.0, 0.0};
};
}