## Math

TODO - create documentation for this API

## Rotations

TODO - create documentation for this API

## Frames
This header file implements a set of templates to convert between ENU frame and NED frames, both for angles expressed in the inertial frame and in the body frame of a vehicle.

### Conventions on rotations:

- **Inertial Frame**

    To convert a rotation expressed in ENU to a rotation expressed in NED, we must apply the following rotation (using Z-Y-X convention):
        
        - Rotate 90º about Z-axis
        - Rotate 0º about Y-axis
        - Rotate -180º about X-axis

    To convert a rotation expressed in NED to a rotation expressed in ENU, we must apply the following rotation (using Z-Y-X convention):

        - Rotate -90º about Z-axis
        - Rotate 180º about Y-axis
        - Rotate 0º about X-axis

- **Body Frame** (a.k.a ROS base_link)

    To convert a rotation expressed in the ENU body frame (ROS base_link) to a rotation expressed in NED body frame (the typical body frame adopted for aerial and marine crafts), we must apply the following rotation (using Z-Y-X convention):

        - Rotate 0º about Z-axis
        - Rotate 0º about Y-axis
        - Rotate 180º about X-axis

    To convert a rotation expressed in the NED body frame (the typical body frame adopted for aerial and marine crafts) to a rotation expressed in the ENU body frame (ROS base_link), we must apply the following rotation (using Z-Y-X convention):

        - Rotate 0º about Z-axis
        - Rotate 0º about Y-axis
        - Rotate -180º about X-axis

## Spherical Coordinates

TODO - create documentation for this API