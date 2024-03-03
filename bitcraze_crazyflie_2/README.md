# Bitcraze Crazyflie2 Description (MJCF)

Tested with MuJoCo 3.1.2

## Overview

This package contains  simplified robot description (MJCF) of the Crazyflie2 model from the [Bitcraze](https://www.bitcraze.io/). It is derived from the publicly available, but archived, ROS description: [crazyflie_ros](https://github.com/whoenig/crazyflie_ros).
![Crazyflie2 Model](cf2.png)

## URDF → MJCF Conversion

1. Convereted the DAE mesh file in `crazyflie_description` to OBJ format using [Blender](https://www.blender.org/).
2. Processed the OBJ file with [obj2mjcf](https://github.com/kevinzakka/obj2mjcf)
3. Added a `<freejoint>` to the root body, and some lighting in the XML file.
4. Set the inertial properties to values obtained from the datasheet and [MIT's system identification](https://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf)
5. These properties are set via inertial tag i.e. `pos ="0 0 0" mass="0.027" diaginertia="2.3951e-5 2.3951e-5 3.2347e-5"`
6. Added combined thrust and body moments about a `site` placed at the inertial frame. The `ctrlrange` limits are currently arbitrary and need to be further tuned.
7. Added `scene.xml` which includes the quadrotor, with a textured ground plane and skybox.

## License
