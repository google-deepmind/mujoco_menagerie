# Unitree Z1 description (MJCF)

## Overview

This package contains robot description (MJCF) of the [Z1 manipulator](https://www.unitree.com/z1/). It is derivede from the [publicly available URDF description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/z1_description/xacro/z1.urdf)

<p float="left">
  <img src="z1.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Added `<mujoco><compiler balanceinertia="true" discardvisual="false"/></mujoco>` to the URDF's `<robot>` clause in order to preserve visual geometries.
2. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3. Manually edited the MJCF to use meshes for collision too.
4. Added `scene.xml` which includes the robot, with a textured groundplane, skybox and haze.e.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
