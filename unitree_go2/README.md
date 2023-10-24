# Unitree Go2 Description (MJCF)

Requires MuJoCo 2.2.2 or later.

## Overview

This package contains a simplified robot description (MJCF) of the [Go2
Quadruped Robot](https://www.unitree.com/products/go2/) developed by [Unitree
Robotics](https://www.unitree.com/). It is derived from the [publicly available
URDF
description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description).

<p float="left">
  <img src="go2.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Converted the DAE [mesh
   files](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/a1/meshes)
to OBJ format using [Blender](https://www.blender.org/).
  - When exporting, ensure "up axis" is `+Z`, and "forward axis" is `+Y`.
2. Processed `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Added `<mujoco> <compiler discardvisual="false" strippath="false" fusestatic="false"/> </mujoco>` to the URDF's
   `<robot>` clause in order to preserve visual geometries.
4. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
5. Added a `<freejoint/>` to the base.
6. Manually edited the MJCF to extract common properties into the `<default>` section.
7. Softened the contacts of the feet to approximate the effect of rubber and
   increased `impratio` to reduce slippage.
8. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
