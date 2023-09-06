# Hello Robot Stretch 2 Description (MJCF)

Requires MuJoCo 2.3.3 or later.

## Overview

This package contains a simplified robot description (MJCF) of the [Hello Robot Stretch 2](https://hello-robot.com/product) developed by [Hello Robot](https://hello-robot.com/). The original URDF and assets were provided directly by Hello Robot under the [Clear BSD License](LICENSE).

<p float="left">
  <img src="stretch.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Converted the DAE [mesh
   files](https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_description/meshes/)
   to OBJ format using [Blender](https://www.blender.org/).
2. Processed `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Added `<mujoco> <compiler discardvisual="false" fusestatic="false" balanceinertia="true"/> </mujoco>` to the URDF's
   `<robot>` clause in order to preserve visual geometries.
4. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
5. Manually edited the MJCF to extract common properties into the `<default>` section.
6. Added actuators.
7. Added <exclude> clauses to prevent collisions in the telescoping arm.
8. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under a [Clear BSD License](LICENSE).

## Acknowledgement

This model would not be possible without the help and patience of [Binit Shah](https://binitshah.github.io/).
