# xArm7 Description (MJCF)

Requires MuJoCo 2.3.3 or later.

## Changelog

- **17/12/2024**: Improved object grasping by:
  - Adding two collision box meshes as pads for each finger.
  - Setting `armature=0.1` for the joints.

  These changes resolve [issue #83](https://github.com/google-deepmind/mujoco_menagerie/issues/83) and are implemented in [PR #131](https://github.com/google-deepmind/mujoco_menagerie/pull/131).

## Overview

This package contains a simplified robot description (MJCF) of the [xArm7
arm](https://www.ufactory.cc/product-page/ufactory-xarm-7/) developed by
[UFACTORY](https://www.ufactory.cc/). It is derived from the [publicly available
URDF
description](https://github.com/xArm-Developer/xarm_ros/tree/master/xarm_description/urdf/xarm7).

<p float="left">
  <img src="xarm7.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Added `<mujoco> <compiler discardvisual="false" fusestatic="false"/> </mujoco>` to the URDF's
   `<robot>` clause in order to preserve visual geometries.
2. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3. Manually edited the MJCF to extract common properties into the `<default>` section.
4. Added actuators for the arm and hand.
5. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
