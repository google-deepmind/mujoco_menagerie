## UMI-Gripper Description (MJCF)

> Requires MuJoCo 2.2.2 or later.

### Overview

This package contains a simplified robot description (MJCF) of the [Universal Manipulation Interface (UMI) Gripper](http://umi-gripper.github.io/). It is derived from the publicly available [CAD model](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU/edit?tab=t.0).

[URDF description](https://github.com/pal-robotics/talos_robot/tree/kinetic-devel/talos_description).

<p float="left">
  <img src="umi_gripper.png" width="600px">
</p>


###  -> URDF derivation step

1. 

### URDF -> MJCF derivation step

1. Added `<mujoco> <compiler balanceinertia="true" discardvisual="false"/> </mujoco>` to the URDF's `<robot>`clause in order to preserve visual geometries.
2. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3. Updated the `<compiler>` tag in the MJCF to specify `meshdir="assets/"` and `texturedir="assets/"` for correct asset loading.
4. Added materials and textures for the mirrors and ArUco markers (`right_aruco_sticker.png`, `left_aruco_sticker.png`).
5. Created a `<default>` section to define common properties for joints, actuators, and geoms.
6. Added two mirror geoms (`left_mirror`, `right_mirror`) with reflective materials for enhanced visual effects.
7. Defined left and right finger holders with appropriate inertial properties and joints (`finger_left_joint`, `finger_right_joint`).
8. Introduced a `<tendon>` section with a fixed tendon named split to synchronize the movement of both fingers.
9. Added position-controlled actuators for the fingers and gripper joints.
10. Tuned contact parameters by setting `impratio="10"` for better non-slip interaction.
11. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

### License

This model is released under an [Apache-2.0 License](LICENSE).
