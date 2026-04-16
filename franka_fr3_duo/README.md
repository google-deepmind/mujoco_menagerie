# Franka Robotics FR3 Duo Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.3 or later.

## Overview

This package contains a robot description (MJCF) of the Franka FR3 Duo: two [FR3v2](https://franka.de/research) arms mounted on a shared duo mount bracket. It is derived from the [franka_description](https://github.com/frankaemika/franka_description) URDF/xacro.

## URDF → MJCF derivation steps

1. Generated the URDF using `xacro` from `franka_description/robots/fr3_duo/fr3_duo.urdf.xacro` (without end-effectors).
2. Added `<mujoco><compiler discardvisual="true" fusestatic="false" balanceinertia="true"/></mujoco>` to the URDF.
3. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
4. Manually edited the MJCF to follow MuJoCo Menagerie style:
   - Extracted common properties into the `<default>` section.
   - Added visual meshes (OBJ) from the existing `franka_fr3_v2` menagerie model alongside collision meshes (STL).
   - Added mount collision meshes from `franka_description`.
   - Added position-controlled actuators for both arms (14 total, 7 per arm).
   - Added a `home` keyframe with both arms in the standard home configuration.
   - Added `scene.xml` with a textured groundplane, skybox, and lighting.

## Kinematic structure

- **fr3_duo_mount_mounting_point**: Duo mount bracket (fixed to base)
  - **fr3_duo_mount_origin** → **fr3_duo_mount_left**: Left arm mounting (tilted)
    - **left_fr3v2_joint1..7**: Left FR3v2 arm (7 DOF)
  - **fr3_duo_mount_right**: Right arm mounting (tilted)
    - **right_fr3v2_joint1..7**: Right FR3v2 arm (7 DOF)

**Total DOF**: 14 (7 per arm)

## License

This model is released under an [Apache-2.0 License](LICENSE).
