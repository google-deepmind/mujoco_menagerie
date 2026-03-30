# Franka Robotics Mobile FR3 Duo Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.3 or later.

## Overview

This package contains a robot description (MJCF) of the Franka Mobile FR3 Duo v0.2: a mobile manipulator consisting of a TMR v0.2 mobile base, a vertical spine, a head, and two [FR3v2](https://franka.de/research) arms on a duo mount. It is derived from the [franka_description](https://github.com/frankaemika/franka_description) URDF/xacro.

## URDF → MJCF derivation steps

1. Generated the URDF using `xacro` from `franka_description/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro` (without end-effectors).
2. Added minimal inertials to the virtual planar joint links for MuJoCo compatibility.
3. Added `<mujoco><compiler discardvisual="true" fusestatic="false" balanceinertia="true"/></mujoco>` to the URDF.
4. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
5. Manually edited the MJCF to follow MuJoCo Menagerie style:
   - Extracted common properties into the `<default>` section.
   - Added visual meshes (OBJ) from the existing `franka_fr3_v2` menagerie model alongside collision meshes (STL).
   - Added collision meshes for the mobile base, spine, head, and duo mount from `franka_description`.
   - Added position-controlled actuators for the base (3), spine (1), and both arms (14).
   - Added a `home` keyframe with the arms in the standard home configuration.
   - Added `scene.xml` with a textured groundplane, skybox, and lighting.

## Kinematic structure

- **base** → **virtual_x** → **virtual_y** → **base_link**: Planar mobile base (3 DOF: x, y, theta)
  - TMR v0.2 chassis with wheels (casters + argo drives)
  - **franka_spine** → **franka_spine_mounting_point**: Vertical spine (1 DOF prismatic)
    - **fr3_duo_mount_mounting_point**: Duo mount bracket
      - **head_link**: Head with camera mounting
      - **fr3_duo_mount_left**: Left arm mounting (tilted)
        - **left_fr3v2_joint1..7**: Left FR3v2 arm (7 DOF)
      - **fr3_duo_mount_right**: Right arm mounting (tilted)
        - **right_fr3v2_joint1..7**: Right FR3v2 arm (7 DOF)

**Total actuated DOF**: 18 (3 base + 1 spine + 7 left arm + 7 right arm)

## License

This model is released under an [Apache-2.0 License](LICENSE).
