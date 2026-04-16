# Franka Robotics Mobile FR3 Duo Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.3 or later.

## Overview

This package contains a robot description (MJCF) of the Franka Mobile FR3 Duo v0.2: a mobile manipulator consisting of a TMR v0.2 mobile base, a vertical spine, a head, and two [FR3v2](https://franka.de/research) arms on a duo mount. It is derived from the [franka_description](https://github.com/frankaemika/franka_description) URDF/xacro.

## URDF → MJCF derivation steps

1. Generated the URDF using `xacro` from `franka_description/robots/mobile_fr3_duo_v0_2/mobile_fr3_duo_v0_2.urdf.xacro` (without end-effectors).
2. Added `<mujoco><compiler discardvisual="true" fusestatic="false" balanceinertia="true"/></mujoco>` to the URDF.
3. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
4. Manually edited the MJCF to follow MuJoCo Menagerie style:
   - The two argo drive modules use position-controlled steering and velocity-controlled driving actuators.
   - Caster wheels and rocker arm are passive (unactuated).
   - Added wheel friction for ground contact (argo wheels: 1.5, casters: 0.5).
   - Extracted common properties into the `<default>` section.
   - Added visual meshes (OBJ) from the existing `franka_fr3_v2` menagerie model alongside collision meshes (STL).
   - Added collision meshes for the mobile base, spine, head, and duo mount from `franka_description`.
   - Added position-controlled actuators for the spine (1) and both arms (14).
   - Added a `home` keyframe with the arms in the standard home configuration.
   - Added `scene.xml` with a textured groundplane, skybox, and lighting.

## Kinematic structure

- **base_link** (freejoint — 6 DOF): TMR v0.2 mobile chassis
  - **caster_front_left** (passive): Front-left caster wheel (steering + rolling)
  - **argo_drive_front** (actuated): Front-right swerve module
    - **tmrv0_2_joint_0**: Steering (position-controlled)
    - **tmrv0_2_joint_1**: Driving (velocity-controlled)
  - **rocker_arm_link** (passive): Rear rocker arm (roll joint, ±0.18 rad)
    - **caster_rear_right** (passive): Rear-right caster wheel
    - **argo_drive_rear** (actuated): Rear-left swerve module
      - **tmrv0_2_joint_2**: Steering (position-controlled)
      - **tmrv0_2_joint_3**: Driving (velocity-controlled)
  - **franka_spine** → **franka_spine_mounting_point**: Vertical spine (1 DOF prismatic)
    - **fr3_duo_mount_mounting_point**: Duo mount bracket
      - **head_link**: Head with camera mounting
      - **fr3_duo_mount_left**: Left arm mounting (tilted)
        - **left_fr3v2_joint1..7**: Left FR3v2 arm (7 DOF)
      - **fr3_duo_mount_right**: Right arm mounting (tilted)
        - **right_fr3v2_joint1..7**: Right FR3v2 arm (7 DOF)

**Total actuated DOF**: 19 (4 swerve drive + 1 spine + 7 left arm + 7 right arm)

## License

This model is released under an [Apache-2.0 License](LICENSE).
