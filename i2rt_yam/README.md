# Yet Another Manipulator (YAM) Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.2.0 or later (`yam_linear_4310.xml`), or 3.1.3 (`yam.xml`).

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains simplified robot descriptions (MJCF) of the [YAM
robot](https://i2rt.com/products/yam-manipulator) developed by [I2RT
Robotics](https://i2rt.com/):

- [`yam.xml`](yam.xml) ([`scene.xml`](scene.xml)): YAM with the `linear_3507`
  gripper, derived from I2RT's [publicly available URDF
  description](https://github.com/i2rt-robotics/i2rt/blob/main/robot_models/yam/yam.urdf).
- [`yam_linear_4310.xml`](yam_linear_4310.xml)
  ([`scene_linear_4310.xml`](scene_linear_4310.xml)): YAM / YAM Pro with the
  `linear_4310` gripper and wrist-mounted Intel RealSense D405 camera on I2RT's
  camera bracket, derived from I2RT's SDK repository
  ([i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) at tag `v1.3.6`,
  commit `120c3c81400171174604e503943f8d1ebc891058`).

<p float="left">
  <img src="yam.png" width="400">
  <img src="yam_linear_4310.png" width="400">
</p>

## MJCF derivation steps (`yam.xml`)

1. Started from `yam.urdf` (commit SHA d4efb66d81bd8bde42909880b16591d4af82e8c0).
2. Added the following to the URDF `<robot>` tag `<mujoco><compiler balanceinertia="true" discardvisual="false" fusestatic="false" strippath="false"/></mujoco>`.
3. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
4. Added home keyframe.
5. Added tracking light.
6. Add frictionloss (not identified).
7. Added armature based on reflected inertia values provided by the manufacturer.
8. Switched to implicitfast and used position actuators with kp/kv semantics.

## `linear_4310` gripper and wrist camera variant (`yam_linear_4310.xml`)

[`yam_linear_4310.xml`](yam_linear_4310.xml) shares the arm links, meshes
(`model2.stl`..`model2__11.stl`), and primitive capsule collision geoms of
`yam.xml`, and replaces the gripper on `link_6` using I2RT's `v1.3.6` models
(`i2rt/robot_models/arm/yam_pro/v1/yam_pro.xml`,
`i2rt/robot_models/gripper/linear_4310/linear_4310.xml`, and
`i2rt/robot_models/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml`):

1. Copied unmodified STL meshes from `i2rt` tag `v1.3.6` into
   `assets/linear_4310/`:
   - `gripper.stl`, `tip_left.stl`, `tip_right.stl` from
     `i2rt/robot_models/gripper/linear_4310/assets/`
   - `d405_wrist_linear_4310.stl` from `i2rt/robot_models/station/assets/`
2. Updated the arm joint ranges and position actuator PD gains (`kp`
   80/80/80/40/10/10, `kv` 5/5/5/1.5/1.5/1.5) to match I2RT's `yam_pro` +
   `linear_4310` SDK configuration.
3. Replaced the gripper on `link_6` with the `linear_4310` housing and sliding
   fingers (`left_finger`, `right_finger`, range `0` to `0.0475` m), modeled
   with primitive box colliders (`finger_collision` on the inner grip pads) and
   a 2×3 grid of `sphere_collision` geoms per fingertip.
4. Added `wrist_camera` on `link_6` at the optical-frame extrinsics from
   `yam_station_linear_4310_d405.xml`, with a `wrist_cam` camera using RealSense
   D405 intrinsics, four collision boxes covering the D405 housing and bracket,
   and an inertial for the 60 g D405 plus solid PLA bracket.

## License

This model is released under an [MIT License](LICENSE).
