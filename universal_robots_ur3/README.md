# Universal Robots UR3 Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.3 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the
UR3 developed by
[Universal Robots](https://www.universal-robots.com/). It is derived from the
[publicly available URDF description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (version 4.4.0).

<p float="left">
  <img src="ur3.png" width="400">
</p>

### URDF → MJCF derivation steps

1. Generated the URDF from the xacro description with `ur_type:=ur3`.
2. Converted the DAE [mesh files](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/4.4.0/meshes) to OBJ format, with one file per color,
   using the colors of the Menagerie UR5e and UR10e models. The rubber seal rings that some meshes keep in
   the material of the joint housing are found by their shape and colored black; the seal rings that
   the meshes do not model are added as thin black bands that follow the housing. Screw heads are blue,
   like the caps they hold.
   The vertex normals are recomputed and coarse curved surfaces are subdivided, so that the meshes
   shade smoothly. Seal rings that the meshes model flush with the housing are raised by 0.3 mm, so
   that the housing does not show through their edges.
   The meshes of `wrist_1_link` and `wrist_2_link` are moved 1.9 mm along the wrist 1 axis, which
   centers the wrist 2 housing on its joint axis.
3. Used the same body frames as the Menagerie UR5e and UR10e models, so that joint angles, the `home`
   keyframe and the `attachment_site` have the same meaning for all Universal Robots models.
4. Copied the inertial parameters from the URDF.
5. Added a tracking light to the base.
6. Extracted common properties into the `<default>` section.
7. Added position-controlled actuators. Force limits are the maximum joint torques of the URDF. The gains
   are computed from the dynamics of the model: the stiffness keeps the error under gravity below
   0.005 rad in every configuration and reaches the torque limit at 0.05 rad, and the
   damping is the lowest for which 0.5 rad steps from the home pose overshoot by less than 2%.
   The armature is 0.1, as in the UR5e and UR10e models. These values are not identified on a real
   robot -- contributions are more than welcome to improve them.
8. Added home joint configuration as a `keyframe`.
9. Designed collision geometries with capsules (and a cylinder for the tool flange) that follow the shape
   of each link.
10. Added `scene.xml`, which includes the robot, with a textured ground plane, skybox, and haze.

### Notes

`wrist_3_joint` has no position limit, as on the real robot.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
