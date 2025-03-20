## ARX L5 Description (MJCF)

> Requires MuJoCo 2.3.4 or later.

### Overview

This package contains a simplified robot description (MJCF) of the [ARX L5](https://arx-x.com/). It is derived from the publicly available [model](https://github.com/ARXroboticsX/ARX_Model/tree/master/X5/X5A/urdf).

<p float="left">
  <img src="arx_l5.png" width="400">
</p>

### Derivation steps

1.  Added `<mujoco> <compiler balanceinertia="true" discardvisual="false"/> </mujoco>` to the URDF's
    `<robot>` clause in order to preserve visual geometries.
2.  Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3.  Converted the .stls to .objs and replaced the original .stls with them (since each .obj in MuJoCo can have 1 color).
4.  Merged similar materials between the .objs
5.  Created a `<default>` section to define common properties for joints, actuators, and geoms.
6.  Added an equality constraint so that the right finger mimics the position of the left finger.
7.  Manually designed box collision geoms for the gripper.
8.  Added `exclude` clause to prevent collisions between `base_link` and `link1`.
9.  Added position controlled actuators.
10. Added `impratio=10` and `cone=elliptic` for better noslip.
11. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under an [MIT License](LICENSE).

## Acknowledgement

This model was graciously contributed by [Jonathan Zamora](https://jonzamora.dev/).
