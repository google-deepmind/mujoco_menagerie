# ANYmal C Description (MJCF)

## Overview

This package contains a simplified robot description (MJCF) of the [ANYmal C
robot](https://www.anybotics.com/anymal) developed by
[ANYbotics](https://www.anybotics.com). It is derived from the [publicly
available URDF
description](https://github.com/ANYbotics/anymal_c_simple_description).

<p float="left">
  <img src="anymal_c.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Converted the DAE [mesh
   files](https://github.com/ANYbotics/anymal_c_simple_description/tree/master/meshes)
   to OBJ format using [Blender](https://www.blender.org/).
2. Processed `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Added `<mujoco> <compiler discardvisual="false"/> </mujoco>` to the
   [URDF](https://github.com/ANYbotics/anymal_b_simple_description/blob/master/urdf/anymal.urdf)'s
   `<robot>` clause in order to preserve visual geometries.
4. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
5. Added a `<freejoint/>` to the base, and a tracking light.
6. Manually edited the MJCF to extract common properties into the `<default>` section.
7. Added `<exclude>` clauses to prevent collisions between the base and the thighs.
8. Added position-controlled actuators, roughly corresponding to the motor spec
   [here](https://doi.org/10.1080/01691864.2017.1378591).
9. Added joint damping to correspond to D gains of the same spec.
10. Added joint frictionloss to correspond to torque resolution of the same spec.
11. Softened the contacts of the feet to approximate the effect of rubber.
12. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
