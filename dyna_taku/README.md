# dyna Taku Description (MJCF)

Updated model tested with MuJoCo 3.3.7 and 4.0.1.

## Overview

This package contains a simplified MuJoCo model of **Taku**, a mobile dual-arm
robot developed by **dyna robotics**, with DynaClaw grippers.

![Taku](taku.png)

The robot has four steered wheels, four waist joints, three head joints,
two seven-joint arms and parallel grippers, with 31 actuators.
Camera, lidar and IMU mounting frames are included.

Open `scene.xml` in MuJoCo; `taku.xml` contains the robot model and `home` keyframe.

## Derivation

Based on the existing MJCF model in
[dyna-robotics/dyna_robot_description at `afa6168`](https://github.com/dyna-robotics/dyna_robot_description/tree/afa6168/dyna_taku).
This package adapts that MJCF directly; it does not perform a new URDF conversion.

- Incorporate cleaned visual meshes, soft-touch shell colors and DynaClaw
  geometry from [source revision `b5687ce`](https://github.com/dyna-robotics/dyna_robot_description/tree/b5687ce57cbe727be90f9732f53a9c522196ea04).
  Preserve the reviewed physical link transforms, masses, inertias, joint limits,
  actuator force limits and home keyframe.
- Use continuous pad/nail faces with box and capsule nail colliders, reducing
  each finger from 16 to 10 collision geoms. Preserve finger coupling and joint
  ranges; shift fingertip sites by approximately 0.595 mm to follow the calibrated
  visuals and colliders. Names remain unchanged.
- Split each wrist housing collider into two convex pieces and remove internal
  head yaw/pitch colliders that have no corresponding visual shell.
- Replace the solid base hull with separate convex hulls of the left, right and
  rear exterior shells plus the rear lidar, preserving the central U-shaped gap.
- Apply Menagerie XML formatting and defaults, inherit position control ranges
  from joint limits, and represent tool reference frames as sites.
- Adjust the `home` pose and position gains for stable initialization. These
  adjustments are simulation tuning, not measurements of the hardware. Folding
  position gains are `kp=5000`, `kv=200`; head-pitch gains are `kp=30`, `kv=1`.
  Hardware force limits are unchanged and no gravity compensation is assumed.

Joint `damping`, `armature` and `frictionloss` values are inherited from the source;
their measurement provenance is not documented here. Actuator `kp` and `kv` values
are inherited or manually tuned for simulation. None of these parameters is
claimed to be system-identified or validated against hardware measurements.

## Contact approximation

Green pads use sliding friction 1.5 and `solref="0.01 1"`; metal nails use
sliding friction 1.0 and `solref="0.003 1"`. Both use `condim=4`,
`solimp="0.95 0.99 0.001"` and torsional/rolling friction 0.005/0.0001.
Elliptic cones, `impratio=10` and tolerance 1e-10 reduce numerical creep.
These are provisional compliant contacts on rigid links, not deformable rubber
or hardware-calibrated material parameters. The solver changes apply globally.

Both hands, extracted from this model, held and released a 3 mm plate and a
cylinder at 1 ms and 0.5 ms timesteps without warnings (eight runs). Peak
penetration was 0.15–0.25 mm. This is a limited gripper regression, not a full
robot grasp benchmark or exhaustive collision validation.

## License

Copyright (c) 2026 dyna robotics.

This model, including its meshes and texture, is released under the
[Apache License 2.0](LICENSE).
