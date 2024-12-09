# IIT SoftFoot Description (MJCF)

Requires MuJoCo 3.2.0 or later.

## Overview

This package contains a simplified robot description (MJCF) of the IIT SoftFoot. The original MJCF and assets were provided directly by
[IIT Soft Robotics for Human Cooperation and Rehabilitation group](https://softbots.iit.it/) under an
[MIT License](LICENSE).

<p float="left">
  <img src="softfoot.png" width="400">
</p>

## Usage

The SoftFoot model is intended to be attached as an end effector to a legged robot. This can be achieved through several methods:
- Using the `<attach/>` element in the robot MJCF, and referencing the `attachment_cube` body from the softfoot MJCF. The provided example `scene.xml` uses this method.
- Procedurally attaching it through [`mjSpec`](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html).
- Adding the SoftFoot to the MJCF through the [`mjcf` python module](https://github.com/google-deepmind/dm_control/tree/main/dm_control/mjcf).


## License

This model is released under an [MIT License](LICENSE).
