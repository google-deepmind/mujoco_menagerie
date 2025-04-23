# Unitree G1 Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.4 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the [G1 Humanoid
Robot](https://www.unitree.com/g1/) developed by [Unitree
Robotics](https://www.unitree.com/). It is derived from the [publicly available
MJCF
description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.xml). Specifically, this model has the fully actuated waist but not the full hands.

<p float="left">
  <img src="g1.png" width="400">
  <img src="g1_with_hands.png" width="400">
</p>

## MJCF derivation steps

1. Copied the MJCF description from [g1_description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.xml).
2. Manually edited the MJCF to extract common properties into the `<default>` section.
3. Added stand keyframe.
4. Added joint position actuators (needs tuning).
5. Applied similar edits to `g1_with_hands.xml`.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
