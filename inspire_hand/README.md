# Inspire Hand RH56DFX Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.6 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains assets of the "RH56DFX" version of the Inspire Hand (often attached to Unitree G1/H1 Humanoids), including both right-handed and left-handed versions.
The original URDF and assets can be found directly on
[Inspire Hand Company](https://www.inspire-robots.com/).

<p float="left">
  <img src="inspire_hand.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Used the hands .xmls through by [ARISE](https://github.com/ARISE-Initiative/robosuite) (to the best of my knowledge). 
2. Broke most of the .stls into multiple smaller versions using blender to match the inspire hand colors.
3. Manually edited the MJCF to extract common properties into the `<default>` section.
4. Added position-controlled actuators.
5. Added `impratio=10` for better noslip.
6. Added `scene_left.xml` and `scene_right.xml` which include the robot, with
    an object, textured groundplane, skybox, and haze.

## License

This model is released under a [BSD-3-Clause License](LICENSE).