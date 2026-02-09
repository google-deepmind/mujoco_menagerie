# Flexiv Robotics Rizon4 Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.3 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a MuJoCo robot description (MJCF) of [Rizon4](https://www.flexiv.com/products/rizon) developed by [Flexiv Robotics](https://www.flexiv.com/).

<p float="left">
  <img src="flexiv_rizon4.png" width="500">
</p>

## URDF → MJCF derivation steps

1. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
2. Manually edited the MJCF to extract common properties into the `<default>` section.
3. Added position-controlled actuators for the arm.
4. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.


## License

This model is released under an [Apache-2.0 License](LICENSE).
