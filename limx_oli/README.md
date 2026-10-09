# LimX Oli Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.0 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a robot description (MJCF) of the [Oli humanoid
robot](https://www.limxdynamics.com/) (HU_D04) developed by [LimX
Dynamics](https://www.limxdynamics.com/). It is derived from the [publicly
available MJCF
description](https://github.com/limxdynamics/humanoid-description/blob/master/HU_D04_description/xml/HU_D04_01.xml).

<p float="left">
  <img src="oli.png" width="400">
</p>

## MJCF derivation steps

1. Copied the MJCF description from
   [humanoid-description](https://github.com/limxdynamics/humanoid-description).
2. Replaced the `meshdir` compiler directive with `assets` and moved the STL
   meshes into `assets/`.
3. Merged the two `<asset>` blocks into one and moved the floor, light and
   tracking camera out of the model file into `scene.xml`.
4. Reformatted the XML to the Menagerie style.

## Notes

Oli's ankles and waist are driven through parallel linkages. The MJCF models
these as closed loops with `equality` constraints, matching the real robot
rather than the serial approximation used by the URDF. The gripper and
dexterous-hand variants of HU_D04 are not included here; see
[humanoid-description](https://github.com/limxdynamics/humanoid-description)
for those.

## License

This model is released under an [Apache-2.0 License](LICENSE).
