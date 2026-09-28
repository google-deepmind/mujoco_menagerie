# LimX Luna Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.0 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a robot description (MJCF) of the [Luna humanoid
robot](https://www.limxdynamics.com/) developed by [LimX
Dynamics](https://www.limxdynamics.com/). It is derived from the [publicly
available MJCF
description](https://github.com/limxdynamics/luna-description/blob/main/HU_L04_description/xml/HU_L04_01.xml).

<p float="left">
  <img src="luna.png" width="400">
</p>

## MJCF derivation steps

1. Copied the MJCF description from
   [luna-description](https://github.com/limxdynamics/luna-description).
2. Replaced the `meshdir` compiler directive with `assets` and moved the STL
   meshes into `assets/`.
3. Moved the floor, light and tracking camera out of the model file into
   `scene.xml`.
4. Reformatted the XML to the Menagerie style.

## Notes

Luna's ankles and waist are driven through parallel linkages. The MJCF models
these as closed loops with `equality` constraints (see the `<equality>`
section), matching the real robot rather than the serial approximation used by
the URDF.

## License

This model is released under an [Apache-2.0 License](LICENSE).
