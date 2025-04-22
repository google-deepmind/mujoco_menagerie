# Shadow DEX-EE Hand Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.1.7 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the
[SHADOW-DEX-EE-HAND](https://www.shadowrobot.com/new-shadow-hand/) developed by
the [Shadow Robot Company](https://www.shadowrobot.com/). The original assets
were provided directly by [Shadow Robot Company](https://www.shadowrobot.com/)
under the [Apache 2.0 License](LICENSE).

<p float="left">
  <img src="shadow_dexee.png" width="400">
</p>

## MJCF

The source of truth for the Shadow DEX-EE Hand in MuJoCo can be found in
`shadow_dexee.xml`.

This model was evolved alongside hardware development. The initial release
matches the hardware version at release (without tactile sensors).

Currently only joint position actuators are available. Other actuation modes to
follow.

## License

This model is released under an [Apache-2.0 License](LICENSE).
