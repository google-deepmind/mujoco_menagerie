# Dynamixel 2R

## Overview

This package contains a 2R testbench arm featuring MX-64 and MX-106 dynamixel actuators.

The motor parameters are the best fit using system identification. However, as described in the
[related paper](https://arxiv.org/pdf/2410.08650v1), enhanced friction modeling can be used to
greatly improve the fidelity of the simulation of such system.

<p float="left">
  <img src="dynamixel_2r.png" width="400">
</p>

## CAD → MJCF conversion

The model was converted using [onshape-to-robot](https://onshape-to-robot.readthedocs.io/) from the [Onshape CAD assembly](https://cad.onshape.com/documents/e764ec92c6d6cd5bcf6d68d9/w/a8e95bf587cc977e65ed8aa5/e/8397fde05620ee94d449e6ed).

The [config.json](config.json) with export configuration file is also present in this package.

## See also

* [BAM repository](https://github.com/rhoban/bam): exploring extended modeling of the friction

## License

This model is released under a [MIT License](LICENSE)
