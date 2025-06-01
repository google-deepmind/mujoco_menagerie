# Google Barkour vB Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 3.0.0 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the Barkour vB Quadruped developed by Google.

<p float="left">
  <img src="barkour_vb.png" width="400">
</p>

## MJCF and URDF

The source of truth for the Barkour vB Quadruped can be found in `barkour_vb.xml`. The URDF in `barkour_vb_rev_1_0_head_straight.urdf` is provided for convenience.

### MJX

A version of the Barkour vB for use in [MJX](https://mujoco.readthedocs.io/en/stable/mjx.html) is available in `scene_mjx.xml` with the following changes:

* The solver `iterations` and `ls_iterations` are modified for performance.
* The `eulerdamp` flag is disabled.
* Frictionloss is removed.
* Collision geometries are only enabled between the feet and the plane.

## License

Copyright 2023 DeepMind Technologies Limited.

This model is released under an [Apache-2.0 License](LICENSE).

## Publications

If you use this work in an academic context, please cite the following publication:

    @misc{caluwaerts2023barkour,
          title={Barkour: Benchmarking Animal-level Agility with Quadruped Robots},
          author={Ken Caluwaerts and Atil Iscen and J. Chase Kew and Wenhao Yu and Tingnan Zhang and Daniel Freeman and Kuang-Huei Lee and Lisa Lee and Stefano Saliceti and Vincent Zhuang and Nathan Batchelor and Steven Bohez and Federico Casarini and Jose Enrique Chen and Omar Cortes and Erwin Coumans and Adil Dostmohamed and Gabriel Dulac-Arnold and Alejandro Escontrela and Erik Frey and Roland Hafner and Deepali Jain and Bauyrjan Jyenis and Yuheng Kuang and Edward Lee and Linda Luu and Ofir Nachum and Ken Oslund and Jason Powell and Diego Reyes and Francesco Romano and Feresteh Sadeghi and Ron Sloat and Baruch Tabanpour and Daniel Zheng and Michael Neunert and Raia Hadsell and Nicolas Heess and Francesco Nori and Jeff Seto and Carolina Parada and Vikas Sindhwani and Vincent Vanhoucke and Jie Tan},
          year={2023},
          eprint={2305.14654},
          archivePrefix={arXiv},
          primaryClass={cs.RO}
    }
