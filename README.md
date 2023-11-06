<h1>
  <a href="#"><img alt="MuJoCo Menagerie" src="banner.png" width="100%"></a>
</h1>

<p>
  <a href="https://github.com/google-deepmind/mujoco_menagerie/actions/workflows/build.yml?query=branch%3Amain" alt="GitHub Actions">
    <img src="https://img.shields.io/github/actions/workflow/status/google-deepmind/mujoco_menagerie/build.yml?branch=main">
  </a>
  <a href="https://mujoco.readthedocs.io/en/latest/models.html" alt="Documentation">
    <img src="https://readthedocs.org/projects/mujoco/badge/?version=latest">
  </a>
  <a href="https://github.com/google-deepmind/mujoco_menagerie/blob/main/CONTRIBUTING.md">
    <img src="https://img.shields.io/badge/PRs-welcome-green.svg" alt="PRs" height="20">
  </a>
</p>

**Menagerie** is a collection of high-quality models for the
[MuJoCo](https://github.com/google-deepmind/mujoco) physics engine, curated by
Google DeepMind.

A physics simulator is only as good as the model it is simulating, and in a
powerful simulator like MuJoCo with many modeling options, it is easy to create
"bad" models which do not behave as expected. The goal of this collection is to
provide the community with a curated library of well-designed models that work
well right out of the gate.

- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Overview](#overview)
  - [Installation and Usage](#installation-and-usage)
- [Model Quality and Contributing](#model-quality-and-contributing)
- [Menagerie Models](#menagerie-models)
- [Citing Menagerie](#citing-menagerie)
- [Acknowledgments](#acknowledgments)
- [License and Disclaimer](#license-and-disclaimer)

## Getting Started

### Prerequisites

The minimum required MuJoCo version for each model is specified in its
respective README. You can download prebuilt binaries from the GitHub
[releases page](https://github.com/google-deepmind/mujoco/releases/), or if you
are working with Python, you can install the native bindings from
[PyPI](https://pypi.org/project/mujoco/) via `pip install mujoco`. For
alternative installation instructions, see
[here](https://github.com/google-deepmind/mujoco#installation).

### Overview

The structure of Menagerie is illustrated below. For brevity, we have only
included one model directory since all others follow the exact same pattern.

```bash
├── agility_cassie
│   ├── assets
│   │   ├── achilles-rod.obj
│   │   ├── ...
│   ├── cassie.png
│   ├── cassie.xml
│   ├── LICENSE
│   ├── README.md
│   └── scene.xml
```

- `assets`: stores the 3D meshes (.stl or .obj) of the model used for visual and
  collision purposes
- `LICENSE`: describes the copyright and licensing terms of the model
- `README.md`: contains detailed steps describing how the model's MJCF XML file
  was generated
- `<model>.xml`: contains the MJCF definition of the model
- `scene.xml`: includes `<model>.xml` with a plane, a light source and
  potentially other objects
- `<model>.png`: a PNG image of `scene.xml`

Note that `<model>.xml` solely describes the model, i.e., no other entity is
defined in the kinematic tree. We leave additional body definitions for the
`scene.xml` file, as can be seen in the Shadow Hand
[`scene.xml`](shadow_hand/scene_right.xml).

### Installation and Usage

To install Menagerie, simply clone the repository in the directory of your
choice:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

The easiest way to interactively explore a model is to load it in the
[simulate](https://github.com/google-deepmind/mujoco/tree/main/simulate) binary
which ships with every MuJoCo distribution. This amounts to simply dragging then
dropping the `scene.xml` file into the simulate window. If you prefer, you can
also use the command line to launch `simulate` and directly pass in the path to
the XML.

Outside of interactive simulation, you can load a model exactly as you would
with any other XML file in MuJoCo, either via the C/C++ API:

```c++
#include <mujoco.h>

mjModel* model = mj_loadXML("unitree_a1/a1.xml", nullptr, nullptr, 0);
mjData* data = mj_makeData(model);
mj_step(model, data);
```

or via Python:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("unitree_a1/a1.xml")
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
```

If you have further questions, please check out our [FAQ](FAQ.md).

## Model Quality and Contributing

Our goal is to eventually make all Menagerie models as faithful as possible to
the real system they are being modeled after. Improving model quality is an
ongoing effort, and the current state of many models is not necessarily
as good as it could be.

However, by releasing Menagerie in its current state, we hope to consolidate
and increase visibility for community contributions. To help Menagerie users
set proper expectations around the quality of each model, we introduce the
following grading system:

| Grade | Description                                                 |
|-------|-------------------------------------------------------------|
| A+    | Values are the product of proper system identification      |
| A     | Values are realistic, but have not been properly identified |
| B     | Stable, but some values are unrealistic                     |
| C     | Conditionally stable, can be significantly improved         |

For more information regarding contributions, for example to add a new model to
Menagerie, see [CONTRIBUTING](CONTRIBUTING.md).

## Menagerie Models

| Robot             | Preview       | Grade   |
| ----------------- | ------------- | :-----: |
| Hands             |               |         |
| [Shadow E3M5](shadow_hand/README.md)|[<img src="shadow_hand/shadow_hand.png" width="400">](shadow_hand/README.md)|A|
| [Allegro V3](wonik_allegro/README.md)|[<img src="wonik_allegro/allegro_hand.png" width="400">](wonik_allegro/README.md)|C|
| [Robotiq 2F-85](robotiq_2f85/README.md)|[<img src="robotiq_2f85/2f85.png" width="400">](robotiq_2f85/README.md)|B|
| Bipeds             |               |         |
| [Cassie](agility_cassie/README.md)|[<img src="agility_cassie/cassie.png" width="400">](agility_cassie/README.md)|C|
| [Robotis OP3](robotis_op3/README.md)|[<img src="robotis_op3/op3.png" width="400">](robotis_op3/README.md)|A+|
| Quadrupeds         |              |         |
| [ANYmal B](anybotics_anymal_b/README.md)|[<img src="anybotics_anymal_b/anymal_b.png" width="400">](anybotics_anymal_b/README.md)|A|
| [ANYmal C](anybotics_anymal_c/README.md)|[<img src="anybotics_anymal_c/anymal_c.png" width="400">](anybotics_anymal_c/README.md)|B|
| [Unitree A1](unitree_a1/README.md)|[<img src="unitree_a1/a1.png" width="400">](unitree_a1/README.md)|B|
| [Unitree Go1](unitree_go1/README.md)|[<img src="unitree_go1/go1.png" width="400">](unitree_go1/README.md)|B|
| [Unitree Go2](unitree_go2/README.md)|[<img src="unitree_go2/go2.png" width="400">](unitree_go2/README.md)|B|
| [Google Barkour v0](google_barkour_v0/README.md)|[<img src="google_barkour_v0/barkour_v0.png" width="400">](google_barkour_v0/README.md)|A|
| [Google Barkour vb](google_barkour_vb/README.md)|[<img src="google_barkour_vb/barkour_vb.png" width="400">](google_barkour_vb/README.md)|A|
| Arms               |              |         |
| [Panda](franka_emika_panda/README.md)|[<img src="franka_emika_panda/panda.png" width="400">](franka_emika_panda/README.md)|B|
| [UR5e](universal_robots_ur5e/README.md)|[<img src="universal_robots_ur5e/ur5e.png" width="400">](universal_robots_ur5e/README.md)|B|
| [UR10e](universal_robots_ur10e/README.md)|[<img src="universal_robots_ur10e/ur10e.png" width="400">](universal_robots_ur10e/README.md)|C|
| [KUKA iiwa 14](kuka_iiwa_14/README.md)|[<img src="kuka_iiwa_14/iiwa_14.png" width="400">](kuka_iiwa_14/README.md)|B|
| [Sawyer](rethink_robotics_sawyer/README.md)|[<img src="rethink_robotics_sawyer/sawyer.png" width="400">](rethink_robotics_sawyer/README.md)|C|
| [xArm7](ufactory_xarm7/README.md)|[<img src="ufactory_xarm7/xarm7.png" width="400">](ufactory_xarm7/README.md)|C|
| [Lite 6](ufactory_lite6/README.md)|[<img src="ufactory_lite6/lite6.png" width="400">](ufactory_lite6/README.md)|C|
| Mobile Manipulators|              |         |
| [Google Robot](google_robot/README.md)|[<img src="google_robot/robot.png" width="400">](google_robot/README.md)|B|
| [Stretch 2](hello_robot_stretch/README.md)|[<img src="hello_robot_stretch/stretch.png" width="400">](hello_robot_stretch/README.md)|C|
| Humanoids          |              |         |
| [Unitree H1](unitree_h1/README.md)|[<img src="unitree_h1/h1.png" width="400">](unitree_h1/README.md)|B|
| Drones             |              |         |
| [Skydio X2](skydio_x2/README.md)|[<img src="skydio_x2/x2.png" width="400">](skydio_x2/README.md)|A|
| Other              |              |         |
| [Realsense D435i](realsense_d435i/README.md)|[<img src="realsense_d435i/d435i.png" width="400">](realsense_d435i/README.md)|B|

For corresponding embedded videos, see the MuJoCo
[documentation](https://mujoco.readthedocs.io/en/latest/models.html).

## Citing Menagerie

If you use Menagerie in your work, please use the following citation:

```bibtex
@software{menagerie2022github,
  author = {Zakka, Kevin and Tassa, Yuval and {MuJoCo Menagerie Contributors}},
  title = {{MuJoCo Menagerie: A collection of high-quality simulation models for MuJoCo}},
  url = {http://github.com/google-deepmind/mujoco_menagerie},
  year = {2022},
}
```

## Acknowledgments

The models in this repository are based on third-party models designed by many
talented people, and would not have been possible without their generous
open-source contributions. We would like to acknowledge all the designers
and engineers who made MuJoCo Menagerie possible.

We'd like to thank Pedro Vergani for his help with visuals and design.

The main effort required to make this repository publicly available was
undertaken by [Kevin Zakka](https://kzakka.com/), with help from the Robotics
Simulation team at Google DeepMind.

## License and Disclaimer

XML and asset files in each individual model directory of this repository are
subject to different license terms. Please consult the `LICENSE` files under
each specific model subdirectory for the relevant license and copyright
information.

All other content is Copyright 2022 DeepMind Technologies Limited and licensed
under the Apache License, Version 2.0. A copy of this license is provided in the
top-level LICENSE file in this repository.
You can also obtain it from https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.
