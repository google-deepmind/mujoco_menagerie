<h1>
  <a href="#"><img alt="MuJoCo Menagerie" src="assets/banner.png" width="100%"></a>
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
  - [Usage](#usage)
    - [Via `robot-descriptions`](#via-robot-descriptions)
    - [Via `git clone`](#via-git-clone)
- [Model Quality and Contributing](#model-quality-and-contributing)
  - [Quick contributor setup](#quick-contributor-setup)
- [Menagerie Models](#menagerie-models)
- [Citing Menagerie](#citing-menagerie)
- [Acknowledgments](#acknowledgments)
- [Changelog](#changelog)
- [License and Disclaimer](#license-and-disclaimer)

## Getting Started

### Prerequisites

The minimum required MuJoCo version for each model is specified in its
respective README. You can download prebuilt binaries for MuJoCo from the GitHub
[releases page](https://github.com/google-deepmind/mujoco/releases/), or if you
are working with Python, you can install the native bindings from
[PyPI](https://pypi.org/project/mujoco/) via `pip install mujoco`. For
alternative installation instructions, see
[here](https://github.com/google-deepmind/mujoco#installation).

### Overview

The structure of Menagerie is illustrated below. For brevity, we have only
included one model directory since all others follow the exact same pattern.

```bash
├── unitree_go2
│   ├── assets
│   │   ├── base_0.obj
│   │   ├── ...
│   ├── go2.png
│   ├── go2.xml
│   ├── LICENSE
│   ├── README.md
│   └── scene.xml
│   └── go2_mjx.xml
│   └── scene_mjx.xml
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
- `<model>_mjx.xml`: contains an MJX-compatible version of the model. Not all
  models have an MJX variant.
- `scene_mjx.xml`: same as `scene.xml` but loads the MJX variant

Note that `<model>.xml` solely describes the model, i.e., no other entity is
defined in the kinematic tree. We leave additional body definitions for the
`scene.xml` file, as can be seen in the Shadow Hand
[`scene.xml`](shadow_hand/scene_right.xml).

### Usage

#### Via `robot-descriptions`

You can use the opensource
[`robot_descriptions`](https://github.com/robot-descriptions/robot_descriptions.py)
package to load any model in Menagerie. It is available on PyPI and can be
installed via `pip install robot_descriptions`.

Once installed, you can load a model of your choice as follows:

```python
import mujoco

# Loading a specific model description as an imported module.
from robot_descriptions import panda_mj_description
model = mujoco.MjModel.from_xml_path(panda_mj_description.MJCF_PATH)

# Directly loading an instance of MjModel.
from robot_descriptions.loaders.mujoco import load_robot_description
model = load_robot_description("panda_mj_description")

# Loading a variant of the model, e.g. panda without a gripper.
model = load_robot_description("panda_mj_description", variant="panda_nohand")
```

#### Via `git clone`

You can also directly clone this repository in the directory of your choice:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

You can then interactively explore the model using the Python viewer:

```bash
python -m mujoco.viewer --mjcf mujoco_menagerie/unitree_go2/scene.xml
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

The grading system will be applied to each model once a proper system
identification toolbox is created. We are currently planning to release
this toolbox later this year.

For more information regarding contributions, for example to add a new model to
Menagerie, see [CONTRIBUTING](CONTRIBUTING.md).

### Quick contributor setup

Two commands and you're set up. You need [`uv`](https://docs.astral.sh/uv/)
installed; that's the only prerequisite.

```bash
make install   # one-time: installs pre-commit + git hook
make all       # run every check CI runs (lint + format + license + XML + tests)
```

After `make install`, the fast checks fire automatically on every `git commit`.
Run `make all` before pushing. See [CONTRIBUTING.md](CONTRIBUTING.md) for the
full breakdown.

## Menagerie Models

> Click any thumbnail below to open the model in an in-browser MuJoCo viewer powered by [live.mujoco.org](https://live.mujoco.org).

<!-- BEGIN MODELS (auto-generated by `make gallery` — do not edit) -->

**Humanoids.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_h1/scene.xml' title='Open live preview for Unitree H1'><img src='assets/unitree_h1-h1.png' width=120></a> | Unitree H1 | 19 | [BSD-3-Clause](unitree_h1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/robotis_op3/scene.xml' title='Open live preview for Robotis OP3'><img src='assets/robotis_op3-op3.png' width=120></a> | Robotis OP3 | 20 | [Apache-2.0](robotis_op3/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_g1/scene.xml' title='Open live preview for Unitree G1'><img src='assets/unitree_g1-g1.png' width=120></a> | Unitree G1 | 29 | [BSD-3-Clause](unitree_g1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/pal_talos/scene_position.xml' title='Open live preview for TALOS'><img src='assets/pal_talos-talos.png' width=120></a> | TALOS | 44 | [Apache-2.0](pal_talos/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/booster_t1/scene.xml' title='Open live preview for Booster T1'><img src='assets/booster_t1-t1.png' width=120></a> | Booster T1 | 23 | [Apache-2.0](booster_t1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/toddlerbot_2xc/scene.xml' title='Open live preview for ToddlerBot 2XC'><img src='assets/toddlerbot_2xc-toddlerbot_2xc.png' width=120></a> | ToddlerBot 2XC | 44 | [MIT](toddlerbot_2xc/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/pndbotics_adam_lite/scene.xml' title='Open live preview for PNDbotics Adam_lite'><img src='assets/pndbotics_adam_lite-adam_lite.png' width=120></a> | PNDbotics Adam_lite | 25 | [MIT](pndbotics_adam_lite/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/apptronik_apollo/scene.xml' title='Open live preview for Apptronik Apollo'><img src='assets/apptronik_apollo-apptronik_apollo.png' width=120></a> | Apptronik Apollo | 32 | [Apache-2.0](apptronik_apollo/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/berkeley_humanoid/scene.xml' title='Open live preview for Berkeley Humanoid'><img src='assets/berkeley_humanoid-berkeley_humanoid.png' width=120></a> | Berkeley Humanoid | 12 | [BSD-3-Clause](berkeley_humanoid/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/fourier_n1/scene.xml' title='Open live preview for Fourier N1'><img src='assets/fourier_n1-n1.png' width=120></a> | Fourier N1 | 23 | [Apache-2.0](fourier_n1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/toddlerbot_2xm/scene.xml' title='Open live preview for ToddlerBot 2XM'><img src='assets/toddlerbot_2xm-toddlerbot_2xm.png' width=120></a> | ToddlerBot 2XM | 44 | [MIT](toddlerbot_2xm/LICENSE) |

**Quadrupeds.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_a1/scene.xml' title='Open live preview for Unitree A1'><img src='assets/unitree_a1-a1.png' width=120></a> | Unitree A1 | 12 | [BSD-3-Clause](unitree_a1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/google_barkour_v0/scene.xml' title='Open live preview for Google Barkour v0'><img src='assets/google_barkour_v0-barkour_v0.png' width=120></a> | Google Barkour v0 | 12 | [Apache-2.0](google_barkour_v0/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/anybotics_anymal_b/scene.xml' title='Open live preview for ANYmal B'><img src='assets/anybotics_anymal_b-anymal_b.png' width=120></a> | ANYmal B | 12 | [BSD-3-Clause](anybotics_anymal_b/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_go1/scene.xml' title='Open live preview for Unitree Go1'><img src='assets/unitree_go1-go1.png' width=120></a> | Unitree Go1 | 12 | [BSD-3-Clause](unitree_go1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/anybotics_anymal_c/scene.xml' title='Open live preview for ANYmal C'><img src='assets/anybotics_anymal_c-anymal_c.png' width=120></a> | ANYmal C | 12 | [BSD-3-Clause](anybotics_anymal_c/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/google_barkour_vb/scene.xml' title='Open live preview for Google Barkour vB'><img src='assets/google_barkour_vb-barkour_vb.png' width=120></a> | Google Barkour vB | 12 | [Apache-2.0](google_barkour_vb/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_go2/scene.xml' title='Open live preview for Unitree Go2'><img src='assets/unitree_go2-go2.png' width=120></a> | Unitree Go2 | 12 | [BSD-3-Clause](unitree_go2/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/boston_dynamics_spot/scene.xml' title='Open live preview for Boston Dynamics Spot'><img src='assets/boston_dynamics_spot-spot_arm.png' width=120></a> | Boston Dynamics Spot | 19 | [BSD-3-Clause](boston_dynamics_spot/LICENSE) |

**Bipeds.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/agility_cassie/scene.xml' title='Open live preview for Agility Cassie'><img src='assets/agility_cassie-cassie.png' width=120></a> | Agility Cassie | 28 | [MIT](agility_cassie/LICENSE) |

**Biomechanical.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/flybody/scene.xml' title='Open live preview for Flybody'><img src='assets/flybody-fruitfly.png' width=120></a> | Flybody | 102 | [Apache-2.0](flybody/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/iit_softfoot/scene.xml' title='Open live preview for IIT SoftFoot'><img src='assets/iit_softfoot-softfoot.png' width=120></a> | IIT SoftFoot | 92 | [BSD-3-Clause](iit_softfoot/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/ms_human_700/scene.xml' title='Open live preview for MS-Human-700'><img src='assets/ms_human_700-MS-Human-700.png' width=120></a> | MS-Human-700 | 85 | [Apache-2.0](ms_human_700/LICENSE) |

**Dual Arms.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/aloha/scene.xml' title='Open live preview for ALOHA'><img src='assets/aloha-aloha.png' width=120></a> | ALOHA | 16 | [BSD-3-Clause](aloha/LICENSE) |

**Mobile Manipulators.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/google_robot/scene.xml' title='Open live preview for Google Robot'><img src='assets/google_robot-robot.png' width=120></a> | Google Robot | 9 | [Apache-2.0](google_robot/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/hello_robot_stretch/scene.xml' title='Open live preview for Hello Robot Stretch 2'><img src='assets/hello_robot_stretch-stretch.png' width=120></a> | Hello Robot Stretch 2 | 17 | [BSD-3-Clause-Clear](hello_robot_stretch/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/stanford_tidybot/scene.xml' title='Open live preview for Stanford TidyBot'><img src='assets/stanford_tidybot-tidybot.png' width=120></a> | Stanford TidyBot | 18 | [MIT](stanford_tidybot/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/hello_robot_stretch_3/scene.xml' title='Open live preview for Hello Robot Stretch 3'><img src='assets/hello_robot_stretch_3-stretch.png' width=120></a> | Hello Robot Stretch 3 | 20 | [Apache-2.0](hello_robot_stretch_3/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/pal_tiago/scene_position.xml' title='Open live preview for TIAGo'><img src='assets/pal_tiago-tiago.png' width=120></a> | TIAGo | 22 | [Apache-2.0](pal_tiago/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/pal_tiago_dual/scene_position.xml' title='Open live preview for TIAGo++'><img src='assets/pal_tiago_dual-tiago_dual.png' width=120></a> | TIAGo++ | 25 | [Apache-2.0](pal_tiago_dual/LICENSE) |

**Drones.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/skydio_x2/scene.xml' title='Open live preview for Skydio X2'><img src='assets/skydio_x2-x2.png' width=120></a> | Skydio X2 | 0 | [Apache-2.0](skydio_x2/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/bitcraze_crazyflie_2/scene.xml' title='Open live preview for Bitcraze Crazyflie 2'><img src='assets/bitcraze_crazyflie_2-cf2.png' width=120></a> | Bitcraze Crazyflie 2 | 0 | [MIT](bitcraze_crazyflie_2/LICENSE) |

**Arms.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/franka_emika_panda/scene.xml' title='Open live preview for Franka Emika Panda'><img src='assets/franka_emika_panda-panda.png' width=120></a> | Franka Emika Panda | 9 | [Apache-2.0](franka_emika_panda/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/franka_fr3/scene.xml' title='Open live preview for Franka Robotics FR3'><img src='assets/franka_fr3-fr3.png' width=120></a> | Franka Robotics FR3 | 7 | [Apache-2.0](franka_fr3/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/ufactory_lite6/scene.xml' title='Open live preview for Lite 6'><img src='assets/ufactory_lite6-lite6.png' width=120></a> | Lite 6 | 6 | [BSD-3-Clause](ufactory_lite6/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/unitree_z1/scene.xml' title='Open live preview for Unitree Z1'><img src='assets/unitree_z1-z1.png' width=120></a> | Unitree Z1 | 6 | [BSD-3-Clause](unitree_z1/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/universal_robots_ur5e/scene.xml' title='Open live preview for Universal Robots UR5e'><img src='assets/universal_robots_ur5e-ur5e.png' width=120></a> | Universal Robots UR5e | 6 | [BSD-3-Clause](universal_robots_ur5e/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/rethink_robotics_sawyer/scene.xml' title='Open live preview for Rethink Robotics Sawyer'><img src='assets/rethink_robotics_sawyer-sawyer.png' width=120></a> | Rethink Robotics Sawyer | 7 | [Apache-2.0](rethink_robotics_sawyer/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/universal_robots_ur10e/scene.xml' title='Open live preview for Universal Robots UR10e'><img src='assets/universal_robots_ur10e-ur10e.png' width=120></a> | Universal Robots UR10e | 6 | [BSD-3-Clause](universal_robots_ur10e/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/kuka_iiwa_14/scene.xml' title='Open live preview for KUKA LBR iiwa 14'><img src='assets/kuka_iiwa_14-iiwa14.png' width=120></a> | KUKA LBR iiwa 14 | 7 | [BSD-3-Clause](kuka_iiwa_14/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/trossen_vx300s/scene.xml' title='Open live preview for ViperX 300 6DOF'><img src='assets/trossen_vx300s-vx300s.png' width=120></a> | ViperX 300 6DOF | 8 | [BSD-3-Clause](trossen_vx300s/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/ufactory_xarm7/scene.xml' title='Open live preview for xArm7'><img src='assets/ufactory_xarm7-xarm7.png' width=120></a> | xArm7 | 13 | [BSD-3-Clause](ufactory_xarm7/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/kinova_gen3/scene.xml' title='Open live preview for Kinova Gen3'><img src='assets/kinova_gen3-gen3.png' width=120></a> | Kinova Gen3 | 7 | [BSD-3-Clause](kinova_gen3/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/agilex_piper/scene.xml' title='Open live preview for AgileX PiPER'><img src='assets/agilex_piper-piper.png' width=120></a> | AgileX PiPER | 8 | [MIT](agilex_piper/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/flexiv_rizon4/scene.xml' title='Open live preview for Flexiv Robotics Rizon4'><img src='assets/flexiv_rizon4-flexiv_rizon4.png' width=120></a> | Flexiv Robotics Rizon4 | 7 | [Apache-2.0](flexiv_rizon4/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/arx_l5/scene.xml' title='Open live preview for ARX L5'><img src='assets/arx_l5-arx_l5.png' width=120></a> | ARX L5 | 8 | [BSD-3-Clause](arx_l5/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/flexiv_rizon4s/scene.xml' title='Open live preview for Flexiv Robotics Rizon4S'><img src='assets/flexiv_rizon4s-flexiv_rizon4s.png' width=120></a> | Flexiv Robotics Rizon4S | 7 | [Apache-2.0](flexiv_rizon4s/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/trossen_wx250s/scene.xml' title='Open live preview for WidowX 250 6DOF'><img src='assets/trossen_wx250s-wx250s.png' width=120></a> | WidowX 250 6DOF | 8 | [BSD-3-Clause](trossen_wx250s/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/trs_so_arm100/scene.xml' title='Open live preview for Standard Open Arm-100 5DOF - Version 1.3'><img src='assets/trs_so_arm100-so_arm100.png' width=120></a> | Standard Open Arm-100 5DOF - Version 1.3 | 6 | [Apache-2.0](trs_so_arm100/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/low_cost_robot_arm/scene.xml' title='Open live preview for Low-Cost Robot Arm'><img src='assets/low_cost_robot_arm-low_cost_robot_arm.png' width=120></a> | Low-Cost Robot Arm | 6 | [Apache-2.0](low_cost_robot_arm/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/i2rt_yam/scene.xml' title='Open live preview for Yet Another Manipulator (YAM)'><img src='assets/i2rt_yam-yam.png' width=120></a> | Yet Another Manipulator (YAM) | 8 | [MIT](i2rt_yam/LICENSE) |

**End-effectors.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/franka_emika_panda/hand.xml' title='Open live preview for Panda Gripper'><img src='assets/franka_emika_panda-hand.png' width=120></a> | Panda Gripper | 2 | [Apache-2.0](franka_emika_panda/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/wonik_allegro/scene_left.xml' title='Open live preview for Allegro Hand V3'><img src='assets/wonik_allegro-left_hand.png' width=120></a> | Allegro Hand V3 | 16 | [BSD-2-Clause](wonik_allegro/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/shadow_hand/scene_left.xml' title='Open live preview for Shadow Hand E3M5'><img src='assets/shadow_hand-left_hand.png' width=120></a> | Shadow Hand E3M5 | 24 | [Apache-2.0](shadow_hand/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/robotiq_2f85/scene.xml' title='Open live preview for Robotiq 2F-85'><img src='assets/robotiq_2f85-2f85.png' width=120></a> | Robotiq 2F-85 | 8 | [BSD-2-Clause](robotiq_2f85/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/ufactory_xarm7/hand.xml' title='Open live preview for xarm7 Gripper'><img src='assets/ufactory_xarm7-hand.png' width=120></a> | xarm7 Gripper | 6 | [BSD-3-Clause](ufactory_xarm7/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/shadow_dexee/scene.xml' title='Open live preview for Shadow DEX-EE Hand'><img src='assets/shadow_dexee-shadow_dexee.png' width=120></a> | Shadow DEX-EE Hand | 12 | [Apache-2.0](shadow_dexee/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/leap_hand/scene_left.xml' title='Open live preview for Leap Hand'><img src='assets/leap_hand-left_hand.png' width=120></a> | Leap Hand | 16 | [MIT](leap_hand/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/umi_gripper/scene.xml' title='Open live preview for UMI-Gripper'><img src='assets/umi_gripper-umi_gripper.png' width=120></a> | UMI-Gripper | 8 | [MIT](umi_gripper/LICENSE) |
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/sharpa_wave/scene_left.xml' title='Open live preview for Sharpa Wave'><img src='assets/sharpa_wave-left_hand.png' width=120></a> | Sharpa Wave | 22 | [Apache-2.0](sharpa_wave/LICENSE) |

**Mobile Bases.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/robot_soccer_kit/scene.xml' title='Open live preview for Robot soccer kit omnidirectional'><img src='assets/robot_soccer_kit-robot_soccer_kit.png' width=120></a> | Robot soccer kit omnidirectional | 64 | [MIT](robot_soccer_kit/LICENSE) |

**Miscellaneous.**

| Preview | Name | DoFs | License |
|:---:|---|---|---|
| <a href='https://live.mujoco.org/?model=github:google-deepmind/mujoco_menagerie/main/realsense_d435i/d435i.xml' title='Open live preview for Realsense D435i'><img src='assets/realsense_d435i-d435i.png' width=120></a> | Realsense D435i | 0 | [Apache-2.0](realsense_d435i/LICENSE) |

<!-- END MODELS -->

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

The models in this repository are based on third-party models designed by many talented people, and would not have been possible without their generous open-source contributions. We would like to acknowledge all the designers and engineers who made MuJoCo Menagerie possible.

We'd like to thank Pedro Vergani for his help with visuals and design.

The main effort required to make this repository publicly available was undertaken by [Kevin Zakka](https://kzakka.com/), with help from the Robotics Simulation team at Google DeepMind.

This project has also benefited from contributions by members of the broader community — see the [CONTRIBUTORS.md](./CONTRIBUTORS.md) for a full list.

## Changelog

For a summary of key updates across the repository, see the [global CHANGELOG.md](./CHANGELOG.md).

Each individual model also includes its own `CHANGELOG.md` file with model-specific updates, linked directly from the corresponding README.

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
