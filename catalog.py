# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Per-robot metadata that cannot be derived from the model directory. Shared by
# generate_gallery.py and build_registry.py; keys are <model_dir>/<xml stem>.

import enum
import pathlib
import re


class ModelType(int, enum.Enum):
  ARM = 0
  DUAL_ARM = 1
  END_EFFECTOR = 2
  MOBILE_MANIPULATOR = 3
  MOBILE_BASE = 4
  QUADRUPED = 5
  BIPED = 6
  HUMANOID = 7
  DRONE = 8
  BIOMECHANICAL = 9
  MISC = 10


# Display name overrides for robots whose model-dir README title is too
# verbose, doesn't exist, or describes a different variant than the entry.
# Everything else is extracted from the first `# <name> Description (MJCF)`
# line of `<maker>/README.md`.
DISPLAY_NAME_OVERRIDE = {
  'franka_emika_panda/hand': 'Panda Gripper',
  'ufactory_xarm7/hand': 'xarm7 Gripper',
  'franka_fr3_v2/fr3v2': 'Franka Robotics FR3 v2',
  'robotiq_2f85_v4/2f85': 'Robotiq 2F-85 v4',
}


_README_TITLE_SUFFIX = re.compile(
  r'\s*(description\s*)?\(mjcf\)\s*$|\s+description\s*$',
  re.IGNORECASE,
)


def display_name(robot):
  if robot in DISPLAY_NAME_OVERRIDE:
    return DISPLAY_NAME_OVERRIDE[robot]
  maker = robot.split('/')[0]
  readme = pathlib.Path(f'{maker}/README.md')
  if readme.exists():
    title = (
      readme.read_text(encoding='utf-8')
      .splitlines()[0]
      .strip()
      .lstrip('#')
      .strip()
    )
    title = _README_TITLE_SUFFIX.sub('', title).rstrip()
    if title:
      return title
  return robot.split('/')[-1]


MODEL_MAP = {
  'franka_emika_panda/panda': ModelType.ARM,
  'franka_emika_panda/hand': ModelType.END_EFFECTOR,
  'franka_fr3/fr3': ModelType.ARM,
  'ufactory_lite6/lite6': ModelType.ARM,
  'flybody/fruitfly': ModelType.BIOMECHANICAL,
  'wonik_allegro/left_hand': ModelType.END_EFFECTOR,
  'shadow_hand/left_hand': ModelType.END_EFFECTOR,
  'skydio_x2/x2': ModelType.DRONE,
  'unitree_h1/h1': ModelType.HUMANOID,
  'bitcraze_crazyflie_2/cf2': ModelType.DRONE,
  'google_robot/robot': ModelType.MOBILE_MANIPULATOR,
  'unitree_a1/a1': ModelType.QUADRUPED,
  'google_barkour_v0/barkour_v0': ModelType.QUADRUPED,
  'anybotics_anymal_b/anymal_b': ModelType.QUADRUPED,
  'unitree_go1/go1': ModelType.QUADRUPED,
  'unitree_z1/z1': ModelType.ARM,
  'anybotics_anymal_c/anymal_c': ModelType.QUADRUPED,
  'agility_cassie/cassie': ModelType.BIPED,
  'realsense_d435i/d435i': ModelType.MISC,
  'universal_robots_ur5e/ur5e': ModelType.ARM,
  'aloha/aloha': ModelType.DUAL_ARM,
  'rethink_robotics_sawyer/sawyer': ModelType.ARM,
  'robotis_op3/op3': ModelType.HUMANOID,
  'universal_robots_ur10e/ur10e': ModelType.ARM,
  'kuka_iiwa_14/iiwa14': ModelType.ARM,
  'trossen_vx300s/vx300s': ModelType.ARM,
  'unitree_g1/g1': ModelType.HUMANOID,
  'robotiq_2f85/2f85': ModelType.END_EFFECTOR,
  'ufactory_xarm7/hand': ModelType.END_EFFECTOR,
  'ufactory_xarm7/xarm7': ModelType.ARM,
  'hello_robot_stretch/stretch': ModelType.MOBILE_MANIPULATOR,
  'google_barkour_vb/barkour_vb': ModelType.QUADRUPED,
  'unitree_go2/go2': ModelType.QUADRUPED,
  'boston_dynamics_spot/spot_arm': ModelType.QUADRUPED,
  'shadow_dexee/shadow_dexee': ModelType.END_EFFECTOR,
  'pal_talos/talos': ModelType.HUMANOID,
  'leap_hand/left_hand': ModelType.END_EFFECTOR,
  'kinova_gen3/gen3': ModelType.ARM,
  'booster_t1/t1': ModelType.HUMANOID,
  'agilex_piper/piper': ModelType.ARM,
  'toddlerbot_2xc/toddlerbot_2xc': ModelType.HUMANOID,
  'flexiv_rizon4/flexiv_rizon4': ModelType.ARM,
  'arx_l5/arx_l5': ModelType.ARM,
  'flexiv_rizon4s/flexiv_rizon4s': ModelType.ARM,
  'trossen_wx250s/wx250s': ModelType.ARM,
  'trs_so_arm100/so_arm100': ModelType.ARM,
  'low_cost_robot_arm/low_cost_robot_arm': ModelType.ARM,
  'i2rt_yam/yam': ModelType.ARM,
  'umi_gripper/umi_gripper': ModelType.END_EFFECTOR,
  'sharpa_wave/left_hand': ModelType.END_EFFECTOR,
  'stanford_tidybot/tidybot': ModelType.MOBILE_MANIPULATOR,
  'hello_robot_stretch_3/stretch': ModelType.MOBILE_MANIPULATOR,
  'pal_tiago/tiago': ModelType.MOBILE_MANIPULATOR,
  'pal_tiago_dual/tiago_dual': ModelType.MOBILE_MANIPULATOR,
  'robot_soccer_kit/robot_soccer_kit': ModelType.MOBILE_BASE,
  'pndbotics_adam_lite/adam_lite': ModelType.HUMANOID,
  'apptronik_apollo/apptronik_apollo': ModelType.HUMANOID,
  'berkeley_humanoid/berkeley_humanoid': ModelType.HUMANOID,
  'fourier_n1/n1': ModelType.HUMANOID,
  'toddlerbot_2xm/toddlerbot_2xm': ModelType.HUMANOID,
  'iit_softfoot/softfoot': ModelType.BIOMECHANICAL,
  'ms_human_700/MS-Human-700': ModelType.BIOMECHANICAL,
  'seeed_rebot_devarm/seeed_rebot_devarm': ModelType.ARM,
  'dynamixel_2r/dynamixel_2r': ModelType.ARM,
  'franka_fr3_v2/fr3v2': ModelType.ARM,
  'rainbow_robotics_rby1/rby1a_1.2': ModelType.MOBILE_MANIPULATOR,
  'robotiq_2f85_v4/2f85': ModelType.END_EFFECTOR,
  'robotstudio_so101/so101': ModelType.ARM,
  'tetheria_aero_hand_open/right_hand': ModelType.END_EFFECTOR,
  'trossen_wxai/wxai_follower': ModelType.ARM,
}


# Default preview target is `<maker>/scene.xml`. Override per robot when that
# file doesn't exist or wraps the wrong model (e.g., panda/scene.xml loads
# the full arm; for the standalone gripper we want hand.xml directly).
PREVIEW_OVERRIDES = {
  'franka_emika_panda/hand': 'franka_emika_panda/hand.xml',
  'ufactory_xarm7/hand': 'ufactory_xarm7/hand.xml',
  'leap_hand/left_hand': 'leap_hand/scene_left.xml',
  'shadow_hand/left_hand': 'shadow_hand/scene_left.xml',
  'wonik_allegro/left_hand': 'wonik_allegro/scene_left.xml',
  'sharpa_wave/left_hand': 'sharpa_wave/scene_left.xml',
  'realsense_d435i/d435i': 'realsense_d435i/d435i.xml',
  'pal_talos/talos': 'pal_talos/scene_position.xml',
  'pal_tiago/tiago': 'pal_tiago/scene_position.xml',
  'pal_tiago_dual/tiago_dual': 'pal_tiago_dual/scene_position.xml',
  'ms_human_700/MS-Human-700': 'ms_human_700/scene.xml',
  'rainbow_robotics_rby1/rby1a_1.2': 'rainbow_robotics_rby1/scene_rby1a_1.2.xml',
  'tetheria_aero_hand_open/right_hand': 'tetheria_aero_hand_open/scene_right.xml',
}


def preview_path(robot, robot_maker):
  return PREVIEW_OVERRIDES.get(robot, f'{robot_maker}/scene.xml')


def detect_license(license_path):
  """Identify the SPDX license name from the LICENSE file contents."""
  text = pathlib.Path(license_path).read_text(encoding='utf-8')
  lower = text.lower()
  if 'apache license' in lower and 'version 2' in lower:
    return 'Apache-2.0'
  if 'clear bsd' in lower:
    return 'BSD-3-Clause-Clear'
  if 'redistribution and use in source' in lower:
    return 'BSD-3-Clause' if 'neither the name' in lower else 'BSD-2-Clause'
  if 'permission is hereby granted, free of charge' in lower:
    return 'MIT'
  return 'Unknown'
