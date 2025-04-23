# Copyright 2024 DeepMind Technologies Limited
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
"""Generate a markdown table with images of some of the models in Menagerie.

Requirements:
    pip install absl-py dm_control pillow numpy tqdm mdutils opencv-python

Instructions:
    `python generate_gallery.py` will create a markdown document called
    `gallery.md` with a table of images. Copy this table into README.md to
    display the images.
"""
import enum
import math
import pathlib

from absl import app
import cv2
from dm_control import mjcf
from mdutils import mdutils
import numpy as np
from PIL import Image
import tqdm.auto


DEFAULT_FOV = 40


class ModelType(int, enum.Enum):
  ARM = 0
  DUAL_ARM = 1
  END_EFFECTOR = 2
  MOBILE_MANIPULATOR = 3
  QUADRUPED = 4
  BIPED = 5
  HUMANOID = 6
  DRONE = 7
  BIOMECHANICAL = 8
  MISC = 9


NAME_MAP = {
    "franka_emika_panda/panda": "panda",
    "franka_emika_panda/hand": "panda gripper",
    "franka_fr3/fr3": "franka fr3",
    "ufactory_lite6/lite6": "lite6",
    "flybody/fruitfly": "fruitfly",
    "skydio_x2/x2": "skydio x2",
    "unitree_h1/h1": "h1",
    "bitcraze_crazyflie_2/cf2": "crazyflie 2",
    "google_robot/robot": "google robot",
    "unitree_a1/a1": "a1",
    "google_barkour_v0/barkour_v0": "barkour v0",
    "anybotics_anymal_b/anymal_b": "anymal b",
    "unitree_go1/go1": "go1",
    "unitree_z1/z1": "z1",
    "anybotics_anymal_c/anymal_c": "anymal c",
    "agility_cassie/cassie": "cassie",
    "realsense_d435i/d435i": "d435i",
    "universal_robots_ur5e/ur5e": "ur5e",
    "aloha/aloha": "aloha 2",
    "rethink_robotics_sawyer/sawyer": "sawyer",
    "robotis_op3/op3": "op3",
    "universal_robots_ur10e/ur10e": "ur10e",
    "kuka_iiwa_14/iiwa14": "iiwa 14",
    "trossen_vx300s/vx300s": "vx300s",
    "unitree_g1/g1": "g1",
    "robotiq_2f85/2f85": "2f85",
    "ufactory_xarm7/hand": "xarm7 gripper",
    "ufactory_xarm7/xarm7": "xarm7",
    "hello_robot_stretch/stretch": "stretch 2",
    "google_barkour_vb/barkour_vb": "barkour vb",
    "unitree_go2/go2": "go2",
    "boston_dynamics_spot/spot_arm": "spot",
    "shadow_dexee/shadow_dexee": "dex-ee",
    "pal_talos/talos": "talos",
    "leap_hand/left_hand": "left leap",
    "wonik_allegro/left_hand": "left allegro",
    "shadow_hand/left_hand": "left shadow",
    "kinova_gen3/gen3": "gen3",
    "booster_t1/t1": "t1",
    "agilex_piper/piper": "piper",
}

MODEL_MAP = {
    "franka_emika_panda/panda": ModelType.ARM,
    "franka_emika_panda/hand": ModelType.END_EFFECTOR,
    "franka_fr3/fr3": ModelType.ARM,
    "ufactory_lite6/lite6": ModelType.ARM,
    "flybody/fruitfly": ModelType.BIOMECHANICAL,
    "wonik_allegro/left_hand": ModelType.END_EFFECTOR,
    "shadow_hand/left_hand": ModelType.END_EFFECTOR,
    "skydio_x2/x2": ModelType.DRONE,
    "unitree_h1/h1": ModelType.HUMANOID,
    "bitcraze_crazyflie_2/cf2": ModelType.DRONE,
    "google_robot/robot": ModelType.MOBILE_MANIPULATOR,
    "unitree_a1/a1": ModelType.QUADRUPED,
    "google_barkour_v0/barkour_v0": ModelType.QUADRUPED,
    "anybotics_anymal_b/anymal_b": ModelType.QUADRUPED,
    "unitree_go1/go1": ModelType.QUADRUPED,
    "unitree_z1/z1": ModelType.ARM,
    "anybotics_anymal_c/anymal_c": ModelType.QUADRUPED,
    "agility_cassie/cassie": ModelType.BIPED,
    "realsense_d435i/d435i": ModelType.MISC,
    "universal_robots_ur5e/ur5e": ModelType.ARM,
    "aloha/aloha": ModelType.DUAL_ARM,
    "rethink_robotics_sawyer/sawyer": ModelType.ARM,
    "robotis_op3/op3": ModelType.HUMANOID,
    "universal_robots_ur10e/ur10e": ModelType.ARM,
    "kuka_iiwa_14/iiwa14": ModelType.ARM,
    "trossen_vx300s/vx300s": ModelType.ARM,
    "unitree_g1/g1": ModelType.HUMANOID,
    "robotiq_2f85/2f85": ModelType.END_EFFECTOR,
    "ufactory_xarm7/hand": ModelType.END_EFFECTOR,
    "ufactory_xarm7/xarm7": ModelType.ARM,
    "hello_robot_stretch/stretch": ModelType.MOBILE_MANIPULATOR,
    "google_barkour_vb/barkour_vb": ModelType.QUADRUPED,
    "unitree_go2/go2": ModelType.QUADRUPED,
    "boston_dynamics_spot/spot_arm": ModelType.QUADRUPED,
    "shadow_dexee/shadow_dexee": ModelType.END_EFFECTOR,
    "pal_talos/talos": ModelType.HUMANOID,
    "leap_hand/left_hand": ModelType.END_EFFECTOR,
    "kinova_gen3/gen3": ModelType.ARM,
    "booster_t1/t1": ModelType.HUMANOID,
    "agilex_piper/piper": ModelType.ARM,
}

DEFAULT_FOV = 40

CAMERA_MAP = {
    "pal_talos/talos": dict(
        pos="2.312 0.005 1.144",
        xyaxes="-0.002 1.000 -0.000 -0.107 -0.000 0.994",
    ),
    "skydio_x2/x2": dict(
        pos="-0.580 -0.260 0.622",
        xyaxes="0.442 -0.897 -0.000 0.428 0.211 0.879",
        fovy=60,
    ),
    "flybody/fruitfly": dict(
        pos="0.430 -0.361 0.326", xyaxes="0.589 0.808 0.000 -0.486 0.354 0.799",
        fovy=50
    ),
    "wonik_allegro/left_hand": dict(
        pos="0.002 0.043 0.432", xyaxes="0.052 -0.999 0.000 0.998 0.052 0.017"
    ),
    "ufactory_xarm7/xarm7": dict(
        pos="0.852 -0.383 0.860",
        xyaxes="0.487 0.874 0.000 -0.354 0.197 0.914",
        fovy=DEFAULT_FOV,
    ),
    "ufactory_xarm7/hand": dict(
        pos="-0.282 0.013 0.118",
        xyaxes="-0.047 -0.999 0.000 0.160 -0.007 0.987",
        fovy=45,
    ),
    "shadow_hand/left_hand": dict(
        pos="0.172 0.005 0.615",
        xyaxes="-0.508 -0.861 -0.000 0.861 -0.508 0.017",
        fovy=45,
    ),
    "franka_emika_panda/panda": dict(
        pos="0.412 1.106 0.849",
        xyaxes="-0.994 0.108 0.000 -0.040 -0.369 0.928",
        fovy=DEFAULT_FOV,
    ),
    "franka_fr3/fr3": dict(
        pos="0.412 1.106 0.849",
        xyaxes="-0.994 0.108 0.000 -0.040 -0.369 0.928",
        fovy=DEFAULT_FOV,
    ),
    "franka_emika_panda/hand": dict(
        pos="0.340 0.008 0.059",
        xyaxes="-0.023 1.000 0.000 -0.084 -0.002 0.996",
        fovy=DEFAULT_FOV,
    ),
    "kuka_iiwa_14/iiwa14": dict(
        pos="0.212 1.138 0.977",
        xyaxes="-1.000 0.027 0.000 -0.012 -0.441 0.898",
        fovy=45,
    ),
    "ufactory_lite6/lite6": dict(
        pos="0.077 -0.778 0.528",
        xyaxes="1.000 -0.004 -0.000 0.001 0.274 0.962",
        fovy=45,
    ),
    "unitree_g1/g1": dict(
        pos="1.466 -0.082 1.271",
        xyaxes="0.072 0.997 -0.000 -0.377 0.027 0.926",
        fovy=45,
    ),
    "unitree_h1/h1": dict(
        pos="2.098 0.006 1.893",
        xyaxes="0.007 1.000 -0.000 -0.394 0.003 0.919",
        fovy=45,
    ),
    "robotis_op3/op3": dict(
        pos="0.673 -0.024 0.447", xyaxes="0.035 0.999 0.000 -0.252 0.009 0.968",
        fovy=45
    ),
    "universal_robots_ur5e/ur5e": dict(
        pos="0.603 1.012 0.595",
        xyaxes="-0.932 0.363 -0.000 -0.080 -0.206 0.975",
        fovy=DEFAULT_FOV,
    ),
    "universal_robots_ur10e/ur10e": dict(
        pos="1.286 -0.798 0.889",
        xyaxes="0.696 0.718 -0.000 -0.224 0.218 0.950",
        fovy=DEFAULT_FOV,
    ),
    "unitree_z1/z1": dict(
        pos="0.305 -0.400 0.552",
        xyaxes="0.755 0.656 0.000 -0.359 0.413 0.837",
        fovy=DEFAULT_FOV,
    ),
    "unitree_go2/go2": dict(
        pos="0.753 -0.427 0.433",
        xyaxes="0.518 0.856 0.000 -0.284 0.172 0.943",
        fovy=DEFAULT_FOV,
    ),
    "unitree_go1/go1": dict(
        pos="0.679 -0.553 0.530",
        xyaxes="0.638 0.770 -0.000 -0.328 0.272 0.905",
        fovy=DEFAULT_FOV,
    ),
    "unitree_a1/a1": dict(
        pos="0.654 -0.564 0.536",
        xyaxes="0.676 0.737 -0.000 -0.327 0.299 0.896",
        fovy=DEFAULT_FOV,
    ),
    "trossen_vx300s/vx300s": dict(
        pos="0.583 0.317 0.549",
        xyaxes="-0.531 0.847 0.000 -0.434 -0.272 0.859",
        fovy=DEFAULT_FOV,
    ),
    "robotiq_2f85/2f85": dict(
        pos="-0.009 -0.251 0.107",
        xyaxes="0.999 -0.033 -0.000 0.005 0.150 0.989",
        fovy=45,
    ),
    "rethink_robotics_sawyer/sawyer": dict(
        pos="1.014 -0.494 0.876",
        xyaxes="0.555 0.832 -0.000 -0.372 0.248 0.895",
        fovy=DEFAULT_FOV,
    ),
    "realsense_d435i/d435i": dict(
        pos="-0.000 -0.002 0.128",
        xyaxes="1.000 -0.000 0.000 0.000 1.000 0.017",
        fovy=DEFAULT_FOV,
    ),
    "hello_robot_stretch/stretch": dict(
        pos="1.464 -0.514 1.439",
        xyaxes="0.271 0.963 -0.000 -0.362 0.102 0.927",
        fovy=45,
    ),
    "google_robot/robot": dict(
        pos="1.753 -0.231 1.305",
        xyaxes="0.025 1.000 0.000 -0.306 0.008 0.952",
        fovy=50,
    ),
    "google_barkour_vb/barkour_vb": dict(
        pos="0.887 0.338 0.565",
        xyaxes="-0.388 0.922 0.000 -0.460 -0.194 0.867",
        fovy=DEFAULT_FOV,
    ),
    "google_barkour_v0/barkour_v0": dict(
        pos="0.733 0.320 0.558",
        xyaxes="-0.416 0.909 -0.000 -0.441 -0.202 0.875",
        fovy=DEFAULT_FOV,
    ),
    "bitcraze_crazyflie_2/cf2": dict(
        pos="0.037 -0.142 0.206",
        xyaxes="0.963 0.268 0.000 -0.167 0.599 0.783",
        fovy=DEFAULT_FOV,
    ),
    "anybotics_anymal_b/anymal_b": dict(
        pos="0.930 -1.239 1.221",
        xyaxes="0.809 0.587 0.000 -0.308 0.424 0.852",
        fovy=DEFAULT_FOV,
    ),
    "anybotics_anymal_c/anymal_c": dict(
        pos="1.547 -0.577 0.941",
        xyaxes="0.378 0.926 -0.000 -0.358 0.146 0.922",
        fovy=45,
    ),
    "aloha/aloha": dict(
        pos="0.484 1.158 0.836",
        xyaxes="-0.939 0.345 -0.000 -0.162 -0.441 0.883",
        fovy=DEFAULT_FOV,
    ),
    "agility_cassie/cassie": dict(
        pos="1.277 -1.122 1.053",
        xyaxes="0.655 0.756 0.000 -0.196 0.170 0.966",
        fovy=DEFAULT_FOV,
    ),
    "boston_dynamics_spot/spot_arm": dict(
        pos="1.326 -0.829 0.651",
        xyaxes="0.573 0.820 -0.000 -0.154 0.108 0.982",
        fovy=DEFAULT_FOV,
    ),
    "shadow_dexee/shadow_dexee": dict(
        pos="-0.003 -0.541 0.221",
        xyaxes="1.000 0.000 -0.000 -0.000 0.131 0.991",
        fovy=45,
    ),
    "leap_hand/left_hand": dict(
        pos="-0.123 0.096 0.512",
        xyaxes="0.004 -1.000 -0.000 0.995 0.004 0.101",
    ),
    "kinova_gen3/gen3": dict(
        pos="0.252 -1.047 0.521",
        xyaxes="0.988 0.156 -0.000 -0.024 0.153 0.988",
    ),
    "booster_t1/t1": dict(
        pos="1.499 -0.777 1.2",
        xyaxes="0.453 0.892 0.000 -0.295 0.150 0.944",
        fovy=DEFAULT_FOV,
    ),
    "agilex_piper/piper": dict(
        pos="0.288 -0.480 0.294",
        xyaxes="0.866 0.500 0.000 -0.171 0.296 0.940",
        fovy=50,
    ),
}

# pylint: disable=line-too-long
KEYFRAME_MAP = {
    "pal_talos": "0 0 1.025 0 0 0 0 0 0.15 0 0 0.3 0.4 -0.5 -1.5 0 0 0 0 -0.4 0 0 0 0 0 -0.3 -0.4 0.5 -1.5 0 0 0 0 -0.4 0 0 0 0 0 0 0 -0.4 0.8 -0.4 0 0 0 -0.4 0.8 -0.4 0",
    "robotis_op3": "0 0 0.2789 1 0 0 0 0.0 0.0 -0.0890 0.7931 -0.79 0.0874 -0.7946 0.7855 -0.0015 -0.0460 -0.1626 0.2316 0.1565 -0.0230 0.0 0.0445 0.1611 -0.2332 -0.1580 0.0215",
    "google_barkour_vb": "0 0 0.21 1 0 0 0 0 0.5 1.0 0 0.5 1.0 0 0.5 1.0 0 0.5 1.0",
    "hello_robot_stretch": "0 0 0 1 0 0 0 0 0 0.1325 0.07995 0.07995 0.07605 0.0702 1.585 0 0.198 0 0 0.126 0 0 0 0",
    "google_robot": "-1.51699e-13 -1.16232e-12 -0.1444 2.9724 -0.146 -0.3759 1.15806e-12 0.5518 0.62275",
    "aloha": "0.43988 -0.206468 1.08253 -0.443382 -1.084 -0.00397598 0.0084 0.00846495 -1.28822 -0.360594 0.717978 -0.000325086 -0.273415 6.76003e-05 0.0084 0.00839987",
    "kuka_iiwa_14": "0 0 0 -1.5708 0 1.5708 0",
}
# pylint: enable=line-too-long

KEEP_LIGHT = ["go1", "a1", "op3", "aloha", "left_hand", "stretch", "piper"]


def create_arena():
  arena = mjcf.RootElement()
  arena.visual.quality.shadowsize = 8192
  arena.visual.headlight.diffuse = (0.6,) * 3
  arena.visual.headlight.ambient = (0.3,) * 3
  arena.visual.headlight.specular = (0.2,) * 3
  getattr(arena.visual, "global").offheight = 720
  getattr(arena.visual, "global").offwidth = 1280
  arena.asset.add(
      "texture",
      type="skybox",
      builtin="gradient",
      height=512,
      width=512,
      rgb1="1 1 1",
      rgb2="1 1 1",
  )
  return arena


MODEL_XMLS = [pathlib.Path(f"../{k}.xml") for k in MODEL_MAP.keys()]


# Sort XML files.
def sort_func(xml):
  name = f"{xml.parent.stem}/{xml.stem}"
  return (MODEL_MAP[name], xml.stem)


MODEL_XMLS = sorted(MODEL_XMLS, key=sort_func)


def main(argv):
  del argv

  paths = []
  pngs = []
  for xml in tqdm.auto.tqdm(MODEL_XMLS):
    try:
      robot_maker = xml.parent.stem
      robot_name = xml.stem
      robot = f"{robot_maker}/{robot_name}"

      if robot not in CAMERA_MAP:
        continue

      arena = create_arena()

      if robot_maker in KEYFRAME_MAP:
        arena.keyframe.add("key", qpos=KEYFRAME_MAP[robot_maker])

      model_xml = mjcf.from_path(xml.as_posix(), escape_separators=True)
      for light in model_xml.find_all("light"):
        if robot_name not in KEEP_LIGHT:
          light.remove()

      if robot in CAMERA_MAP:
        camera_kwargs = CAMERA_MAP[robot]
        arena.worldbody.add("camera", name="thumbnail", **camera_kwargs)

      if robot_maker == "aloha":
        right_base = model_xml.find("body", "right\\base_link")
        right_base.pos[0] = 0.3
        left_base = model_xml.find("body", "left\\base_link")
        left_base.pos[0] = -0.3

      arena.include_copy(model_xml, override_attributes=True)

      physics = mjcf.Physics.from_mjcf_model(arena)

      try:
        physics.reset(keyframe_id=0)
      except:
        physics.reset()

      physics.forward()

      if robot in CAMERA_MAP:
        img = physics.render(height=500, width=500, camera_id="thumbnail")
      else:
        img = physics.render(height=500, width=500)

      img = cv2.putText(
          img.copy(),
          NAME_MAP[robot],
          (5, 480),
          cv2.FONT_HERSHEY_SIMPLEX,
          1.3,
          (0, 0, 0),
          1,
          cv2.LINE_AA,
      )

      filename = f"assets/{robot_maker}-{robot_name}.png"
      paths.append(filename)

      png = np.zeros((500, 500, 4), dtype=np.uint8)
      u, v = np.where(np.all(img == 255, axis=-1))
      png[u, v, -1] = 0
      png[u, v, :3] = 0
      u, v = np.where(np.any(img != 255, axis=-1))
      png[u, v, :3] = img[u, v]
      png[u, v, -1] = 255
      pngs.append(png.copy())
      Image.fromarray(png).save(filename)
    except Exception as e:
      print(e)
      print(f"failed to load {xml.as_posix()}")

  n_models = len(paths)
  n_cols = 5
  n_rows = int(math.ceil(n_models / n_cols))
  table = []
  for r in range(n_rows):
    row = []
    for c in range(n_cols):
      i = r * n_cols + c
      if i >= n_models:
        row.append("")
      else:
        row.append(f"<img src='{paths[i]}' width=100>")
    table.extend(row)

  mdfile = mdutils.MdUtils(file_name="gallery")
  mdfile.new_table(columns=n_cols, rows=n_rows, text=table, text_align="center")
  mdfile.create_md_file()


if __name__ == "__main__":
  app.run(main)
