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

# /// script
# dependencies = ["absl-py", "mujoco", "pillow", "numpy", "tqdm", "mdutils", "opencv-python"]
# ///
"""Generate a markdown table with images of some of the models in Menagerie.

Requirements:
    pip install absl-py mujoco pillow numpy tqdm mdutils opencv-python

Instructions:
    `python generate_gallery.py` will create a markdown document called
    `gallery.md` with a table of images. Copy this table into README.md to
    display the images.
"""

import enum
import math
import pathlib

import cv2
import mujoco
import numpy as np
import tqdm.auto
from absl import app
from mdutils import mdutils
from PIL import Image

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
  'franka_emika_panda/panda': 'panda',
  'franka_emika_panda/hand': 'panda gripper',
  'franka_fr3/fr3': 'franka fr3',
  'ufactory_lite6/lite6': 'lite6',
  'flybody/fruitfly': 'fruitfly',
  'skydio_x2/x2': 'skydio x2',
  'unitree_h1/h1': 'h1',
  'bitcraze_crazyflie_2/cf2': 'crazyflie 2',
  'google_robot/robot': 'google robot',
  'unitree_a1/a1': 'a1',
  'google_barkour_v0/barkour_v0': 'barkour v0',
  'anybotics_anymal_b/anymal_b': 'anymal b',
  'unitree_go1/go1': 'go1',
  'unitree_z1/z1': 'z1',
  'anybotics_anymal_c/anymal_c': 'anymal c',
  'agility_cassie/cassie': 'cassie',
  'realsense_d435i/d435i': 'd435i',
  'universal_robots_ur5e/ur5e': 'ur5e',
  'aloha/aloha': 'aloha 2',
  'rethink_robotics_sawyer/sawyer': 'sawyer',
  'robotis_op3/op3': 'op3',
  'universal_robots_ur10e/ur10e': 'ur10e',
  'kuka_iiwa_14/iiwa14': 'iiwa 14',
  'trossen_vx300s/vx300s': 'vx300s',
  'unitree_g1/g1': 'g1',
  'robotiq_2f85/2f85': '2f85',
  'ufactory_xarm7/hand': 'xarm7 gripper',
  'ufactory_xarm7/xarm7': 'xarm7',
  'hello_robot_stretch/stretch': 'stretch 2',
  'google_barkour_vb/barkour_vb': 'barkour vb',
  'unitree_go2/go2': 'go2',
  'boston_dynamics_spot/spot_arm': 'spot',
  'shadow_dexee/shadow_dexee': 'dex-ee',
  'pal_talos/talos': 'talos',
  'leap_hand/left_hand': 'left leap',
  'wonik_allegro/left_hand': 'left allegro',
  'shadow_hand/left_hand': 'left shadow',
  'kinova_gen3/gen3': 'gen3',
  'booster_t1/t1': 't1',
  'agilex_piper/piper': 'piper',
  'toddlerbot_2xc/toddlerbot_2xc': 'toddlerbot',
  'flexiv_rizon4/flexiv_rizon4': 'rizon4',
}

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
}

# Per-model camera overrides. Populated only when auto-camera produces a
# bad thumbnail; the dict can stay empty otherwise. Example entry:
#   'pal_talos/talos': dict(
#       pos='2.312 0.005 1.144',
#       xyaxes='-0.002 1.000 -0.000 -0.107 -0.000 0.994',
#       fovy=45,
#   ),
CAMERA_MAP = {}

# pylint: disable=line-too-long
KEYFRAME_MAP = {
  'pal_talos/talos': (
    '0 0 1.025 0 0 0 0 0 0.15 0 0 0.3 0.4 -0.5 -1.5 0 0 0 0 -0.4 0 0 0 0 0'
    ' -0.3 -0.4 0.5 -1.5 0 0 0 0 -0.4 0 0 0 0 0 0 0 -0.4 0.8 -0.4 0 0 0'
    ' -0.4 0.8 -0.4 0'
  ),
  'robotis_op3/op3': (
    '0 0 0.2789 1 0 0 0 0.0 0.0 -0.0890 0.7931 -0.79 0.0874 -0.7946 0.7855'
    ' -0.0015 -0.0460 -0.1626 0.2316 0.1565 -0.0230 0.0 0.0445 0.1611'
    ' -0.2332 -0.1580 0.0215'
  ),
  'google_barkour_vb/barkour_vb': (
    '0 0 0.21 1 0 0 0 0 0.5 1.0 0 0.5 1.0 0 0.5 1.0 0 0.5 1.0'
  ),
  'hello_robot_stretch/stretch': (
    '0 0 0 1 0 0 0 0 0 0.1325 0.07995 0.07995 0.07605 0.0702 1.585 0 0.198'
    ' 0 0 0.126 0 0 0 0'
  ),
  'google_robot/robot': (
    '-1.51699e-13 -1.16232e-12 -0.1444 2.9724 -0.146 -0.3759 1.15806e-12'
    ' 0.5518 0.62275'
  ),
  'aloha/aloha': (
    '0.43988 -0.206468 1.08253 -0.443382 -1.084 -0.00397598 0.0084'
    ' 0.00846495 -1.28822 -0.360594 0.717978 -0.000325086 -0.273415'
    ' 6.76003e-05 0.0084 0.00839987'
  ),
  'kuka_iiwa_14/iiwa14': '0 0 0 -1.5708 0 1.5708 0',
  'flexiv_rizon4/flexiv_rizon4': '0 -0.524 0 1.833 0 0.785 0',
  'franka_emika_panda/hand': '0.04 0.04',
}
# pylint: enable=line-too-long

KEEP_LIGHT = ['go1', 'a1', 'op3', 'aloha', 'left_hand', 'stretch', 'piper']


def _parse_floats(s):
  return [float(t) for t in s.split()]


AUTO_FOVY = 45
# Padding around the projected model AABB. 1.0 = model touches the frame
# edge; >1 leaves margin around the model.
AUTO_PADDING = 1.08

# (azimuth_deg, elevation_deg). Azimuth is measured from +X around +Z.
# Arms and end-effectors in Menagerie are typically mounted facing +Y, so we
# view from ~70° (front-right). Legged robots default to facing +X (identity
# quat), so we view them from ~20–-30° (front-right of +X).
VIEW_ANGLES = {
  ModelType.ARM: (70, 25),
  ModelType.DUAL_ARM: (70, 25),
  # End-effectors look bad from the side — fingers extend ~horizontally so
  # a high elevation looks down at the spread of the digits.
  ModelType.END_EFFECTOR: (45, 55),
  ModelType.MOBILE_MANIPULATOR: (15, 25),
  ModelType.QUADRUPED: (-30, 25),
  ModelType.BIPED: (-30, 25),
  ModelType.HUMANOID: (15, 25),
  ModelType.DRONE: (110, 30),
  ModelType.BIOMECHANICAL: (110, 25),
  ModelType.MISC: (80, 25),
}


_CORNER_SIGNS = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)


def posed_bounds(model, data):
  """World-frame AABB of visible geoms in the current forward-evaluated pose."""
  visible = np.where(model.geom_group != 3)[0]
  aabb = model.geom_aabb[visible]  # (n, 6): center_xyz + halfsize_xyz, in local frame
  c_local = aabb[:, :3]
  h_local = aabb[:, 3:]
  corners_local = c_local[:, None, :] + h_local[:, None, :] * _CORNER_SIGNS  # (n, 8, 3)
  rot = data.geom_xmat[visible].reshape(-1, 3, 3)
  trans = data.geom_xpos[visible]
  corners_world = np.einsum('nij,nkj->nki', rot, corners_local) + trans[:, None, :]
  pts = corners_world.reshape(-1, 3)
  return pts.min(axis=0), pts.max(axis=0)


def auto_camera(lo, hi, model_type):
  """Frame the model's AABB tightly from a per-type viewing direction."""
  azimuth_deg, elevation_deg = VIEW_ANGLES[model_type]
  az = math.radians(azimuth_deg)
  el = math.radians(elevation_deg)
  z_cam = np.array(
    [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
  )
  x_cam = np.cross([0.0, 0.0, 1.0], z_cam)
  x_cam /= np.linalg.norm(x_cam)
  y_cam = np.cross(z_cam, x_cam)
  # Pick the smallest distance along Z_cam such that all 8 AABB corners fall
  # inside the perspective frustum (corners closer to the camera need more
  # margin, since they project larger).
  center = (lo + hi) / 2
  corners = np.stack(np.meshgrid(*zip(lo, hi))).reshape(3, -1).T - center
  half_fov = math.radians(AUTO_FOVY / 2)
  depth = corners @ z_cam
  dist_x = (depth + np.abs(corners @ x_cam) / math.tan(half_fov)).max()
  dist_y = (depth + np.abs(corners @ y_cam) / math.tan(half_fov)).max()
  dist = max(dist_x, dist_y) * AUTO_PADDING
  pos = center + z_cam * dist
  return dict(
    pos=pos.tolist(),
    xyaxes=x_cam.tolist() + y_cam.tolist(),
    fovy=AUTO_FOVY,
  )


def apply_gallery_settings(spec):
  """Apply the gallery's visual settings and white skybox to a model spec."""
  spec.visual.quality.shadowsize = 8192
  spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
  spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
  spec.visual.headlight.specular = [0.2, 0.2, 0.2]
  spec.visual.global_.offheight = 720
  spec.visual.global_.offwidth = 1280
  spec.add_texture(
    name='gallery_skybox',
    type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
    builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
    height=512,
    width=512,
    rgb1=[1, 1, 1],
    rgb2=[1, 1, 1],
  )


MODEL_XMLS = [pathlib.Path(f'./{k}.xml') for k in MODEL_MAP.keys()]


# Sort XML files.
def sort_func(xml):
  name = f'{xml.parent.stem}/{xml.stem}'
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
      robot = f'{robot_maker}/{robot_name}'

      spec = mujoco.MjSpec.from_file(xml.as_posix())
      apply_gallery_settings(spec)

      if robot_name not in KEEP_LIGHT:
        for light in list(spec.lights):
          spec.delete(light)

      gallery_key_name = None
      if robot in KEYFRAME_MAP:
        gallery_key_name = 'gallery_thumbnail'
        spec.add_key(
          name=gallery_key_name,
          qpos=_parse_floats(KEYFRAME_MAP[robot]),
        )

      if robot_maker == 'aloha':
        spec.body('right/base_link').pos[0] = 0.3
        spec.body('left/base_link').pos[0] = -0.3

      if robot in CAMERA_MAP:
        camera_kwargs = dict(CAMERA_MAP[robot])
        camera_kwargs['pos'] = _parse_floats(camera_kwargs['pos'])
        camera_kwargs['xyaxes'] = _parse_floats(camera_kwargs['xyaxes'])
      else:
        # Compile once to get the posed geometry, then place the camera.
        probe_model = spec.compile()
        probe_data = mujoco.MjData(probe_model)
        if gallery_key_name is not None:
          probe_key = mujoco.mj_name2id(
            probe_model, mujoco.mjtObj.mjOBJ_KEY, gallery_key_name
          )
          mujoco.mj_resetDataKeyframe(probe_model, probe_data, probe_key)
        elif probe_model.nkey > 0:
          mujoco.mj_resetDataKeyframe(probe_model, probe_data, 0)
        else:
          mujoco.mj_resetData(probe_model, probe_data)
        mujoco.mj_forward(probe_model, probe_data)
        lo, hi = posed_bounds(probe_model, probe_data)
        camera_kwargs = auto_camera(lo, hi, MODEL_MAP[robot])
      spec.worldbody.add_camera(name='thumbnail', **camera_kwargs)

      model = spec.compile()
      data = mujoco.MjData(model)
      if gallery_key_name is not None:
        key_id = mujoco.mj_name2id(
          model, mujoco.mjtObj.mjOBJ_KEY, gallery_key_name
        )
        mujoco.mj_resetDataKeyframe(model, data, key_id)
      elif model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
      else:
        mujoco.mj_resetData(model, data)
      mujoco.mj_forward(model, data)

      renderer = mujoco.Renderer(model, height=500, width=500)
      renderer.update_scene(data, camera='thumbnail')
      img = renderer.render()

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

      filename = f'assets/{robot_maker}-{robot_name}.png'
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
      print(f'failed to load {xml.as_posix()}')

  n_models = len(paths)
  n_cols = 5
  n_rows = int(math.ceil(n_models / n_cols))
  table = []
  for r in range(n_rows):
    row = []
    for c in range(n_cols):
      i = r * n_cols + c
      if i >= n_models:
        row.append('')
      else:
        row.append(f"<img src='{paths[i]}' width=100>")
    table.extend(row)

  mdfile = mdutils.MdUtils(file_name='gallery')
  mdfile.new_table(columns=n_cols, rows=n_rows, text=table, text_align='center')
  mdfile.create_md_file()


if __name__ == '__main__':
  app.run(main)
