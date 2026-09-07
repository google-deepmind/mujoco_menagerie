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
# dependencies = ["absl-py", "mujoco", "pillow", "numpy", "tqdm"]
# ///
"""Render every Menagerie model's thumbnail and update the README gallery.

Requirements:
    pip install absl-py mujoco pillow numpy tqdm

Instructions:
    `python generate_gallery.py` (or `make gallery`) regenerates the
    thumbnails in `assets/` and splices a categorized table between the
    `<!-- BEGIN MODELS -->` / `<!-- END MODELS -->` markers in README.md.
"""

import math
import pathlib
import re

import mujoco
import numpy as np
import tqdm.auto
from absl import app
from PIL import Image

from catalog import MODEL_MAP
from catalog import ModelType
from catalog import detect_license
from catalog import display_name
from catalog import preview_path

DEFAULT_FOV = 40


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

# Each thumbnail in gallery.md links to a live preview of the model XML on
# live.mujoco.org. The repo's PR-preview workflow uses
# `github:OWNER/REPO/pull/N/head/PATH`; for a branch the parser expects the
# bare ref: `github:OWNER/REPO/REF/PATH` (same shape as raw.githubusercontent
# paths, just without the host).
LIVE_REPO = 'google-deepmind/mujoco_menagerie'
LIVE_REF = 'main'


def live_url(xml_path):
  return (
    f'https://live.mujoco.org/?model='
    f'github:{LIVE_REPO}/{LIVE_REF}/{xml_path}'
  )


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
# Per-robot view angle override, falling back to VIEW_ANGLES[category].
VIEW_ANGLE_OVERRIDE = {
  # Default biomechanical angle catches MS-Human-700 from the side; nudge
  # to a near-frontal 3/4.
  'ms_human_700/MS-Human-700': (20, 15),
}


VIEW_ANGLES = {
  ModelType.ARM: (70, 25),
  ModelType.DUAL_ARM: (70, 25),
  # End-effectors look bad from the side — fingers extend ~horizontally so
  # a high elevation looks down at the spread of the digits.
  ModelType.END_EFFECTOR: (45, 55),
  ModelType.MOBILE_MANIPULATOR: (15, 25),
  ModelType.MOBILE_BASE: (-30, 35),
  ModelType.QUADRUPED: (-30, 25),
  ModelType.BIPED: (-30, 25),
  ModelType.HUMANOID: (15, 25),
  ModelType.DRONE: (110, 30),
  ModelType.BIOMECHANICAL: (110, 25),
  ModelType.MISC: (80, 25),
}


_CORNER_SIGNS = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(
  -1, 3
)


def posed_bounds(model, data):
  """World-frame AABB of visible geoms in the current forward-evaluated pose."""
  visible = np.where(model.geom_group != 3)[0]
  aabb = model.geom_aabb[
    visible
  ]  # (n, 6): center_xyz + halfsize_xyz, in local frame
  c_local = aabb[:, :3]
  h_local = aabb[:, 3:]
  corners_local = (
    c_local[:, None, :] + h_local[:, None, :] * _CORNER_SIGNS
  )  # (n, 8, 3)
  rot = data.geom_xmat[visible].reshape(-1, 3, 3)
  trans = data.geom_xpos[visible]
  corners_world = (
    np.einsum('nij,nkj->nki', rot, corners_local) + trans[:, None, :]
  )
  pts = corners_world.reshape(-1, 3)
  return pts.min(axis=0), pts.max(axis=0)


def auto_camera(lo, hi, model_type, robot=None):
  """Frame the model's AABB tightly from a per-type viewing direction."""
  azimuth_deg, elevation_deg = VIEW_ANGLE_OVERRIDE.get(
    robot, VIEW_ANGLES[model_type]
  )
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


# Section heading for each ModelType in the README. Iteration order
# determines display order.
SECTION_LABEL = {
  ModelType.HUMANOID: 'Humanoids',
  ModelType.QUADRUPED: 'Quadrupeds',
  ModelType.BIPED: 'Bipeds',
  ModelType.BIOMECHANICAL: 'Biomechanical',
  ModelType.DUAL_ARM: 'Dual Arms',
  ModelType.MOBILE_MANIPULATOR: 'Mobile Manipulators',
  ModelType.DRONE: 'Drones',
  ModelType.ARM: 'Arms',
  ModelType.END_EFFECTOR: 'End-effectors',
  ModelType.MOBILE_BASE: 'Mobile Bases',
  ModelType.MISC: 'Miscellaneous',
}

THUMB_WIDTH = 120

MODELS_BEGIN = (
  '<!-- BEGIN MODELS (auto-generated by `make gallery` — do not edit) -->'
)
MODELS_END = '<!-- END MODELS -->'


def _row(robot, png_path, xml_path, nu):
  maker = robot.split('/')[0]
  name = display_name(robot)
  license_path = f'{maker}/LICENSE'
  license_name = (
    detect_license(license_path)
    if pathlib.Path(license_path).exists()
    else 'Unknown'
  )
  if png_path is not None:
    preview = (
      f"<a href='{live_url(xml_path)}' title='Open live preview for {name}'>"
      f"<img src='{png_path}' width={THUMB_WIDTH}></a>"
    )
  else:
    preview = ''
  dof = str(nu) if nu is not None else '—'
  return f'| {preview} | {name} | {dof} | [{license_name}]({license_path}) |'


def write_gallery_to_readme(rendered, dofs, readme_path='README.md'):
  """Replace the Menagerie Models section between markers with auto-gen tables.

  rendered: list of (robot, png_path, xml_path) for successfully rendered models.
  dofs: dict[robot -> nu] populated for all models that compiled.
  """
  rendered_by_robot = {robot: (png, xml) for robot, png, xml in rendered}

  sections = []
  for cat, label in SECTION_LABEL.items():
    rows = []
    for robot, robot_cat in MODEL_MAP.items():
      if robot_cat != cat:
        continue
      png, xml = rendered_by_robot.get(robot, (None, None))
      rows.append(_row(robot, png, xml, dofs.get(robot)))
    if not rows:
      continue
    table = (
      f'**{label}.**\n\n'
      '| Preview | Name | DoFs | License |\n'
      '|:---:|---|---|---|\n' + '\n'.join(rows)
    )
    sections.append(table)

  body = '\n\n'.join(sections)
  readme = pathlib.Path(readme_path).read_text(encoding='utf-8')
  pattern = re.compile(
    re.escape(MODELS_BEGIN) + r'.*?' + re.escape(MODELS_END), re.DOTALL
  )
  if not pattern.search(readme):
    raise SystemExit(
      f'models markers not found in {readme_path}; expected\n  {MODELS_BEGIN}\n  {MODELS_END}'
    )
  new = pattern.sub(f'{MODELS_BEGIN}\n\n{body}\n\n{MODELS_END}', readme)
  pathlib.Path(readme_path).write_text(new, encoding='utf-8')


def main(argv):
  del argv

  paths = []
  pngs = []
  dofs = {}
  for xml in tqdm.auto.tqdm(MODEL_XMLS):
    robot_maker = xml.parent.stem
    robot_name = xml.stem
    robot = f'{robot_maker}/{robot_name}'

    try:
      # Load with an absolute path so each XML's own directory is used to
      # resolve nested includes and per-model meshdir. chdir-ing into the
      # model dir caused mesh-cache collisions across specs that share asset
      # filenames (e.g., UR5e + UR10e both ship `assets/base_0.obj`).
      spec = mujoco.MjSpec.from_file(str(xml.resolve()))
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
        camera_kwargs = auto_camera(lo, hi, MODEL_MAP[robot], robot)
      spec.worldbody.add_camera(name='thumbnail', **camera_kwargs)

      model = spec.compile()
      n_freejoints = int((model.jnt_type == mujoco.mjtJoint.mjJNT_FREE).sum())
      dofs[robot] = int(model.nq) - 7 * n_freejoints
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
      # Build the alpha mask from a segmentation render so background
      # pixels become transparent without nuking any geom (chroma-keying
      # against the white skybox would eat white robot parts like UR5e's
      # aluminum links).
      renderer.enable_segmentation_rendering()
      renderer.update_scene(data, camera='thumbnail')
      mask = renderer.render()[..., 0] != -1
      renderer.disable_segmentation_rendering()

      filename = f'assets/{robot_maker}-{robot_name}.png'
      paths.append((robot, filename, preview_path(robot, robot_maker)))

      png = np.zeros((500, 500, 4), dtype=np.uint8)
      png[mask, :3] = img[mask]
      png[mask, 3] = 255
      pngs.append(png.copy())
      Image.fromarray(png).save(filename)
    except Exception as e:
      print(e)
      print(f'failed to load {xml.as_posix()}')

  write_gallery_to_readme(paths, dofs)


if __name__ == '__main__':
  app.run(main)
