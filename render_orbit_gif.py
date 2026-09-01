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

# /// script
# dependencies = ["absl-py", "mujoco", "pillow", "numpy"]
# ///
"""Render a looping 360-degree orbit GIF of a model.

The camera orbits the model at a fixed elevation and a fixed distance chosen
so that the model stays fully inside the frame for every azimuth, which keeps
the loop free of zoom jitter. The model is held in its initial pose (the first
keyframe if it has one), so this shows geometry, not a simulation.

Example:
    uv run render_orbit_gif.py ostrich/ostrich.xml \
        --output ostrich/ostrich.gif --width 560 --height 760 \
        --start_azimuth 135
"""

import math
import pathlib

import mujoco
import numpy as np
from absl import app
from absl import flags
from PIL import Image

_OUTPUT = flags.DEFINE_string(
  'output', None, 'Output GIF path. Defaults to the model path with .gif.'
)
_FRAMES = flags.DEFINE_integer('frames', 48, 'Number of frames in the loop.')
_FPS = flags.DEFINE_float('fps', 20.0, 'Playback rate, in frames per second.')
_WIDTH = flags.DEFINE_integer('width', 600, 'Output width, in pixels.')
_HEIGHT = flags.DEFINE_integer('height', 600, 'Output height, in pixels.')
_SUPERSAMPLE = flags.DEFINE_integer(
  'supersample', 2, 'Render at this multiple of the output size, then downscale.'
)
_ELEVATION = flags.DEFINE_float(
  'elevation', 12.0, 'Camera elevation above the horizon, in degrees.'
)
_START_AZIMUTH = flags.DEFINE_float(
  'start_azimuth',
  90.0,
  'Azimuth of the first frame, in degrees. Follows the MuJoCo free-camera '
  'convention: the direction the camera looks along, so 0 views a +X-facing '
  'model from behind and 180 views it head-on.',
)
_FOVY = flags.DEFINE_float('fovy', 45.0, 'Vertical field of view, in degrees.')
_PADDING = flags.DEFINE_float(
  'padding', 1.06, 'Framing margin. 1.0 makes the model touch the frame edge.'
)
_COLORS = flags.DEFINE_integer(
  'colors', 128, 'Palette size. Fewer colors means a smaller file.'
)
_KEEP_FLOOR = flags.DEFINE_bool(
  'keep_floor', False, 'Keep plane geoms (they otherwise fill the frame).'
)
_BACKGROUND = flags.DEFINE_string(
  'background', 'white', 'Background: "white" or "model" (the model\'s skybox).'
)

_CORNER_SIGNS = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)


def prepare_spec(path):
  """Load the model and strip anything that would spoil an orbit render."""
  spec = mujoco.MjSpec.from_file(str(path.resolve()))

  spec.visual.quality.shadowsize = 8192
  spec.visual.quality.offsamples = 8
  spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
  spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
  spec.visual.headlight.specular = [0.2, 0.2, 0.2]

  # Fixed lights cast a shadow that sweeps across the model as the camera
  # orbits; the headlight travels with the camera instead.
  for light in list(spec.lights):
    spec.delete(light)

  if not _KEEP_FLOOR.value:
    for geom in list(spec.geoms):
      if geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
        spec.delete(geom)

  if _BACKGROUND.value == 'white':
    for texture in list(spec.textures):
      if texture.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
        spec.delete(texture)
    spec.add_texture(
      name='orbit_skybox',
      type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
      builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
      height=512,
      width=512,
      rgb1=[1, 1, 1],
      rgb2=[1, 1, 1],
    )

  return spec


def visible_corners(model, data):
  """World-frame AABB corners of the visible geoms, in the current pose."""
  visible = np.where(model.geom_group != 3)[0]
  aabb = model.geom_aabb[visible]  # (n, 6): local center + half-size.
  corners_local = aabb[:, None, :3] + aabb[:, None, 3:] * _CORNER_SIGNS
  rot = data.geom_xmat[visible].reshape(-1, 3, 3)
  corners = np.einsum('nij,nkj->nki', rot, corners_local)
  corners += data.geom_xpos[visible][:, None, :]
  return corners.reshape(-1, 3)


def orbit_distance(points, center, elevation_deg, azimuths_deg, fovy, aspect):
  """Smallest distance that frames `points` at every azimuth of the orbit."""
  half_fovy = math.radians(fovy / 2)
  half_fovx = math.atan(math.tan(half_fovy) * aspect)
  el = math.radians(elevation_deg)
  rel = points - center

  distance = 0.0
  for azimuth_deg in azimuths_deg:
    az = math.radians(azimuth_deg)
    # MuJoCo's camera azimuth is the direction the camera looks along, so the
    # camera sits on the opposite side of the model.
    z_cam = np.array(
      [-math.cos(el) * math.cos(az), -math.cos(el) * math.sin(az), math.sin(el)]
    )
    x_cam = np.cross([0.0, 0.0, 1.0], z_cam)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    # Points nearer the camera project larger, so the depth term matters.
    depth = rel @ z_cam
    dist_x = (depth + np.abs(rel @ x_cam) / math.tan(half_fovx)).max()
    dist_y = (depth + np.abs(rel @ y_cam) / math.tan(half_fovy)).max()
    distance = max(distance, dist_x, dist_y)
  return distance * _PADDING.value


def quantize(frames, colors):
  """Quantize every frame against one shared palette, for a stable loop."""
  # The widest frame of the orbit is the most representative sample.
  sample = max(frames, key=lambda im: np.count_nonzero(np.array(im) < 250))
  palette = sample.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)
  return [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in frames]


def main(argv):
  if len(argv) != 2:
    raise app.UsageError('Expected exactly one model XML path.')
  model_path = pathlib.Path(argv[1])
  output = pathlib.Path(_OUTPUT.value or model_path.with_suffix('.gif'))

  width = _WIDTH.value * _SUPERSAMPLE.value
  height = _HEIGHT.value * _SUPERSAMPLE.value

  spec = prepare_spec(model_path)
  spec.visual.global_.fovy = _FOVY.value
  spec.visual.global_.offwidth = width
  spec.visual.global_.offheight = height

  model = spec.compile()
  data = mujoco.MjData(model)
  if model.nkey > 0:
    mujoco.mj_resetDataKeyframe(model, data, 0)
  else:
    mujoco.mj_resetData(model, data)
  mujoco.mj_forward(model, data)

  points = visible_corners(model, data)
  center = (points.min(axis=0) + points.max(axis=0)) / 2
  azimuths = _START_AZIMUTH.value + np.linspace(
    0, 360, _FRAMES.value, endpoint=False
  )
  distance = orbit_distance(
    points, center, _ELEVATION.value, azimuths, _FOVY.value, width / height
  )

  camera = mujoco.MjvCamera()
  camera.type = mujoco.mjtCamera.mjCAMERA_FREE
  camera.lookat = center
  camera.distance = distance
  camera.elevation = -_ELEVATION.value

  frames = []
  with mujoco.Renderer(model, height=height, width=width) as renderer:
    for azimuth in azimuths:
      camera.azimuth = azimuth
      renderer.update_scene(data, camera=camera)
      frame = Image.fromarray(renderer.render())
      if _SUPERSAMPLE.value > 1:
        frame = frame.resize(
          (_WIDTH.value, _HEIGHT.value), Image.Resampling.LANCZOS
        )
      frames.append(frame)

  frames = quantize(frames, _COLORS.value)
  frames[0].save(
    output,
    save_all=True,
    append_images=frames[1:],
    duration=round(1000 / _FPS.value),
    loop=0,
    optimize=True,
    disposal=2,
  )
  size_mb = output.stat().st_size / 1e6
  print(f'wrote {output} ({len(frames)} frames, {size_mb:.1f} MB)')


if __name__ == '__main__':
  app.run(main)
