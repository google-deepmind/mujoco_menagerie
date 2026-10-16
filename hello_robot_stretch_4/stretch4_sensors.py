"""Sensor helpers for the Hello Robot Stretch 4 MJCF.

MJCF can describe a pinhole camera, and `stretch4.xml` does: every camera
carries its calibrated resolution and intrinsics. MJCF cannot describe a
fisheye lens, a multi-channel lidar or a line sensor, so this file supplies
them:

- `Stretch4Camera` renders a camera the way the robot captures it: at its real
  resolution, through the fisheye model for the two wide-angle head cameras,
  and with the gripper depth clipped at the range the real stereo pair reports.
- `Stretch4Lidars` ray casts the two 128-channel head lidars into point clouds,
  using the channel calibration of a real unit.
- `Stretch4LineSensors` ray casts the six base line sensors into the per-bin
  ranges and status codes the robot reports.

Everything here is ported from stretch4_mujoco
(https://github.com/hello-robot/stretch4_mujoco), Hello Robot's simulator for
Stretch 4, where it lives in `enums/stretch_cameras.py`, `fisheye_renderer.py`,
`enums/line_sensor.py`, `mujoco_server_sensor_manager.py` and
`models/stretch_4/lidar_calibration.csv`. The sensor models and calibration
values are unchanged; only the simulator plumbing (threads, locks, rates) was
left behind. The README describes each sensor.

Requires `numpy` and `mujoco`. The fisheye head cameras also need
`opencv-python`.

Example::

    import mujoco
    from stretch4_sensors import Stretch4Camera
    from stretch4_sensors import Stretch4Lidars
    from stretch4_sensors import Stretch4LineSensors

    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    with Stretch4Camera(model, "camera_left_link") as camera:
        image = camera.render(data)  # (1200, 1920, 3) fisheye frame

    clouds = Stretch4Lidars(model).trace(data)  # {"left": (N, 3), "right": ...}
    ranges, codes, points = Stretch4LineSensors(model).trace(data)
"""

from dataclasses import dataclass
from dataclasses import replace
from enum import IntEnum

import mujoco
import numpy as np

try:
  import cv2
except ImportError:  # Only the fisheye cameras need it.
  cv2 = None


ROOT_BODY = 'stretch4'
"""The floating root body. It sits on the floor between the wheels, so it
stands in for the URDF's `base_footprint`, the frame the robot reports its
point clouds in."""

RAY_GEOMGROUP = (1, 0, 0, 1, 0, 0)
"""Geom groups the lidars and line sensors see: the scene (group 0) and the
robot's collision geoms (group 3). Pass your own if your scene puts geoms in
other groups."""


# -----------------------------------------------------------------------------
# Cameras
# -----------------------------------------------------------------------------


@dataclass(frozen=True)
class CameraSpec:
  """A camera's calibration, in the frame MuJoCo renders.

  That frame is the raw sensor frame: x right, y down, z out of the lens. The
  head sensors are mounted on their side, so their frames come out sideways;
  `upright_rot90` is the `np.rot90` that stands them up. `K` and `distortion`
  describe the frame before that rotation.
  """

  width: int
  height: int
  fx: float
  fy: float
  cx: float
  cy: float
  distortion_model: str = 'none'
  """`"none"`, `"equidistant"` (Kannala-Brandt k1..k4, as in OpenCV's fisheye
  module and ROS's `equidistant`), or `"opencv"` (OpenCV's full 14-coefficient
  model: k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tau_x, tau_y)."""
  distortion: tuple[float, ...] = ()
  image_circle_radius_px: float | None = None
  """For a fisheye lens, the radius of the image circle around the optical
  centre. Everything past it is black."""
  upright_rot90: int = 0
  rate_hz: float = 30.0
  """The rate the real robot publishes this camera at."""
  max_depth_m: float | None = None
  """Set for depth cameras: depth past this reads as 0, as the real one does."""

  @property
  def K(self) -> np.ndarray:
    return np.array(
      [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]]
    )

  @property
  def is_fisheye(self) -> bool:
    """Whether the frame is rendered through `FisheyeRenderer`.

    Only the two head cameras are. The centre camera carries distortion
    coefficients as well, but its frames are left as MuJoCo renders them, as
    they are in stretch4_mujoco.
    """
    return self.distortion_model == 'equidistant'

  @property
  def is_depth(self) -> bool:
    return self.max_depth_m is not None

  def scaled(self, scale: float) -> 'CameraSpec':
    """The same camera rendered at `scale` times the resolution."""
    if scale == 1.0:
      return self
    return replace(
      self,
      width=round(self.width * scale),
      height=round(self.height * scale),
      fx=self.fx * scale,
      fy=self.fy * scale,
      cx=self.cx * scale,
      cy=self.cy * scale,
      image_circle_radius_px=(
        None
        if self.image_circle_radius_px is None
        else self.image_circle_radius_px * scale
      ),
    )


_GRIPPER_CAMERA = CameraSpec(
  # Luxonis SR stereo camera, from a real unit's calibration.
  width=640,
  height=400,
  fx=396.84320068359375,
  fy=396.9339904785156,
  cx=313.5003967285156,
  cy=214.98081970214844,
)

CAMERAS: dict[str, CameraSpec] = {
  # AR0234 behind a ~196 degree fisheye lens, head_left.
  'camera_left_link': CameraSpec(
    width=1920,
    height=1200,
    fx=516.9768177458288,
    fy=517.3644719961303,
    cx=933.1137897347201,
    cy=612.4085502991915,
    distortion_model='equidistant',
    distortion=(
      -0.02523316108971853,
      0.011127651341704358,
      -0.005848339151583135,
      0.0007940458582321494,
    ),
    image_circle_radius_px=825,  # Measured on SE4 head_left frames.
    upright_rot90=1,
  ),
  # AR0234 behind a ~196 degree fisheye lens, head_right.
  'camera_right_link': CameraSpec(
    width=1920,
    height=1200,
    fx=519.094494257149,
    fy=518.418931744225,
    cx=974.2541200745,
    cy=575.5427056279029,
    distortion_model='equidistant',
    distortion=(
      -0.025962597755860257,
      0.0111022256976195,
      -0.005205105236511768,
      0.0004939891505609986,
    ),
    image_circle_radius_px=825,  # Measured on SE4 head_left frames, same lens.
    upright_rot90=-1,
  ),
  # IMX378 wide-angle, head_center. Its frames are not distorted, see
  # `CameraSpec.is_fisheye`.
  'camera_center_link': CameraSpec(
    width=4034,
    height=3040,
    fx=2329.4044093344937,
    fy=2338.7901418403994,
    cx=2061.704964987135,
    cy=1518.022485719209,
    distortion_model='opencv',
    distortion=(
      1.2129752168148256,
      5.841373920191379,
      -0.0016434465993966064,
      -0.004229881605602358,
      -0.7139811586224234,
      1.380989287039654,
      6.445212600290566,
      -0.03269498452468606,
      0.001801163713650778,
      0.001218315951582083,
      0.0031951489521495073,
      -0.0006358028024435923,
      0.002186544781267366,
      -0.020580090114988337,
    ),
    upright_rot90=-1,
    rate_hz=10.0,
  ),
  'gripper_camera_left_rgb': _GRIPPER_CAMERA,
  'gripper_camera_right_rgb': _GRIPPER_CAMERA,
  # The stereo pair's depth, modelled as a MuJoCo depth camera with the left
  # camera's intrinsics.
  'gripper_camera_stereo_depth': replace(_GRIPPER_CAMERA, max_depth_m=1.0),
}
"""Every Stretch 4 camera, keyed by its MJCF name."""

CENTER_CAMERA_LOW_RES_SCALE = 1280 / 4034
"""Scale that gives the centre camera's 1280x965 low resolution stream."""


def intrinsics_from_model(
  model: mujoco.MjModel,
  camera_name: str,
  width: int | None = None,
  height: int | None = None,
) -> np.ndarray:
  """The OpenCV K matrix MuJoCo renders `camera_name` with at a given size.

  The camera must define `sensorsize`, which every Stretch 4 camera does.
  Width and height default to the camera's `resolution`.

  MuJoCo stores the principal point as an offset from the image centre in
  sensor units, and its x offset runs the opposite way to OpenCV's: the XML's
  `principalpixel` is `(W/2 - cx, cy - H/2)`. This undoes that.
  """
  cam_id = model.camera(camera_name).id
  sensor_w, sensor_h = model.cam_sensorsize[cam_id]
  if sensor_w == 0 or sensor_h == 0:
    raise ValueError(
      f'Camera {camera_name} has no sensorsize, so MuJoCo renders it from fovy.'
    )
  res_w, res_h = model.cam_resolution[cam_id]
  width = res_w if width is None else width
  height = res_h if height is None else height
  focal_x, focal_y, principal_x, principal_y = model.cam_intrinsic[cam_id]
  px_per_m_x = width / sensor_w
  px_per_m_y = height / sensor_h
  return np.array(
    [
      [focal_x * px_per_m_x, 0.0, width / 2 - principal_x * px_per_m_x],
      [0.0, focal_y * px_per_m_y, height / 2 + principal_y * px_per_m_y],
      [0.0, 0.0, 1.0],
    ]
  )


class Stretch4Camera:
  """Renders one Stretch 4 camera the way the robot captures it.

  Pinhole cameras are a plain `mujoco.Renderer` render, since the MJCF already
  holds their intrinsics. The two head cameras go through `FisheyeRenderer`.
  The gripper depth camera returns metres, zeroed past `max_depth_m`.
  """

  def __init__(
    self, model: mujoco.MjModel, camera_name: str, scale: float = 1.0
  ):
    """`scale` renders at a fraction of full resolution, scaling K with it."""
    if camera_name not in CAMERAS:
      raise ValueError(
        f'Unknown camera {camera_name}, expected one of {list(CAMERAS)}.'
      )
    self.camera_name = camera_name
    self.spec = CAMERAS[camera_name].scaled(scale)

    width, height = self.spec.width, self.spec.height
    self._fisheye: FisheyeRenderer | None = None
    if self.spec.is_fisheye:
      self._fisheye = FisheyeRenderer(
        self.spec.fx,
        self.spec.fy,
        self.spec.cx,
        self.spec.cy,
        self.spec.distortion,
        self.spec.width,
        self.spec.height,
        self.spec.image_circle_radius_px,
      )
      width, height = self._fisheye.render_size

    # mujoco.Renderer refuses to render past the model's offscreen buffer.
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)

    self._renderer = mujoco.Renderer(model, height=height, width=width)
    if self.spec.is_depth:
      self._renderer.enable_depth_rendering()

  def render(self, data: mujoco.MjData, upright: bool = False) -> np.ndarray:
    """Render the frame for `data`.

    `upright` turns the head cameras' sideways frames upright. `spec.K`
    describes the frame without it.
    """
    if self._fisheye is not None:
      self._fisheye.render_views(self._renderer, data, self.camera_name)
      frame = self._fisheye.project()
    else:
      self._renderer.update_scene(data, camera=self.camera_name)
      frame = self._renderer.render()
      if self.spec.is_depth:
        frame = np.where(frame > self.spec.max_depth_m, 0, frame)

    if upright and self.spec.upright_rot90:
      frame = np.rot90(frame, self.spec.upright_rot90)
    return frame

  def close(self) -> None:
    self._renderer.close()

  def __enter__(self) -> 'Stretch4Camera':
    return self

  def __exit__(self, *exc) -> None:
    self.close()


FISHEYE_VIEW_RESOLUTION_SCALE = 0.75
"""View resolution as a fraction of the one that matches the lens.

At 1.0 each view is sized so that nowhere it is sampled is it coarser than the
fisheye frame, so no detail is lost. 0.75 renders a little over half the pixels
for a barely visible softening, which on an integrated GPU takes a head camera
from ~27ms to ~19ms a frame.
"""

FISHEYE_LENS_PROTRUSION_M = 0.006
"""How far in front of the camera frame the views are rendered from.

The head lenses stand proud of the shell, so the real cameras only see the
shell where it bulges out at the bottom of the frame. The optical frame sits
just behind the shell's opening, and rendering from there rings the whole image
circle with the edge of that opening.
"""

FISHEYE_VIGNETTE_WIDTH_PX = 30.0
"""Width of the ring over which the image circle fades to black.

Measured on the SE4 head cameras, where the brightness at the edge of the image
circle drops from full to black over roughly 30 pixels.
"""

View = tuple[tuple[float, float, float], tuple[float, float, float]]
"""A pinhole view as (forward, image-down) in the optical frame.

The optical frame has x right, y down and z forward.
"""


def views_yawed_about_y(*yaw_degrees: float) -> tuple[View, ...]:
  """Views fanned out across the frame's long (x) axis."""
  return tuple(
    (
      (float(np.sin(np.radians(yaw))), 0.0, float(np.cos(np.radians(yaw)))),
      (0.0, 1.0, 0.0),
    )
    for yaw in yaw_degrees
  )


FISHEYE_VIEWS = views_yawed_about_y(0, -60, 60)
"""One view straight ahead and one either side along the frame's long axis.

Every view costs a fixed ~3ms on top of its pixels, but a pinhole view also
oversamples its edges by sec^2 of the angle off its axis, so fewer, wider views
pay for themselves in pixels instead. Three is the cheapest: two views ~50
degrees either side are slower, and the five faces of a cube are no faster.
"""


@dataclass
class _View:
  forward: np.ndarray
  """Forward axis in the optical frame."""
  down: np.ndarray
  """Image-down axis in the optical frame."""
  u_min: float
  v_min: float
  pixels_per_unit: float
  rect: mujoco.MjrRect
  """Where in the framebuffer the view renders the part of it the lens sees."""


def _smallest_step(u: np.ndarray, v: np.ndarray) -> float:
  """The least a one-pixel step in the frame moves `(u, v)`, in any direction.

  `u` and `v` are per-pixel maps, NaN where the view is not sampled. The
  answer is the smallest singular value of the map's Jacobian over the pixels
  the view serves, which is the finest detail the view has to resolve.
  """
  du_dx = (u[1:-1, 2:] - u[1:-1, :-2]) / 2
  dv_dx = (v[1:-1, 2:] - v[1:-1, :-2]) / 2
  du_dy = (u[2:, 1:-1] - u[:-2, 1:-1]) / 2
  dv_dy = (v[2:, 1:-1] - v[:-2, 1:-1]) / 2
  # Smallest eigenvalue of J^T J, in closed form.
  a = du_dx**2 + dv_dx**2
  b = du_dx * du_dy + dv_dx * dv_dy
  d = du_dy**2 + dv_dy**2
  smallest = 0.5 * (a + d) - np.sqrt(0.25 * (a - d) ** 2 + b**2)
  return float(np.sqrt(np.nanmin(np.maximum(smallest, 0.0))))


def fisheye_angles(
  fx: float,
  fy: float,
  cx: float,
  cy: float,
  distortion_params: tuple,
  width: int,
  height: int,
  max_normalized_radius: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """`(theta, phi, r_d)` for every pixel of a Kannala-Brandt fisheye frame.

  `theta` is the ray's angle off the optical axis, `phi` its direction around
  it and `r_d` the pixel's normalized distance from the optical centre. The
  model maps theta to `r_d = theta * (1 + k1 theta^2 + ... + k4 theta^8)`, so
  going from pixels to rays means inverting that polynomial, done here with
  Newton's method. `r_d` is clamped to `max_normalized_radius` first: past the
  image circle the polynomial is extrapolating, and those pixels are black.
  """
  k1, k2, k3, k4 = distortion_params[:4]
  grid_y, grid_x = np.indices((height, width), dtype=np.float64)
  x_d = (grid_x - cx) / fx
  y_d = (grid_y - cy) / fy
  r_d = np.hypot(x_d, y_d)

  target = np.minimum(r_d, max_normalized_radius)
  theta = target.copy()
  for _ in range(10):
    theta2 = theta * theta
    residual = (
      theta * (1 + theta2 * (k1 + theta2 * (k2 + theta2 * (k3 + theta2 * k4))))
      - target
    )
    slope = 1 + theta2 * (
      3 * k1 + theta2 * (5 * k2 + theta2 * (7 * k3 + theta2 * 9 * k4))
    )
    theta -= residual / slope

  return theta, np.arctan2(y_d, x_d), r_d


class FisheyeRenderer:
  """Renders a wide-angle fisheye camera from a few pinhole views.

  A single pinhole render cannot be warped into the SE4 head cameras: their
  lenses see ~98 degrees off axis (a ~196 degree image circle), and a pinhole
  camera puts anything at 90 degrees at infinity. So the scene is rendered
  from several pinhole views fanned out around the camera, and every fisheye
  pixel looks up the view nearest its ray. Each view renders only the part of
  the scene the lens sees through it.

  The frame comes back at the calibration's resolution and intrinsics, with
  no crop, so the published K and D describe it exactly, and with the image
  circle and its vignette where the real lens puts them.
  """

  def __init__(
    self,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    distortion_params: tuple,
    width: int,
    height: int,
    image_circle_radius_px: float,
    views: tuple[View, ...] = FISHEYE_VIEWS,
    view_resolution_scale: float = FISHEYE_VIEW_RESOLUTION_SCALE,
    lens_protrusion_m: float = FISHEYE_LENS_PROTRUSION_M,
  ):
    if cv2 is None:
      raise ImportError(
        'The fisheye head cameras need opencv-python: pip install opencv-python'
      )
    self.lens_protrusion_m = lens_protrusion_m
    focal = max(fx, fy)

    circle_radius = image_circle_radius_px / focal
    theta, phi, r_d = fisheye_angles(
      fx, fy, cx, cy, distortion_params, width, height, circle_radius
    )
    rays = np.stack(
      [
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta),
      ],
      axis=-1,
    )

    fade = np.clip(
      (circle_radius - r_d) * focal / FISHEYE_VIGNETTE_WIDTH_PX, 0.0, 1.0
    )
    fade = fade * fade * (3.0 - 2.0 * fade)
    seen = fade > 0
    vignette = np.round(fade * 255).astype(np.uint8)
    self._vignette = cv2.merge([vignette] * 3)

    forwards = np.array([forward for forward, _ in views], dtype=np.float64)
    view_index = (rays @ forwards.T).argmax(axis=-1)

    self.map_x = np.full((height, width), -1.0, dtype=np.float32)
    self.map_y = np.full((height, width), -1.0, dtype=np.float32)
    self._views: list[_View] = []
    atlas_x = 0
    for index, (forward, down) in enumerate(views):
      forward = np.array(forward, dtype=np.float64)
      down = np.array(down, dtype=np.float64)
      right = np.cross(down, forward)

      in_view = (view_index == index) & seen
      if not in_view.any():
        continue
      with np.errstate(divide='ignore', invalid='ignore'):
        depth = rays @ forward
        u_all = np.where(in_view, (rays @ right) / depth, np.nan)
        v_all = np.where(in_view, (rays @ down) / depth, np.nan)
      u = u_all[in_view]
      v = v_all[in_view]

      # Face pixels per unit of u and v. A pinhole view samples its edges
      # far more densely than its centre, so rather than a fixed size,
      # each view gets just enough that no fisheye pixel it serves is
      # coarser than the lens itself.
      pixels_per_unit = view_resolution_scale / _smallest_step(u_all, v_all)

      # Pad by a couple of pixels so bilinear sampling never reads past
      # the part of the view that was rendered. Stopping exactly at the
      # boundary with the next view leaves a dark line along every seam.
      pad = 2.0 / pixels_per_unit
      u_min = u.min() - pad
      v_min = v.min() - pad
      rect_width = int(np.ceil((u.max() + pad - u_min) * pixels_per_unit))
      rect_height = int(np.ceil((v.max() + pad - v_min) * pixels_per_unit))

      # The views render side by side into one framebuffer that is read
      # back in one go. OpenGL's rows run bottom up, so v is flipped here
      # rather than flipping the pixels after every read.
      self.map_x[in_view] = atlas_x + (u - u_min) * pixels_per_unit - 0.5
      self.map_y[in_view] = rect_height - 0.5 - (v - v_min) * pixels_per_unit

      self._views.append(
        _View(
          forward=forward,
          down=down,
          u_min=u_min,
          v_min=v_min,
          pixels_per_unit=pixels_per_unit,
          rect=mujoco.MjrRect(atlas_x, 0, rect_width, rect_height),
        )
      )
      atlas_x += rect_width

    atlas_height = max(view.rect.height for view in self._views)
    self._atlas_rect = mujoco.MjrRect(0, 0, atlas_x, atlas_height)
    self._atlas = np.zeros((atlas_height, atlas_x, 3), dtype=np.uint8)

  @property
  def render_size(self) -> tuple[int, int]:
    """`(width, height)` the renderer handed to `render_views` needs."""
    return (self._atlas_rect.width, self._atlas_rect.height)

  def render_views(
    self, renderer: mujoco.Renderer, data: mujoco.MjData, camera_name: str
  ):
    """Render the views into `renderer`'s framebuffer."""
    renderer.update_scene(data=data, camera=camera_name)
    scene = renderer.scene

    # A headlight is lit along whichever way the GL camera faces, so each
    # view would be lit differently and the seams would show. Pin it in the
    # world, pointing where the real camera looks.
    for light_index in range(scene.nlight):
      scene.lights[light_index].headlight = 0

    gl_camera = scene.camera[0]
    # MuJoCo's camera looks down -z with +y up; the optical frame is that
    # rotated 180 degrees about x.
    z_axis = np.array(gl_camera.forward, dtype=np.float64)
    y_axis = -np.array(gl_camera.up, dtype=np.float64)
    optical_to_world = np.stack(
      [np.cross(y_axis, z_axis), y_axis, z_axis], axis=1
    )
    # `scene.camera` is a stereo pair: [0] and [1] are the left and right
    # eyes, each offset half of `vis.global_.ipd` (34mm by default) along
    # the camera's x axis. The lens sits at neither -- it sits at their
    # midpoint, which is the camera's own position. Rendering from
    # `camera[0]` puts every head frame 34mm to the left of where the
    # camera is, which is an error the size of a tennis ball at 1m.
    eye_midpoint = 0.5 * (
      np.array(gl_camera.pos, dtype=np.float64)
      + np.array(scene.camera[1].pos, dtype=np.float64)
    )
    position = eye_midpoint + z_axis * self.lens_protrusion_m
    near = gl_camera.frustum_near

    # The views draw straight into the renderer's GL context, which
    # `mujoco.Renderer` keeps private.
    if renderer._gl_context:
      renderer._gl_context.make_current()

    for view in self._views:
      forward = optical_to_world @ view.forward
      up = -(optical_to_world @ view.down)
      u_max = view.u_min + view.rect.width / view.pixels_per_unit
      v_max = view.v_min + view.rect.height / view.pixels_per_unit
      for camera in (scene.camera[0], scene.camera[1]):
        camera.pos[:] = position
        camera.forward[:] = forward
        camera.up[:] = up
        camera.frustum_center = 0.5 * (view.u_min + u_max) * near
        camera.frustum_width = 0.5 * (u_max - view.u_min) * near
        # GL's y points up, the image's v points down.
        camera.frustum_bottom = -v_max * near
        camera.frustum_top = -view.v_min * near

      mujoco.mjr_render(view.rect, scene, renderer._mjr_context)

    mujoco.mjr_readPixels(
      self._atlas, None, self._atlas_rect, renderer._mjr_context
    )

  def project(self) -> np.ndarray:
    """Warp the last `render_views` into the fisheye frame."""
    frame = cv2.remap(
      self._atlas,
      self.map_x,
      self.map_y,
      cv2.INTER_LINEAR,
      borderMode=cv2.BORDER_CONSTANT,
      borderValue=(0, 0, 0),
    )
    return cv2.multiply(frame, self._vignette, scale=1 / 255)


# -----------------------------------------------------------------------------
# Head lidars
# -----------------------------------------------------------------------------

LIDAR_SITES = {'left': 'lidar_left', 'right': 'lidar_right'}

LIDAR_MAX_RANGE_M = 30.0

LIDAR_AZIMUTH_STEPS = 250
"""Firings per revolution. Each fires all 128 channels."""

LIDAR_CHANNELS: tuple[tuple[float, float], ...] = (
  # (elevation, azimuth offset) in degrees for channels 1..128, from a real
  # unit's calibration. Elevation is measured up from the site's xy plane.
  (-4.316, -2.191), (-3.533, -10.941), (-2.536, -2.210), (-1.726, -10.945),
  (-0.835, -2.230), (0.002, -10.962), (0.812, -2.250), (1.674, -10.987),
  (2.476, 1.661), (3.340, 10.370), (4.136, 1.629), (4.972, 10.366),
  (5.758, 1.602), (6.567, 10.371), (7.343, 1.577), (8.129, 10.385),
  (8.925, -2.262), (9.684, -11.102), (10.443, -2.288), (11.186, -11.169),
  (11.932, -2.315), (12.661, -11.245), (13.396, -2.343), (14.111, -11.327),
  (14.982, 1.478), (15.702, 10.522), (16.393, 1.479), (17.098, 10.617),
  (17.787, 1.480), (18.477, 10.694), (19.165, 1.483), (19.843, 10.755),
  (20.568, -2.362), (21.216, -11.745), (21.917, -2.396), (22.556, -11.866),
  (23.257, -2.431), (23.887, -11.995), (24.588, -2.467), (25.209, -12.134),
  (25.874, 1.625), (26.532, 11.415), (27.189, 1.644), (27.833, 11.554),
  (28.500, 1.663), (29.128, 11.700), (29.806, 1.683), (30.418, 11.856),
  (31.141, -2.542), (31.830, -12.860), (32.434, -2.581), (33.111, -13.046),
  (33.725, -2.622), (34.389, -13.246), (35.013, -2.666), (35.663, -13.458),
  (36.260, 1.893), (36.926, 12.930), (37.549, 1.934), (38.194, 13.198),
  (38.837, 1.976), (39.461, 13.455), (40.125, 2.018), (40.730, 13.699),
  (41.450, -2.816), (42.165, -14.693), (42.735, -2.830), (43.421, -15.023),
  (44.020, -2.870), (44.681, -15.329), (45.304, -2.937), (45.946, -15.613),
  (46.552, 2.376), (47.250, 15.464), (47.843, 2.299), (48.513, 15.881),
  (49.131, 2.390), (49.773, 16.334), (50.417, 2.677), (51.031, 16.824),
  (51.759, -3.177), (52.479, -17.707), (53.055, -3.233), (53.740, -18.202),
  (54.354, -3.283), (55.002, -18.733), (55.657, -3.328), (56.266, -19.303),
  (56.910, 3.285), (57.673, 20.055), (58.226, 3.462), (58.933, 20.913),
  (59.546, 3.671), (60.196, 21.830), (60.872, 3.918), (61.464, 22.811),
  (62.286, -3.827), (63.049, -23.567), (63.635, -3.914), (64.338, -24.604),
  (64.996, -4.001), (65.632, -25.768), (66.368, -4.091), (66.928, -27.085),
  (67.686, 5.346), (68.740, 31.193), (69.087, 5.773), (69.995, 33.439),
  (70.503, 6.284), (71.231, 36.063), (71.938, 6.904), (72.435, 39.147),
  (73.523, -5.187), (74.951, -42.338), (75.024, -5.582), (76.140, -46.646),
  (76.553, -6.066), (77.261, -51.845), (78.112, -6.674), (78.281, -58.132),
  (79.356, 25.063), (80.802, 29.750), (82.192, 36.287), (83.442, 45.708),
  (85.099, -7.449), (86.772, -2.293), (88.354, 13.736), (89.078, 59.895),
)  # fmt: skip


def lidar_ray_directions(
  azimuth_steps: int = LIDAR_AZIMUTH_STEPS,
) -> np.ndarray:
  """Unit ray directions in the lidar site's frame, (azimuth_steps * 128, 3).

  Rows are grouped by firing: all 128 channels at the first azimuth, then the
  next. Each channel's azimuth is the spin angle plus its calibrated offset.
  """
  elevation, azimuth_offset = np.deg2rad(np.array(LIDAR_CHANNELS)).T
  spin = np.linspace(-np.pi, np.pi, azimuth_steps)
  azimuth = spin[:, None] + azimuth_offset[None, :]
  azimuth = ((azimuth + np.pi) % (2 * np.pi) - np.pi).ravel()
  elevation = np.tile(elevation, azimuth_steps)
  return np.stack(
    (
      np.cos(elevation) * np.cos(azimuth),
      np.cos(elevation) * np.sin(azimuth),
      np.sin(elevation),
    ),
    axis=-1,
  )


class Stretch4Lidars:
  """Ray casts the two hemispherical head lidars with `mujoco.mj_multiRay`.

  Each lidar sees the scene and the rest of the robot (so the arm shows up in
  the cloud, as it does on the real robot) but not the head it is mounted in.
  """

  def __init__(
    self,
    model: mujoco.MjModel,
    azimuth_steps: int = LIDAR_AZIMUTH_STEPS,
    max_range: float = LIDAR_MAX_RANGE_M,
    geomgroup: tuple[int, ...] = RAY_GEOMGROUP,
  ):
    self.model = model
    self.max_range = max_range
    self.geomgroup = np.array(geomgroup, dtype=np.uint8)
    self.site_ids = {
      key: model.site(name).id for key, name in LIDAR_SITES.items()
    }
    self.head_body_id = model.body('head_link').id
    self.root_body_id = model.body(ROOT_BODY).id

    self.directions = lidar_ray_directions(azimuth_steps)
    """Ray directions in the site frame, see `lidar_ray_directions`."""
    self._dist = np.zeros(len(self.directions))
    self._geomid = np.zeros(len(self.directions), dtype=np.int32)

  def trace(
    self, data: mujoco.MjData, frame: str = 'robot'
  ) -> dict[str, np.ndarray]:
    """Point clouds, `{"left": (N, 3), "right": (M, 3)}`, of the rays that hit.

    `frame` is `"robot"` (the `stretch4` root body, i.e. `base_footprint`, as
    stretch4_mujoco reports them), `"sensor"` (each lidar's own site) or
    `"world"`. Call `mj_forward` (or step) before tracing.
    """
    if frame not in ('robot', 'sensor', 'world'):
      raise ValueError(
        f"frame must be 'robot', 'sensor' or 'world', not {frame!r}"
      )

    clouds = {}
    for key, site_id in self.site_ids.items():
      site_pos = data.site_xpos[site_id]
      site_mat = data.site_xmat[site_id].reshape(3, 3)
      mujoco.mj_multiRay(
        m=self.model,
        d=data,
        pnt=site_pos.astype(np.float64).reshape(3, 1),
        vec=(self.directions @ site_mat.T).ravel(),
        geomgroup=self.geomgroup,
        flg_static=1,
        bodyexclude=self.head_body_id,
        geomid=self._geomid,
        dist=self._dist,
        normal=None,
        nray=len(self.directions),
        cutoff=self.max_range,
      )
      hit = (
        (self._geomid != -1) & (self._dist > 0) & (self._dist < self.max_range)
      )
      points = self.directions[hit] * self._dist[hit, None]

      if frame != 'sensor':
        points = points @ site_mat.T + site_pos
      if frame == 'robot':
        root_pos = data.xpos[self.root_body_id]
        root_mat = data.xmat[self.root_body_id].reshape(3, 3)
        points = (points - root_pos) @ root_mat
      clouds[key] = points

    return clouds


# -----------------------------------------------------------------------------
# Line sensors
# -----------------------------------------------------------------------------

# Six PixArt J3 line sensors ring the base, each projecting a laser line down
# and out onto the floor. The values below mirror `line_sensor_geometry` in
# stretch4_body's robot_params_SE4.py.

NUM_LINE_SENSORS = 6
"""Sites `line_sensor_0` ... `line_sensor_5`, clockwise from robot forward."""

LINE_SENSOR_NUM_BINS = 320
"""Range bins per sensor (`pixart_report_num`)."""

LINE_SENSOR_FOV_DEG = 103.0
"""Width of the fan the bins span (`sensor_horizontal_fov_degrees`)."""

LINE_SENSOR_MAX_RANGE_M = 1.0
"""Furthest depth the chip reports as a distance. Past this a return comes back
as `LineSensorCode.beyond_limit`. Not in the robot params; the floor sits at
~0.23m, so this only matters for cliffs and walls."""

LINE_SENSOR_RANGE_BEYOND_LIMIT_M = 5.09
LINE_SENSOR_RANGE_NO_RETURN_M = 5.11
"""What the real sensors report, in metres, in place of a distance. The real
robot NaNs these out of its ranges and keeps them as codes; so does
`Stretch4LineSensors`."""


class LineSensorCode(IntEnum):
  """What a line sensor bin carries.

  Matches stretch4_body's `protocol.CODE_*`.
  """

  valid = 0
  """A real distance measurement."""
  beyond_limit = 1
  """5.09 -- a return from past the range limit: the floor dropped away."""
  no_return = 2
  """5.11 -- nothing reflected back at all."""
  other_invalid = 3
  """Zero, negative, non-finite, or an unknown code. Never simulated."""


class Stretch4LineSensors:
  """Ray casts the six base line sensors with `mujoco.mj_multiRay`.

  Each sensor fans `LINE_SENSOR_NUM_BINS` rays across `LINE_SENSOR_FOV_DEG` in
  the plane of its site's x (boresight) and y (across the fan) axes. Like the
  real sensor, a bin reports depth along the boresight, not ray length, so a
  flat floor reads the same range in every bin.

  The sensors point down at the floor from the rim of the base, so they see
  the scene and the robot (a lowered gripper, say) but not the base shell
  their emitters sit on.
  """

  def __init__(
    self,
    model: mujoco.MjModel,
    max_range: float = LINE_SENSOR_MAX_RANGE_M,
    cutoff_dist: float = 5.0,
    geomgroup: tuple[int, ...] = RAY_GEOMGROUP,
  ):
    self.model = model
    self.max_range = max_range
    self.cutoff_dist = cutoff_dist
    self.geomgroup = np.array(geomgroup, dtype=np.uint8)
    self.site_ids = [
      model.site(f'line_sensor_{i}').id for i in range(NUM_LINE_SENSORS)
    ]
    self.base_body_id = model.body('base_link').id
    self.root_body_id = model.body(ROOT_BODY).id

    # Bin 0 is the left edge of the fan (+y), as the real robot orders them.
    half_fov = np.deg2rad(LINE_SENSOR_FOV_DEG) / 2
    fan_angles = np.linspace(half_fov, -half_fov, LINE_SENSOR_NUM_BINS)
    self._cos_fan = np.cos(fan_angles)
    self._local_vecs = np.stack(
      (self._cos_fan, np.sin(fan_angles), np.zeros(LINE_SENSOR_NUM_BINS)),
      axis=-1,
    )

    self._dist = np.zeros(LINE_SENSOR_NUM_BINS, dtype=np.float64)
    self._geomid = np.zeros(LINE_SENSOR_NUM_BINS, dtype=np.int32)

  def trace(
    self, data: mujoco.MjData
  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`(ranges, codes, points)`, each indexed `[sensor, bin]`.

    - `ranges`, (6, 320): depth along each sensor's boresight, in metres. NaN
      wherever the bin holds a status code rather than a distance.
    - `codes`, (6, 320): the `LineSensorCode` per bin, the authority on what a
      bin carries.
    - `points`, (6, 320, 3): where each bin hit, in the robot (`stretch4`, i.e.
      `base_footprint`) frame. NaN wherever the range is.
    """
    shape = (NUM_LINE_SENSORS, LINE_SENSOR_NUM_BINS)
    ranges = np.full(shape, np.nan)
    codes = np.full(shape, LineSensorCode.valid, dtype=np.uint8)
    points = np.full((*shape, 3), np.nan)

    root_pos = data.xpos[self.root_body_id]
    root_mat = data.xmat[self.root_body_id].reshape(3, 3)

    for i, site_id in enumerate(self.site_ids):
      site_pos = data.site_xpos[site_id]
      world_vecs = self._local_vecs @ data.site_xmat[site_id].reshape(3, 3).T

      mujoco.mj_multiRay(
        m=self.model,
        d=data,
        pnt=site_pos.astype(np.float64).reshape(3, 1),
        vec=world_vecs.ravel(),
        geomgroup=self.geomgroup,
        flg_static=1,
        bodyexclude=self.base_body_id,
        geomid=self._geomid,
        dist=self._dist,
        normal=None,
        nray=LINE_SENSOR_NUM_BINS,
        cutoff=self.cutoff_dist,
      )

      is_hit = self._geomid != -1
      depth = self._dist * self._cos_fan
      is_valid = is_hit & (depth <= self.max_range)

      codes[i, ~is_hit] = LineSensorCode.no_return
      codes[i, is_hit & ~is_valid] = LineSensorCode.beyond_limit
      ranges[i, is_valid] = depth[is_valid]

      world_points = (
        site_pos + world_vecs[is_valid] * self._dist[is_valid, np.newaxis]
      )
      points[i, is_valid] = (world_points - root_pos) @ root_mat

    return ranges, codes, points
