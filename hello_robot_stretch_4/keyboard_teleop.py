"""Drive Stretch 4 from the keyboard and watch its sensors in Rerun.

Loads `scene.xml` (or `scene_pg4.xml` with `--pg4`) in the MuJoCo viewer.
The cameras, head lidars and line sensors are rendered with
`stretch4_sensors.py` and streamed to Rerun.

A small, single-file port of `examples/keyboard_teleop.py` in stretch4_mujoco
(https://github.com/hello-robot/stretch4_mujoco), keeping its key bindings.
Keys are read globally with pynput, as in the original, so the MuJoCo window
does not need focus, and the viewer's own shortcuts do not get in the way.

Requires `mujoco`, `numpy`, `opencv-python`, `pynput` and `rerun-sdk`.

    python keyboard_teleop.py [--pg4]
"""

import argparse
import pathlib
import time

import mujoco
import mujoco.viewer
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from pynput import keyboard
from stretch4_sensors import Stretch4Camera
from stretch4_sensors import Stretch4Lidars
from stretch4_sensors import Stretch4LineSensors

HELP = """
Keyboard controls
=================
W / S     base forward / back      U / J   lift up / down
A / D     base left / right        H / K   arm in / out
Q / E     base rotate left / right O / P   wrist yaw
L         print joint positions    C / V   wrist pitch
I         cycle eye colour         T / Y   wrist roll
                                   N / M   gripper open / close
Close the viewer or press Ctrl+C to quit.
"""

BASE_SPEED = 0.2  # m/s
BASE_TURN_SPEED = 0.8  # rad/s

# key: (actuator, velocity in ctrl units per second), as in stretch4_mujoco.
JOINT_KEYS = {
  'u': ('lift', 0.10),
  'j': ('lift', -0.10),
  'h': ('arm', -0.08),
  'k': ('arm', 0.08),
  'o': ('wrist_yaw', -0.3),
  'p': ('wrist_yaw', 0.3),
  'c': ('wrist_pitch', 0.3),
  'v': ('wrist_pitch', -0.3),
  't': ('wrist_roll', 0.3),
  'y': ('wrist_roll', -0.3),
}
# key: fraction of the fingers' travel per second, towards closed.
GRIPPER_KEYS = {'n': -1.0, 'm': 1.0}
FINGERS = ('gripper_left_finger', 'gripper_right_finger')

SENSOR_HZ = 10.0
CAMERA_SCALES = {
  'camera_left_link': 0.5,
  'camera_center_link': 0.2,
  'camera_right_link': 0.5,
  'gripper_camera_left_rgb': 1.0,
  'gripper_camera_right_rgb': 1.0,
  'gripper_camera_stereo_depth': 1.0,
}
EYE_COLOURS = [(0.55, 0.2, 1.0), (0.0, 0.85, 0.95), (0.1, 0.9, 0.35),
               (1.0, 0.45, 0.0), (1.0, 1.0, 1.0)]  # fmt: skip
POINT_COLOURS = {'left': (0, 220, 255), 'right': (255, 180, 0)}


# The omnibase controller from stretch4_mujoco's mujoco_server.py, with its
# config.robot_settings_se4['omnibase'] values: H0 kinematics, velocity
# limits, and all three wheels ramped together along trapezoidal velocity
# profiles. Stepping the wheels straight to speed breaks their traction.
WHEEL_DIAMETER_M = 0.20
BASE_RADIUS_M = 0.174
GEAR_RATIO = 6
WHEEL_POLARITY = -1
MAX_VEL_XY_M, MAX_VEL_W_R = 0.3, 2.0
MAX_ACCEL_XY_M, MAX_ACCEL_W_R = 0.25, 2.0
WHEEL_MAX_VEL, WHEEL_DEFAULT_ACCEL = 50.0, 15.0
WHEELS = ('left_wheel_vel', 'back_wheel_vel', 'right_wheel_vel')
"""H0's wheel order: wheel0, wheel1, wheel2."""


class OmniBase:
  """Ramps the three wheel velocity actuators towards a base twist."""

  def __init__(self):
    gamma = np.radians([30.0, 150.0, 270.0])  # The 'calder' layout.
    self.h0 = np.stack(
      [np.sin(gamma), -np.cos(gamma), np.full(3, -BASE_RADIUS_M)], axis=1
    ) / (WHEEL_DIAMETER_M / 2)
    self.ctrl = np.zeros(3)
    """Wheel actuator ctrl, in `WHEELS` order: motor velocities in rad/s."""
    self._target = np.zeros(3)
    self._accel = np.full(3, WHEEL_DEFAULT_ACCEL)

  def set_twist(self, vx: float, vy: float, wz: float) -> None:
    """Command a base twist: m/s forward and left, rad/s counterclockwise."""
    vx, vy = np.clip([vx, vy], -MAX_VEL_XY_M, MAX_VEL_XY_M)
    wz = np.clip(wz, -MAX_VEL_W_R, MAX_VEL_W_R)
    motor = self.h0 @ np.array([vx, vy, wz]) * GEAR_RATIO

    # Every wheel reaches its target at the same moment, at the rate the
    # slowest-to-change wheel is allowed.
    max_wheel_accel = (2 / WHEEL_DIAMETER_M) * (
      MAX_ACCEL_XY_M + BASE_RADIUS_M * MAX_ACCEL_W_R
    )
    delta = (motor - WHEEL_POLARITY * self.ctrl) / GEAR_RATIO
    max_delta = np.max(np.abs(delta))
    accel = np.zeros(3)
    if max_delta > 0:
      accel = np.abs(delta) / (max_delta / max_wheel_accel) * GEAR_RATIO
    self._accel = np.where(accel > 1e-3, accel, WHEEL_DEFAULT_ACCEL)
    self._target = np.clip(
      WHEEL_POLARITY * motor, -WHEEL_MAX_VEL, WHEEL_MAX_VEL
    )

  def step(self, dt: float) -> np.ndarray:
    """Advance the ramps by `dt` and return the wheels' ctrl."""
    up = np.minimum(self._target, self.ctrl + self._accel * dt)
    down = np.maximum(self._target, self.ctrl - self._accel * dt)
    self.ctrl = np.where(self.ctrl < self._target, up, down)
    return self.ctrl


def blueprint() -> rrb.Blueprint:
  cameras = [
    rrb.Spatial2DView(origin=f'cameras/{name}', name=name)
    for name in CAMERA_SCALES
  ]
  return rrb.Blueprint(
    rrb.Horizontal(
      rrb.Grid(*cameras),
      rrb.Spatial3DView(origin='robot', name='Lidars and line sensors'),
      column_shares=[3, 2],
    ),
    collapse_panels=True,
  )


def log_sensors(cameras, lidars, line_sensors, data: mujoco.MjData) -> None:
  rr.set_time('sim_time', duration=data.time)
  for name, camera in cameras.items():
    frame = camera.render(data, upright=True)
    if camera.spec.is_depth:
      rr.log(f'cameras/{name}', rr.DepthImage(frame, meter=1.0))
    else:
      rr.log(f'cameras/{name}', rr.Image(frame))
  for side, points in lidars.trace(data).items():
    rr.log(
      f'robot/lidar_{side}',
      rr.Points3D(points, radii=0.008, colors=POINT_COLOURS[side]),
    )
  _, _, points = line_sensors.trace(data)
  points = points.reshape(-1, 3)
  rr.log(
    'robot/line_sensors',
    rr.Points3D(
      points[~np.isnan(points[:, 0])], radii=0.008, colors=(0, 0, 160)
    ),
  )


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
  parser.add_argument(
    '--pg4', action='store_true', help='Use the parallel jaw gripper.'
  )
  args = parser.parse_args()

  scene = 'scene_pg4.xml' if args.pg4 else 'scene.xml'
  model = mujoco.MjModel.from_xml_path(
    str(pathlib.Path(__file__).parent / scene)
  )
  data = mujoco.MjData(model)
  mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
  mujoco.mj_forward(model, data)

  wheels = [model.actuator(name).id for name in WHEELS]
  base = OmniBase()
  twist = np.zeros(3)
  # The home keyframe commands the fingers open; closed is the other end.
  open_ctrl = {f: data.ctrl[model.actuator(f).id] for f in FINGERS}
  eye_material = model.material('led_ring').id
  eye_colour = 0

  # Created before the viewer, since they may grow the offscreen buffer.
  cameras = {
    name: Stretch4Camera(model, name, scale)
    for name, scale in CAMERA_SCALES.items()
  }
  lidars = Stretch4Lidars(model)
  line_sensors = Stretch4LineSensors(model)
  snapshot = mujoco.MjData(model)

  rr.init('Stretch 4 keyboard teleop', spawn=True)
  rr.send_blueprint(blueprint())
  rr.log('robot', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

  held: set[str] = set()

  def on_press(key):
    nonlocal eye_colour
    char = getattr(key, 'char', None)
    if not char:
      return
    char = char.lower()
    if char == 'l':
      for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
          continue
        qpos = data.qpos[model.jnt_qposadr[i]]
        print(f'{model.joint(i).name:42s} {qpos: .3f}')
    elif char == 'i':
      eye_colour = (eye_colour + 1) % len(EYE_COLOURS)
      model.mat_rgba[eye_material, :3] = EYE_COLOURS[eye_colour]
    held.add(char)

  def on_release(key):
    char = getattr(key, 'char', None)
    held.discard(char.lower() if char else None)

  print(HELP)
  listener = keyboard.Listener(on_press=on_press, on_release=on_release)
  listener.start()

  try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
      start_wall, start_sim = time.perf_counter(), data.time
      next_sensors = 0.0
      while viewer.is_running():
        # Step physics until the simulation catches up with the wall clock.
        with viewer.lock():
          dt = model.opt.timestep
          keys = set(held)
          new_twist = np.array(
            [
              BASE_SPEED * (('w' in keys) - ('s' in keys)),
              BASE_SPEED * (('a' in keys) - ('d' in keys)),
              BASE_TURN_SPEED * (('q' in keys) - ('e' in keys)),
            ]
          )
          if not np.array_equal(new_twist, twist):
            twist = new_twist
            base.set_twist(*twist)
          steps = 0
          while data.time - start_sim < time.perf_counter() - start_wall:
            for key in keys:
              if key in JOINT_KEYS:
                name, speed = JOINT_KEYS[key]
                data.ctrl[model.actuator(name).id] += speed * dt
              elif key in GRIPPER_KEYS:
                for finger in FINGERS:
                  i = model.actuator(finger).id
                  low, high = model.actuator_ctrlrange[i]
                  closed = high if open_ctrl[finger] == low else low
                  step = GRIPPER_KEYS[key] * (closed - open_ctrl[finger]) * dt
                  data.ctrl[i] += step
            np.clip(
              data.ctrl,
              *model.actuator_ctrlrange.T,
              out=data.ctrl,
              where=model.actuator_ctrllimited.astype(bool),
            )
            data.ctrl[wheels] = base.step(dt)
            mujoco.mj_step(model, data)
            steps += 1
            if steps > 50:  # Don't spiral if the machine can't keep up.
              start_wall, start_sim = time.perf_counter(), data.time
              break
          take_sensors = data.time >= next_sensors
          if take_sensors:
            next_sensors = data.time + 1 / SENSOR_HZ
            mujoco.mj_copyData(snapshot, model, data)
        viewer.sync()

        # Rendered from a copy, so the viewer isn't locked out meanwhile.
        if take_sensors:
          log_sensors(cameras, lidars, line_sensors, snapshot)
        time.sleep(0.001)
  except KeyboardInterrupt:
    pass
  finally:
    listener.stop()
    for camera in cameras.values():
      camera.close()


if __name__ == '__main__':
  main()
