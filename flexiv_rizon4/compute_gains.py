# /// script
# dependencies = ["mujoco", "numpy"]
# ///
"""Compute PD gains for the Flexiv Rizon4 from effective inertia.

Uses the diagonal of the joint-space mass matrix M as a heuristic for effective
inertia at each DOF, then derives kp/kd from a target natural frequency and
damping ratio:

    kp = M_ii * w_n^2
    kd = 2 * zeta * M_ii * w_n

where w_n = 2 * pi * f_hz.

Natural frequencies are derived per actuator class from a target saturation
angle (the position error at which the actuator reaches its force limit):

    f_hz = (1 / 2pi) * sqrt(F_max / (M_ii_max * delta_theta_sat))

where M_ii_max is the largest effective inertia in the class and F_max is the
actuator force limit.
"""

import math
from pathlib import Path

import mujoco
import numpy as np

MJCF = Path(__file__).parent / "flexiv_rizon4.xml"

SATURATION_ANGLE_DEG = 10.0  # [deg]
DAMPING_RATIO = 1.0  # Critically damped.


def compute_gains(model_path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Dense mass matrix at qpos0.
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    effective_inertia = np.diag(M)

    delta_theta = math.radians(SATURATION_ANGLE_DEG)

    # Group joints by actuator class and find worst-case (largest) M_ii per class.
    classes = {}
    for i in range(model.nu):
        joint_id = model.actuator_trnid[i, 0]
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        frc_max = model.jnt_actfrcrange[joint_id, 1]
        m_ii = effective_inertia[joint_id]
        # Use forcerange as class key.
        key = frc_max
        if key not in classes:
            classes[key] = {"m_ii_max": m_ii, "frc_max": frc_max, "joints": []}
        else:
            classes[key]["m_ii_max"] = max(classes[key]["m_ii_max"], m_ii)
        classes[key]["joints"].append((name, joint_id, m_ii))

    # Derive natural frequency per class from saturation constraint.
    class_freq = {}
    for key, info in classes.items():
        f_hz = math.sqrt(info["frc_max"] / (info["m_ii_max"] * delta_theta)) / (2.0 * math.pi)
        f_hz = round(f_hz * 2) / 2  # Round to nearest 0.5 Hz.
        class_freq[key] = 2.0 * math.pi * f_hz

    print(f"Saturation angle: {SATURATION_ANGLE_DEG} deg")
    print(f"Damping ratio:    {DAMPING_RATIO}")
    print()
    print(f"{'joint':<10} {'f_hz':>6} {'M_ii':>10} {'kp':>10} {'kd':>10}")
    print("-" * 50)

    for info in classes.values():
        w_n = class_freq[info["frc_max"]]
        f_hz = w_n / (2.0 * math.pi)
        for name, joint_id, m_ii in info["joints"]:
            kp = m_ii * w_n**2
            kd = 2.0 * DAMPING_RATIO * m_ii * w_n
            print(f"{name:<10} {f_hz:>6.2f} {m_ii:10.6f} {kp:10.4f} {kd:10.4f}")


if __name__ == "__main__":
    compute_gains(MJCF)
