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
"""

import math
from pathlib import Path

import mujoco
import numpy as np

MJCF = Path(__file__).parent / "flexiv_rizon4.xml"

# Tuning parameters.
NATURAL_FREQ_HZ = 5.0  # [Hz]
DAMPING_RATIO = 2.0  # Overdamped to compensate for missing armature.


def compute_gains(
    model_path: Path,
    natural_freq_hz: float = NATURAL_FREQ_HZ,
    damping_ratio: float = DAMPING_RATIO,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Dense mass matrix at qpos0.
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    effective_inertia = np.diag(M)

    w_n = 2.0 * math.pi * natural_freq_hz

    print(f"Natural frequency: {natural_freq_hz} Hz  (w_n = {w_n:.4f} rad/s)")
    print(f"Damping ratio:     {damping_ratio}")
    print()
    print(f"{'joint':<10} {'M_ii':>10} {'kp':>10} {'kd':>10}")
    print("-" * 44)

    for i in range(model.nv):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        m_ii = effective_inertia[i]
        kp = m_ii * w_n**2
        kd = 2.0 * damping_ratio * m_ii * w_n
        print(f"{name:<10} {m_ii:10.6f} {kp:10.4f} {kd:10.4f}")


if __name__ == "__main__":
    compute_gains(MJCF)
