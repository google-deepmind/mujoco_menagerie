# Anybotics Anymal C

## MJCF Instructions

The MuJoCo config in `anymal_c_mjx.xml` was copied from https://github.com/google-deepmind/mujoco_menagerie/tree/main/anybotics_anymal_c. The following edits were made to the MJCF specifically for MJX:

* `frictionloss` was removed.
* PD gain was re-tuned.
* A custom `init_qpos` was added.
* The friction cone was changed from elliptic to pyramidal.
* All contacts other than the sphere geoms on the feet were turned off. Its contact dimensionality was changed from 6 to 3.
* Pairwise contacts between feet were turned off to speed up simulation.
* The compiler option was changed to `<option timestep="0.002" iterations="1" solver="Newton"/>`, with `<flag eulerdamp="disable"/>`.
* Some decorative geoms were removed from the torso, to speed up rendering.