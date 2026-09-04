# Changelog – Franka Mobile FR3 Duo

All notable changes to this model will be documented in this file.

## [2026-03-30]
- Added initial Mobile FR3 Duo model (TMR mobile base + spine + head + dual FR3v2 arms).
- Visual meshes for platform parts (TMR base, spine, head, mount, cover) converted from URDF DAE sources with correct per-part coloring.
- TMR base split into 5 color groups (red, black, white, yellow, gray).
- Spine split into 3 color groups (white, dark gray, black).
- Fixed arm material assignments to match DAE ground truth (link0, link5, link6, link7).
- Added missing link5_4 visual geom.
- Replaced MoveIt planar joint workaround (virtual_x/y/theta) with freejoint and actuated swerve drive.
- Argo drive modules: position-controlled steering + velocity-controlled driving.
- Added wheel friction (argo: 1.5, casters: 0.5) and condim=4 for proper ground contact.
- Housing box geoms (argo/caster mounts) set to non-colliding.
