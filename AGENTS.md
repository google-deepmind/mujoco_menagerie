# AGENTS.md — MuJoCo Menagerie Guide

This file provides instructions for AI coding agents working in `google-deepmind/mujoco_menagerie`. Read [CONTRIBUTING.md](CONTRIBUTING.md) alongside this file.

## Repository Overview

MuJoCo Menagerie is a collection of curated MJCF robot descriptions for [MuJoCo](https://github.com/google-deepmind/mujoco), shipped both as model directories at the repository root and as the `mujoco-menagerie` Python package (`python/`, see [python/DOC.md](python/DOC.md)). Every model directory is discovered automatically by the tests and by `build_registry.py`; the only manual registration points are `catalog.py`, `CONTRIBUTORS.md`, and the top-level `LICENSE`.

### Contribution Workflow

1. **Open an issue first** proposing the new model or fix (per [CONTRIBUTING.md](CONTRIBUTING.md)), then open a PR from a fork that references it with `Fixes: #<issue>` (the PR template's checklist is filled by the author, not copied verbatim).
2. **Never modify an existing model in place in a way that breaks downstream users** (renaming bodies, joints, actuators, sites, or keyframes). Ship a new versioned directory instead (e.g. `franka_fr3_v2`, `robotiq_2f85_v4`) and leave the old one untouched.
3. **Do not hand-edit generated files**: the top-level `LICENSE` (use `regenerate_license.py`), `python/src/mujoco_menagerie/registry.json` (built by `make registry`), or the gallery table between `<!-- BEGIN MODELS -->` / `<!-- END MODELS -->` in the root `README.md` and the thumbnails in `assets/` (built by `make gallery`; maintainers typically regenerate these after merge, so do not include them in a model PR unless asked).
4. Pre-commit hooks (`trailing-whitespace`, `end-of-file-fixer`, `mixed-line-ending`) apply to **all** files, including Markdown, so keep `README.md` / `CHANGELOG.md` free of trailing whitespace and end them with a single newline.

### Key Commands (requires [`uv`](https://docs.astral.sh/uv/))

```bash
make install      # One-time setup: installs pre-commit and git hooks
make check        # Fast checks: ruff lint/format, MJCF XML formatting, top-level LICENSE check
make test         # Slow checks: pytest over all models (simulation + structural layout)
make python-test  # Builds python/src/mujoco_menagerie/registry.json and runs package tests
make all          # Runs check + test + python-test (everything CI runs)
```

Useful single-target commands during iteration:

```bash
# Format MJCF XML files in-place:
uv run format_xml.py --write <model_dir>/*.xml

# Regenerate the concatenated top-level LICENSE after adding/updating <model_dir>/LICENSE:
uv run regenerate_license.py

# Run tests for a single model directory only (same environment as the pre-commit `pytest` hook):
uv run --with-requirements test/requirements.txt pytest test/ -k "<model_dir> or ContributorsTest" -v

# Interactively inspect a scene (needs a display / GPU):
uv run --with mujoco python -m mujoco.viewer --mjcf <model_dir>/scene.xml
```

---

## Adding a New Robot Model

### 1. Directory & Naming Conventions

- Directory name: `<vendor>_<model>` in lowercase `snake_case` (e.g., `franka_fr3_v2`, `universal_robots_ur5e`, `agilex_piper`). Append `_v<N>` for a new revision of an existing model.
- Required files in `<model_dir>/` (enforced by `test/model_dir_test.py`):
  - `<model>.xml` — Standalone robot MJCF (no floor, skybox, `<visual>` settings, or global lights).
  - `scene.xml` — Simulation scene that `<include file="<model>.xml"/>` and adds a groundplane, skybox, lighting, and camera `<statistic>` / `<global>` framing. Directories with several variants ship `scene_<variant>.xml` files instead and declare the default in `catalog.PREVIEW_OVERRIDES`. Models that are not meant to be simulated standalone (e.g. the `realsense_d435i` sensor) are exempted via `_NO_SCENE_REQUIRED` in `test/model_dir_test.py`.
  - `<model>.png` — Hero screenshot of `scene.xml` rendered with MuJoCo's Filament renderer (referenced in `README.md` with `<img src="<model>.png" width="400">`).
  - `assets/` — Visual (`.obj`) and collision (`.stl` or `.obj`) mesh files.
  - `README.md` — Model overview, screenshot, URDF → MJCF derivation steps, and license notice.
  - `CHANGELOG.md` — Release history using dated headings (`## [YYYY-MM-DD]`, **not** `## Unreleased`).
  - `LICENSE` — Exact filename `LICENSE` (never `LICENSE.txt` or `LICENSE.md`) containing the full upstream license text. `catalog.detect_license` recognises Apache-2.0, BSD-2/3-Clause(-Clear), and MIT from this text.

### 2. Standard URDF → MJCF Conversion Workflow

Do **not** manually transcribe link masses, inertia tensors, or body transforms from a URDF into XML by hand. Instead, use MuJoCo's built-in URDF compiler:

1. **Expand xacro (if applicable)**:
   If upstream provides `.urdf.xacro`, expand it to a plain `.urdf` file first (`xacro robot.urdf.xacro > robot.urdf`).
2. **Convert visual meshes to `.obj`**:
   - MuJoCo's URDF compiler supports `.stl` and `.obj` meshes, not `.dae` (COLLADA). If the URDF references `.dae` visual meshes, convert them to `.obj` first (using Blender or trimesh, ensuring `+Z` up and `+Y` forward as appropriate).
   - Because each `.obj` mesh in MuJoCo is assigned a single `<material>`, split multi-material `.obj` files by material using [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. **Add `<mujoco>` compiler hints to the URDF**:
   Insert a `<mujoco>` block inside the URDF's `<robot>` clause so MuJoCo preserves visual geometries and balances inertias:
   ```xml
   <mujoco>
     <compiler meshdir="assets" discardvisual="false" balanceinertia="true" strippath="true"/>
   </mujoco>
   ```
4. **Compile the URDF in MuJoCo and save the MJCF**:
   Load the URDF and save the compiled MJCF via Python or the MuJoCo `compile` CLI:
   ```python
   import mujoco
   model = mujoco.MjModel.from_xml_path("robot.urdf")
   mujoco.mj_saveLastXML("robot.xml", model)
   ```
   This automatically computes canonical body frames, joint axes/ranges, and principal-axis diagonal inertias (`<inertial pos="..." quat="..." mass="..." diaginertia="..."/>`). The saved XML is flat (no default classes) and uses the URDF link/joint names — keep those names, but restructure the file as described next.

### 3. Curating the MJCF (`<model>.xml`)

After generating the base XML from the URDF compiler, refactor it to match Menagerie conventions:

- **Header**: `<mujoco model="<model>">` followed by `<compiler angle="radian" meshdir="assets" autolimits="true"/>` and, if needed, `<option integrator="implicitfast"/>` (recommended for position-controlled arms so joint damping is integrated implicitly).
- **Element ordering**: `<compiler>`, `<option>`, `<default>`, `<asset>`, `<worldbody>`, `<contact>`, `<tendon>` / `<equality>`, `<actuator>`, `<sensor>`, `<keyframe>`.
- **Defaults (`<default>`)**:
  - Wrap all model defaults inside a top-level `<default class="<model>">`, and set `childclass="<model>"` on the root `<body>` in `<worldbody>`.
  - Define `visual` and `collision` sub-classes:
    ```xml
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom type="mesh" group="3"/>
    </default>
    ```
  - Extract shared `<joint>`, `<material>`, `<site>`, and actuator (`<position>` / `<general>`) attributes into default classes (e.g., joint subgroups like `wrist` or `finger`) rather than repeating attributes on every element.
- **Collision geometries**:
  - Prefer simple primitives (`capsule`, `cylinder`, `box`, `sphere`) where they accurately approximate link geometry, or simplified collision meshes in `group="3"`.
- **Contact exclusions (`<contact>`)**:
  - MuJoCo automatically filters contacts between directly adjacent parent–child bodies (`filterparent="enable"` by default). **Do not add `<exclude>` entries for directly adjacent parent–child links.**
  - Only add `<exclude body1="..." body2="..."/>` for **non-adjacent** bodies whose collision geometries legitimately overlap or touch within joint limits (e.g., base-to-shoulder or upper-arm-to-wrist at extreme elbow flexion).
- **Actuators & Keyframes**:
  - Add actuators (typically position actuators with `inheritrange="1"` or explicit `ctrlrange` matching joint limits, and `forcerange` matching URDF effort limits).
  - Add a `home` `<keyframe>` (`<key name="home" qpos="..." ctrl="..."/>`) in a natural, collision-free posture.
  - For arms, add an `attachment_site` `<site>` on the tool flange body (with `+Z` pointing out of the flange) for attaching grippers.
- **XML Formatting**:
  - Always run `uv run format_xml.py --write <model_dir>/*.xml`. `format_xml.py` enforces 2-space indentation, double quotes, `<self-closing/>` tags without spaces, and 120-column attribute wrapping.

### 4. `scene.xml`, `README.md`, and `CHANGELOG.md`

- **`scene.xml`**:
  Include `<model>.xml`, set `<visual><global azimuth="..." elevation="..."/></visual>` (and `<statistic center="..." extent="..."/>` if the auto-framing is off) so the default free camera frames the robot cleanly, and add a textured groundplane and skybox. The canonical template used across the repo is:
  ```xml
  <mujoco model="<model> scene">
    <include file="<model>.xml"/>

    <visual>
      <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
      <rgba haze="0.15 0.25 0.35 1"/>
      <global azimuth="120" elevation="-20"/>
    </visual>

    <asset>
      <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
      <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
        markrgb="0.8 0.8 0.8" width="300" height="300"/>
      <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    </asset>

    <worldbody>
      <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
      <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    </worldbody>
  </mujoco>
  ```
- **`README.md`**:
  - First line must be `# <Human-Readable Robot Name> Description (MJCF)` (`catalog.display_name` strips the `Description (MJCF)` suffix from this heading to derive the gallery/package display name unless overridden in `DISPLAY_NAME_OVERRIDE`).
  - Document the exact `## URDF → MJCF derivation steps` (including the upstream repository link, tag/commit hash, `<mujoco><compiler .../></mujoco>` clause, mesh processing, and actuator/scene additions), and state explicitly when joint `damping` / `armature` / `frictionloss` and actuator gains are **not** system-identified.
  - Link to `[LICENSE](LICENSE)` (not `LICENSE.txt`).
- **`CHANGELOG.md`**:
  Use the release date heading (not `## Unreleased`):
  ```markdown
  # Changelog – <Robot Name> Description

  All notable changes to this model will be documented in this file.

  ## [YYYY-MM-DD]
  - Initial release.
  ```

### 5. Repository Registration & Verification Checklist

Before committing or opening a PR, complete all of the following:

1. **`catalog.py`**: Add `'<model_dir>/<xml_stem>': ModelType.<TYPE>` to `MODEL_MAP` (one entry per standalone robot XML; e.g. an arm directory that also ships a standalone gripper gets two entries). Add a `PREVIEW_OVERRIDES` entry if the directory has multiple `scene*.xml` files instead of a single `scene.xml`, and a `DISPLAY_NAME_OVERRIDE` entry only if the README heading does not produce the desired name.
2. **`CONTRIBUTORS.md`**: Add the contributor's name in alphabetical order by first name (verified by `ContributorsTest::test_sorted`, which sorts case-insensitively).
3. **Top-level `LICENSE`**: Run `uv run regenerate_license.py` so the root `LICENSE` includes `<model_dir>/LICENSE`.
4. **`<model_dir>/CHANGELOG.md`**: Dated `## [YYYY-MM-DD]` entry (model-local; the root `CHANGELOG.md` is only for repo-wide tooling changes).
5. **Format & Test**: Run `make all` (`make check`, `make test`, `make python-test`) and verify zero warnings or failures. `test/model_test.py` steps every `scene*.xml` for 0.1 s under pseudorandom control inputs and fails on any MuJoCo simulation warning (`mjWARN_*`), so unstable gains or interpenetrating collision geoms at the initial pose will fail CI. `make python-test` confirms the new directory is picked up by `build_registry.py` with the expected entry points.
6. **Coherence-check the physics beyond CI**: hold the `home` keyframe for several seconds with no contacts or warnings, and step random in-range joint targets to confirm the actuators are stable (CI only covers 0.1 s).
