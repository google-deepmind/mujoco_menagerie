# AGENTS.md

Guidelines for contributors, reviewers, and coding agents working in `google-deepmind/mujoco_menagerie`. Read [CONTRIBUTING.md](CONTRIBUTING.md) alongside this file.

## Development Workflow

Always use `uv run` or the `Makefile` targets rather than bare `python`:

```bash
make install      # One-time: install pre-commit and git hook
make check        # Fast: ruff lint/format, MJCF XML formatting, top-level LICENSE
make test         # Slow: pytest simulation + structural checks over all models
make python-test  # Derives registry.json and runs mujoco-menagerie package tests
make gallery      # Maintainers: re-render thumbnails in assets/ and update README.md gallery
make all          # Everything CI runs (check + test + python-test)
```

Useful single-target commands while iterating:

```bash
# Format MJCF XML files in place:
uv run format_xml.py --write <model_dir>/*.xml

# Regenerate the top-level LICENSE after adding/updating <model_dir>/LICENSE:
uv run regenerate_license.py

# Test a single model directory + CONTRIBUTORS.md sorting:
uv run --with-requirements test/requirements.txt pytest test/ -k "<model_dir> or ContributorsTest" -v

# Visually inspect a scene:
uv run --with mujoco python -m mujoco.viewer --mjcf <model_dir>/scene.xml
```

### Generated Files & Formatting Order

- **Do not hand-edit generated files**: root `LICENSE` (use `regenerate_license.py`), `python/src/mujoco_menagerie/registry.json` (built by `make registry`), or the root `README.md` gallery table (`<!-- BEGIN MODELS -->` ... `<!-- END MODELS -->`) and `assets/*.png` thumbnails (built by `make gallery`).
- **Gallery updates (maintainers only)**: No CI workflow runs `make gallery`. To avoid merge conflicts and cross-GPU thumbnail churn, contributors should **not** commit gallery changes in model PRs; after new models merge to `main`, maintainers run `make gallery` (set `MUJOCO_GL=egl` on headless Linux), revert unrelated modified `assets/*.png` files, verify `git diff README.md` only adds the new rows, and open a PR with the new `assets/<model_dir>-<stem>.png` thumbnails and updated `README.md`.
- **Formatting order gotcha**: Run `format_xml.py --write` and `make check` (specifically `end-of-file-fixer`) **before** `uv run regenerate_license.py`. Fixing a missing trailing newline in `<model_dir>/LICENSE` changes the concatenated root `LICENSE`.

## Commits & PR Hygiene

- **Issue first**: Open an issue before major changes or new models (per [CONTRIBUTING.md](CONTRIBUTING.md)) and reference it with `Fixes: #<issue>`.
- **Fork branches only**: Push branches to your own fork, not directly to `google-deepmind/mujoco_menagerie`.
- **No AI co-authors**: Never add AI assistants (e.g. Claude, Codex) as commit co-authors. The `cla/google` check requires every commit author and co-author to have signed the Google CLA, and AI tools cannot sign it.
- **Keep PRs scoped**: In a model PR, do not modify unrelated model directories, `.gitignore`, or the root `CHANGELOG.md` (use `<model_dir>/CHANGELOG.md` for model-specific changes; root `CHANGELOG.md` is for repo-wide changes only).
- **No breaking in-place renames**: Never rename bodies, joints, actuators, sites, or keyframes in an existing model directory if it could break downstream users. Ship a new versioned directory (e.g. `franka_fr3_v2`, `robotiq_2f85_v4`) instead.
- **Keep `test/model_test.py` model-agnostic**: Never add single-robot test cases to `test/model_test.py`.
- **Explicit UTF-8 in Python tooling**: When editing `*.py` tooling, always pass `encoding='utf-8'` to `read_text()` / `write_text()` so non-UTF-8 locales (e.g. Windows `cp1252`) do not crash on non-ASCII characters in `LICENSE` files.

## Adding or Updating a Robot Model

### 1. Directory Layout (`<vendor>_<model>` in lowercase `snake_case`)

Required files in `<model_dir>/` (checked by `test/model_dir_test.py`):

- `<model>.xml`: Standalone robot MJCF.
- `scene.xml`: Scene that includes `<model>.xml` with floor, skybox, and lighting. If a directory ships multiple `scene_<variant>.xml` files and no `scene.xml`, declare the default in `catalog.PREVIEW_OVERRIDES`.
- `<model>.png`: Unclipped hero screenshot of `scene.xml` rendered with MuJoCo's Filament renderer.
- `assets/`: Visual (`.obj`) and collision (`.stl` or `.obj`) meshes.
- `README.md`: First line must be `# <Human-Readable Name> Description (MJCF)` (`catalog.display_name` strips the suffix unless overridden in `DISPLAY_NAME_OVERRIDE`). Link to `[LICENSE](LICENSE)` (not `LICENSE.txt`).
- `CHANGELOG.md`: Dated headings `## [YYYY-MM-DD]` (never `## Unreleased`). Collapse initial model additions into a single initial-release entry.
- `LICENSE`: Exact filename `LICENSE` (never `LICENSE.txt` or `LICENSE.md`) containing the full upstream license text (`catalog.detect_license` recognizes Apache-2.0, BSD-2/3-Clause(-Clear), and MIT).

### 2. URDF to MJCF Conversion

Do **not** manually transcribe link masses, inertia tensors, or body transforms from a URDF into MJCF:

1. Expand `.urdf.xacro` to a plain `.urdf` if upstream uses xacro.
2. Convert `.dae` (COLLADA) visual meshes to `.obj` (MuJoCo's URDF compiler supports `.stl` and `.obj`, not `.dae`), and split multi-material `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Insert `<mujoco><compiler meshdir="assets" discardvisual="false" balanceinertia="true" strippath="true"/></mujoco>` inside the URDF's `<robot>` block.
4. Load the URDF with `mujoco.MjModel.from_xml_path("robot.urdf")` and export via `mujoco.mj_saveLastXML("robot.xml", model)` to get canonical body frames and diagonal inertias (`diaginertia` + `quat`).
5. Document the exact derivation steps (upstream repo URL and commit/tag) in `<model_dir>/README.md`. Explicitly explain any non-upstream offsets or design choices, and state when joint `damping`/`armature`/`frictionloss` or actuator gains are not system-identified.

### 3. MJCF Conventions & Frequent Pitfalls

- **Element ordering & defaults**: Place `<compiler angle="radian" meshdir="assets" autolimits="true"/>` and `<option>` at the top of `<model>.xml` (order: `compiler`, `option`, `default`, `asset`, `worldbody`, `contact`, `tendon`/`equality`, `actuator`, `sensor`, `keyframe`). Wrap shared defaults in a robot-specific `<default class="<model>">` (not generic `robot`) with `childclass="<model>"` on the root body, plus `visual` (`type="mesh" contype="0" conaffinity="0" group="2"`) and `collision` (`group="3"`) sub-classes.
- **No `<light>` in base robot XMLs**: All non-`scene*` XMLs must define zero `<light>` elements (enforced by `test_no_lights_in_base_xml`). Place simple static `<worldbody>` lights in `scene*.xml` without `spotlight`, `targetbodycom`, or `trackcom`.
- **Scene `<statistic>` tags**: Do not set arbitrary `<statistic center="..."/>` overrides; let MuJoCo auto-frame the view. Only set `extent` for scenes with finite floor boundaries or heightfields, not infinite groundplanes.
- **Collision geometries & mass**: Do not disable self-collisions wholesale. Prefer simple primitives (`capsule`, `cylinder`, `box`, `sphere`) or simplified `group="3"` collision meshes. Ensure collision geoms do not double-count body mass (use explicit `<inertial>` or `mass="0"` where appropriate) and do not dip below the floor (`z=0`) at `qpos0` or `home`.
- **Contact `<exclude>` rules**: MuJoCo's default `filterparent="enable"` filters contacts between a body and its parent **unless the parent is welded to the world**. Do not add `<exclude>` entries between moving parent-child links. Add `<exclude>` for an adjacent pair only when the parent is fixed to the world (e.g. a jointless `base_link` and its first moving link) and their geoms touch within joint limits, or for non-adjacent bodies whose collision geoms legitimately overlap.
- **Actuator force limits**: Put effort limits on `<actuator>` via `forcerange` (with `autolimits="true"`), never legacy `actuatorfrcrange` on `<joint>`.
- **Gravity compensation (`gravcomp`)**: If the real arm has onboard gravity compensation, set `gravcomp="1"` on its link bodies so position actuators do not sag under gravity at `home`.
- **Joint limits, sites & keyframes**:
  - When adjusting a joint range to prevent self-collision, push it right up to the collision boundary rather than trimming arbitrarily.
  - For arms, expose a single `attachment_site` `<site>` on the tool flange (`+Z` pointing out of the flange) and remove redundant duplicate flange sites.
  - Add a collision-free `home` `<keyframe>` and verify its `ctrl` vector matches `<actuator>` ordering.

### 4. Repository Registration

- **`catalog.py`**: Append `'<model_dir>/<xml_stem>': ModelType.<TYPE>` at the bottom of `MODEL_MAP` (one entry per standalone robot XML; dict insertion order controls row order within each gallery section). Pick the `ModelType` carefully — it determines the README gallery section, the default thumbnail camera angle in `generate_gallery.py`'s `VIEW_ANGLES`, and the package `category` in `registry.json`. Add `PREVIEW_OVERRIDES` or `DISPLAY_NAME_OVERRIDE` only when defaults are insufficient.
- **`CONTRIBUTORS.md`**: Add your name once (no duplicates), sorted alphabetically by first name in the appropriate section.
- **Top-level `LICENSE`**: Run `uv run regenerate_license.py` after formatting.

## PR Review Checklist

When reviewing a PR (or self-reviewing before opening one), check both automated gates and common manual catches:

1. **Automated checks**: `make all` passes (`make check`, `make test`, `make python-test`) and `cla/google` passes (no AI co-authors in commit trailers).
2. **Physics beyond the 0.1 s CI test**: Hold the `home` keyframe for several seconds (no contacts, warnings, or gravity sag), sweep random in-range actuator targets (stable gains), and verify floor clearance (`ncon == 0` at `qpos0`/`home`) and expected total mass/inertia (no mass double-counting).
3. **Provenance & MJCF quality**:
   - Derived via MuJoCo's URDF compiler rather than hand-transcribed; non-upstream offsets and non-sysid parameters documented in `README.md`.
   - Zero `<light>` elements in base XMLs; no arbitrary `<statistic center="..."/>` or tracking spotlights in `scene*.xml`.
   - `<compiler>`/`<option>` at top, robot-specific `<default class="...">`, `forcerange` on actuators (not `actuatorfrcrange` on joints), single `attachment_site` (no duplicate flange site), `gravcomp="1"` if hardware supports it.
   - No wholesale self-collision disabling; `<exclude>` only for fixed-base-to-first-link or legitimately overlapping non-adjacent pairs.
4. **Repo hygiene**:
   - No committed gallery edits (`README.md` table, `assets/*.png`), no `registry.json`, no per-model tests in `test/model_test.py`, and no stray `.gitignore` or unrelated-model edits.
   - Model-local `CHANGELOG.md` uses `## [YYYY-MM-DD]` (single initial entry for new models), `CONTRIBUTORS.md` has no duplicates and stays sorted, hero `<model>.png` is unclipped, `catalog.py` appends the right `ModelType` at the bottom of `MODEL_MAP`, and the regenerated root `LICENSE` is included.
