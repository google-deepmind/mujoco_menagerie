# Changelog – MuJoCo Menagerie

All notable changes to this repository will be documented here.

## [2026-10-24]

- Added the Universal Robots [UR3](universal_robots_ur3/README.md), [UR3e](universal_robots_ur3e/README.md), [UR5](universal_robots_ur5/README.md), [UR7e](universal_robots_ur7e/README.md), [UR8 Long](universal_robots_ur8long/README.md), [UR10](universal_robots_ur10/README.md), [UR10g-1750](universal_robots_ur10g_1750/README.md), [UR12e](universal_robots_ur12e/README.md), [UR15](universal_robots_ur15/README.md), [UR16e](universal_robots_ur16e/README.md), [UR17g-1300](universal_robots_ur17g_1300/README.md), [UR18](universal_robots_ur18/README.md), [UR18g-950](universal_robots_ur18g_950/README.md), [UR20](universal_robots_ur20/README.md) and [UR30](universal_robots_ur30/README.md) from Universal Robots, in the layout of the UR5e and UR10e models. Contribution by @HamidRobotics.

## [2026-09-12]

- Corrected the list of models whose `assets()` raises in `python/DOC.md`: `pal_talos` collides on `base_link.stl`, `sharpa_wave` does not collide on any entry point. Added a test that derives the list from the checkout.

## [2026-09-01]

- Added the `mujoco-menagerie` Python package under `python/`: models download on first use into a content-addressed cache, pinned by package version. `build_registry.py` derives the registry from the checkout; `release.yml` publishes archives and the wheel on a `vYYYY.M.N` tag. Extracted the gallery's catalog tables into `catalog.py`, shared by both tools, and registered seven model directories that were missing from the README gallery.

## [2026-07-04]

- Cleaned up manual `<statistic>` camera overrides across scene XMLs to improve visualizer default framings. Removed arbitrary `center` attributes, only keeping `extent` where finite floor surfaces or heightfields require custom framing bounds.

## [2026-05-19]

- Streamlined the contributor workflow behind a top-level `Makefile`: `make install` for one-time setup, `make check` for lint/format/license/XML, `make test` for the simulation suite, `make all` for everything CI runs. Slimmed the PR template and rewrote the CONTRIBUTING dev-setup section to match. Removed the redundant `check_license.yml` workflow (its job is already covered by the pre-commit workflow).

## [2026-05-18]

- Added `format_xml.py`, an XML formatter for MJCF files (2-space indent, 120-col wrap, single-line attribute values). Wired into pre-commit so the same check runs locally and in CI. Reformatted every existing XML in the repo so the rules are now actually enforced.
- Added pre-commit hooks (ruff lint+format for Python, trailing-whitespace and EOF fixers, license check) with a matching CI workflow. Contributors can install locally with `uv tool install pre-commit && pre-commit install`. Excludes `.patch` and `.ipynb` files.
- Removed the MJX column from the README model tables and dropped the MJX test from CI. MJX-compatible XMLs are still shipped per-model.
- Added a CI lint that every model directory ships a README, LICENSE, CHANGELOG, and scene*.xml, and that `CONTRIBUTORS.md` stays sorted. Backfilled missing changelogs and renamed `rainbow_robotics_rby1/LICENSE.txt` to `LICENSE`.

## [2026-04-07]

- Added [MS-Human-700](ms_human_700/README.md) from LNS Group.

## [2026-03-18]

- Added [Flexiv Rizon4](flexiv_rizon4/README.md) from Flexiv Robotics. Contribution by @ctkuan-flexiv.

## [2025-12-12]

- fix Skydio X2 gear settings for yaw control

## [2025-05-30]

- Added an MJX-tuned version of the G1 model.

## [2025-05-19]

- Added [YAM manipulator](i2rt_yam/README.md) from I2RT Robotics.

## [2025-04-22]

- Adds changelog structure, contributor list, and PR template.

## [2022-09-07]

- Initial release.
