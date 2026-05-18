# Changelog – MuJoCo Menagerie

All notable changes to this repository will be documented here.

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
