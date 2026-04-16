# Changelog – Robotiq 2F-85 Description

All notable changes to this model will be documented in this file.

## [2026-04-13]
- Fix base collision geom alignment (missing `pos` and `quat` attributes).
- Fix pad geom masses: set each pad geom to 1.75 g so each pad body totals 3.5 g, matching the original `robotiq_2f85` model.

## [2024-12-02]
- Initial release.
