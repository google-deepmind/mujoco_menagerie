# Changelog – SO-ARM100 Description

All notable changes to this model will be documented in this file.

## [2026-10-10]

- Limit positive elbow flexion to 1.45 rad, matching the actuator control
  limit, to avoid shoulder collisions near the previous 1.69 rad endpoint.

## [2026-09-27]

- Fix the `wrist_roll` joint range upper limit to match the source and the actuator `ctrlrange`.

## [2026-09-18]

- Add camera mount PCB board to SO101 XML and move wrist_cam forward so that it's not obstructed by the PCB board.

## [2025-12-18]

- Initial release.
