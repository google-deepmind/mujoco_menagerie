# Changelog – Ostrich Description

All notable changes to this model will be documented in this file.

## [2026-08-31]

- Initial release, ported from the [OstrichRL](https://github.com/vittorione94/ostrichrl) repository.
- Zeroed the stiffness of the six root joints in `ostrich_legs_torque.xml`; upstream they inherited the default joint stiffness of 10, which springs the body back towards the origin.
