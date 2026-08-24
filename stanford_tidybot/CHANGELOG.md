# Changelog – Stanford TidyBot Description

All notable changes to this model will be documented in this file.

## [2026-08-24]
- Fix `base_link` mass and inertia in `base.xml`, matching the fix already applied to
  `tidybot.xml` in #281 (fixes #231): drop the placeholder `diaginertia="0.001 0.001 0.001"`
  and derive the base inertia from the body mesh geometry instead.

## [2024-09-09]
- Initial release.
