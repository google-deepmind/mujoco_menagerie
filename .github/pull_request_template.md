# Description

<!-- Briefly describe what this PR does -->

Fixes: <!-- List any GitHub issues this PR addresses -->

# Checklist

If you haven't already, set up pre-commit (one-time):

```bash
uv tool install pre-commit
pre-commit install
```

That auto-runs lint, formatting (Python via ruff + MJCF XML via
`format_xml.py`), trailing-whitespace, and the top-level `LICENSE` check on
every commit. To verify everything against the whole repo, including the
simulation tests:

```bash
pre-commit run --all-files
pre-commit run --hook-stage manual pytest --all-files
```

Please check off each item (`[x]`) once complete, or mark it as `[N/A]` if it doesn't apply:

- [ ] Added your name to `CONTRIBUTORS.md` (alphabetically by first name)
- [ ] Updated `CHANGELOG.md`:
  - [ ] Global changelog (if your change affects the overall repo)
  - [ ] Model-specific changelog (if it affects a specific model only)
- [ ] `pre-commit run --all-files` passes (lint, format, license check)
- [ ] `pre-commit run --hook-stage manual pytest --all-files` passes (simulation + structural tests)
- [ ] Signed the [Contributor License Agreement (CLA)](https://cla.developers.google.com/)

Refer to the [contributing guide](https://github.com/google-deepmind/mujoco_menagerie/blob/main/CONTRIBUTING.md) if you're unsure about any of the steps.
