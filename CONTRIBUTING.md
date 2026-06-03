# Contributing to Menagerie

We want Menagerie to be a true community-driven effort that continuously
improves and grows over time for the benefit of the entire research community.
As such, we welcome contributions that:

- Fix issues with an existing model
- Improve the realism of a model (e.g. via
  [system identification](https://en.wikipedia.org/wiki/System_identification))
- Add an entirely new model

Note that Menagerie follows [Google's Open Source Community Guidelines](https://opensource.google/conduct/).

## How to contribute

Whether you want to fix an issue with an existing model, improve it, or add a
completely new model, please get in touch with us first (ideally _before_
starting work if it's something major) by opening a new
[issue](https://github.com/google-deepmind/mujoco_menagerie/issues).
Coordinating up front makes it much easier to avoid frustration later on.

Once we reach an agreement on the proposed change, please submit a
[pull request](https://github.com/google-deepmind/mujoco_menagerie/pulls) (PR)
so that we can review your implementation.

## XML Style

You can browse existing models to get a general sense of the style we adopt for
our MJCF (XML) files. In no particular order, we try to adhere to the following
guidelines:

- Use 2-space indentation
- Make generous use of default classes to reduce redundancies in the kinematic
  tree
- Preserve attribute ordering: compiler, asset and default class definitions
  first, then worldbody and actuators, etc.
- Always have a `scene.xml` that includes the model

XML formatting is enforced by `format_xml.py` at the repo root, which the
pre-commit hook runs in `--check` mode. Run it locally before committing:

```bash
uv run format_xml.py --write path/to/file.xml ...   # rewrite in place
uv run format_xml.py --check path/to/file.xml ...   # exit 1 if not formatted
```

The script enforces:

- 2-space indentation
- Double-quoted attribute values
- Self-closing empty elements as `<foo/>` (no space before the slash)
- Lines wrap at 120 characters; overflow attributes continue at `(depth + 1) *
  2` spaces of indent
- Blank lines between sibling elements are preserved
- Multi-line attribute values are collapsed to a single line (XML attribute
  value normalization makes them un-recoverable after parsing anyway)

If you prefer formatting in VS Code, the
[XML Language Support by Red Hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
extension produces output that is close to (but not always byte-identical to)
`format_xml.py`. Use the script as the source of truth — its output is what
CI checks.

## Pre-commit hooks

All linting, formatting, and license checks are wrapped in
[pre-commit](https://pre-commit.com/) so the same gates run locally and in CI.
Set it up once:

```bash
uv tool install pre-commit
pre-commit install              # auto-run on every git commit
```

From then on, the hooks fire on every commit. To run everything against the
whole repo (e.g. after a rebase):

```bash
pre-commit run --all-files
```

To also run the slow pytest suite (model simulation tests and the structural
lint over every model directory):

```bash
pre-commit run --hook-stage manual pytest --all-files
```

`model_test.py` simply simulates each robot for a fixed duration of time and
checks that no simulation instabilities occur. In the future, we will likely add
more tests that check for model realism (e.g., that a trajectory in real matches
one in simulation).

## Changelog & Contributors

Please document your changes in the appropriate changelog:

- For updates that affect the general repository (e.g., CI, tooling, documentation, shared infrastructure), add an entry to the [global `CHANGELOG.md`](./CHANGELOG.md).
- For changes specific to a model, update the `CHANGELOG.md` in that model’s directory (e.g., `unitree_go1/CHANGELOG.md`).

Make sure to also add your name to the [`CONTRIBUTORS.md`](./CONTRIBUTORS.md), keeping the list sorted alphabetically by first name.

## Contributor License Agreement

Contributions to this project must be accompanied by a Contributor License
Agreement (CLA). You (or your employer) retain the copyright to your
contribution; this simply gives us permission to use and redistribute your
contributions as part of the project. Head over to <https://cla.developers.google.com/>
to see your current agreements on file or to sign a new one.

You generally only need to submit a CLA once, so if you've already submitted one
(even if it was for a different project), you probably don't need to do it
again.
