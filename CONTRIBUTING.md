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

## Development setup

Everything is wrapped behind three `make` commands. You need
[`uv`](https://docs.astral.sh/uv/) installed — that's the only prerequisite.

```bash
make install   # one-time: installs pre-commit and the git hook
make all       # run every check CI runs (lint + format + license + XML + tests)
```

After `make install`, the lint/format/license/XML checks fire automatically on
every `git commit`. Before pushing, run `make all` to also execute the (slower)
simulation tests. The two intermediate targets are also exposed if you want
finer control:

| Command       | What it runs                                                  |
| ------------- | ------------------------------------------------------------- |
| `make check`  | Lint + format (ruff), MJCF XML formatting, top-level LICENSE  |
| `make test`   | `pytest` over every model directory (simulation + structural) |
| `make all`    | `make check` followed by `make test`                          |

CI runs the same things, so a green `make all` locally means a green CI.

## XML style

You can browse existing models to get a general sense of the style we adopt for
our MJCF (XML) files. In no particular order:

- 2-space indentation
- Make generous use of default classes to reduce redundancies in the kinematic tree
- Preserve attribute ordering: compiler, asset and default class definitions
  first, then worldbody and actuators, etc.
- Always ship a `scene.xml` that includes the model

Formatting is enforced by `format_xml.py` (called automatically via the
pre-commit hook). The script enforces:

- 2-space indentation
- Double-quoted attribute values
- Self-closing empty elements as `<foo/>` (no space before the slash)
- Lines wrap at 120 characters; overflow attributes continue at `(depth + 1) * 2` spaces of indent
- Blank lines between sibling elements are preserved
- Multi-line attribute values are collapsed to a single line (XML attribute
  value normalization makes them un-recoverable after parsing anyway)

To format on demand:

```bash
uv run format_xml.py --write path/to/file.xml ...   # rewrite in place
uv run format_xml.py --check path/to/file.xml ...   # exit 1 if not formatted
```

The
[XML Language Support by Red Hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
VS Code extension produces output that is close to (but not always
byte-identical to) `format_xml.py`. Use the script as the source of truth — its
output is what CI checks.

## Changelog & contributors

Please document your changes in the appropriate changelog:

- Repo-wide changes (CI, tooling, documentation, shared infrastructure):
  update the [global `CHANGELOG.md`](./CHANGELOG.md)
- Model-specific changes: update the `CHANGELOG.md` in that model's directory
  (e.g. `unitree_go1/CHANGELOG.md`)

Then add your name to [`CONTRIBUTORS.md`](./CONTRIBUTORS.md), keeping the list
sorted alphabetically by first name.

## Contributor License Agreement

Contributions to this project must be accompanied by a Contributor License
Agreement (CLA). You (or your employer) retain the copyright to your
contribution; this simply gives us permission to use and redistribute your
contributions as part of the project. Head over to <https://cla.developers.google.com/>
to see your current agreements on file or to sign a new one.

You generally only need to submit a CLA once, so if you've already submitted
one (even if it was for a different project), you probably don't need to do it
again.
