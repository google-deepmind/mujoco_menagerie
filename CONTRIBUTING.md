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

Furthermore, we automatically format our XMLs in [Visual Studio Code](https://code.visualstudio.com/)
using the [XML Language Support by Red Hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
extension. Once installed, you will need to edit its settings as follows:

- View > Command Palette > `Preferences: Open User Settings`
- Search for XML
- `Enforce quote style` → preferred
- `Max line width` → 120
- `Preserve attribute line breaks` → toggle OFF
- `Xml › Format: Space Before Empty Close Tag` → toggle OFF

Once installed, you can format an XML file by opening the command palette and
executing `Format Document`.

## Unit Tests

Before submitting your PR, you can test your change locally by invoking pytest:

```bash
pytest test/
```

This same test will run on GitHub CI once you open your PR. Currently,
`model_test.py` simply simulates each robot for a fixed duration of time and
checks that no simulation instabilities occur. In the future, we will likely add
more tests that check for model realism (e.g., that a trajectory in real matches
one in simulation).

## Contributor License Agreement

Contributions to this project must be accompanied by a Contributor License
Agreement (CLA). You (or your employer) retain the copyright to your
contribution; this simply gives us permission to use and redistribute your
contributions as part of the project. Head over to <https://cla.developers.google.com/>
to see your current agreements on file or to sign a new one.

You generally only need to submit a CLA once, so if you've already submitted one
(even if it was for a different project), you probably don't need to do it
again.
