# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Lint that every model directory ships the expected files."""

import pathlib

from absl.testing import absltest
from absl.testing import parameterized

# Internal import.


_ROOT_DIR = pathlib.Path(__file__).parent.parent

# Non-model top-level directories.
_SKIP_DIRS = {'assets', 'test', '.git', '.github', 'opensource'}

# Model dirs that legitimately do not need a scene*.xml (e.g. sensors, not
# robots intended to be simulated on their own).
_NO_SCENE_REQUIRED = {'realsense_d435i'}


def _model_dirs() -> list[pathlib.Path]:
  dirs = []
  for d in sorted(_ROOT_DIR.iterdir()):
    if not d.is_dir() or d.name.startswith('.') or d.name in _SKIP_DIRS:
      continue
    if not any(d.glob('*.xml')):
      continue
    dirs.append(d)
  return dirs


_MODEL_DIRS = [(d.name, d) for d in _model_dirs()]


class ModelDirLayoutTest(parameterized.TestCase):
  """Checks that each model directory contains the required files."""

  @parameterized.named_parameters(_MODEL_DIRS)
  def test_has_readme(self, model_dir: pathlib.Path) -> None:
    self.assertTrue(
      (model_dir / 'README.md').is_file(),
      f'{model_dir.name}/README.md is missing.',
    )

  @parameterized.named_parameters(_MODEL_DIRS)
  def test_has_license(self, model_dir: pathlib.Path) -> None:
    self.assertTrue(
      (model_dir / 'LICENSE').is_file(),
      f'{model_dir.name}/LICENSE is missing (note: the filename must be '
      'exactly "LICENSE", not "LICENSE.txt" or similar).',
    )

  @parameterized.named_parameters(_MODEL_DIRS)
  def test_has_changelog(self, model_dir: pathlib.Path) -> None:
    self.assertTrue(
      (model_dir / 'CHANGELOG.md').is_file(),
      f'{model_dir.name}/CHANGELOG.md is missing.',
    )

  @parameterized.named_parameters(_MODEL_DIRS)
  def test_has_scene_xml(self, model_dir: pathlib.Path) -> None:
    if model_dir.name in _NO_SCENE_REQUIRED:
      self.skipTest(f'{model_dir.name} is exempt from the scene*.xml rule.')
    self.assertTrue(
      any(model_dir.glob('scene*.xml')),
      f'{model_dir.name} has no scene*.xml.',
    )


class ContributorsTest(absltest.TestCase):
  """Checks CONTRIBUTORS.md sections are sorted alphabetically by first name."""

  def test_sorted(self) -> None:
    contributors = (_ROOT_DIR / 'CONTRIBUTORS.md').read_text().splitlines()

    # Each section is a contiguous block of lines starting with "- ".
    section_start = None
    sections: list[tuple[int, list[str]]] = []
    for i, line in enumerate(contributors):
      if line.startswith('- '):
        if section_start is None:
          section_start = i
      elif section_start is not None:
        sections.append((section_start, contributors[section_start:i]))
        section_start = None
    if section_start is not None:
      sections.append((section_start, contributors[section_start:]))

    for start, lines in sections:
      sorted_lines = sorted(lines, key=str.casefold)
      if lines != sorted_lines:
        first_bad = next(
          (i for i, (a, b) in enumerate(zip(lines, sorted_lines)) if a != b),
          0,
        )
        self.fail(
          f'CONTRIBUTORS.md section starting at line {start + 1} is not '
          f'sorted. First out-of-order entry: {lines[first_bad]!r} '
          f'(expected {sorted_lines[first_bad]!r}).'
        )


if __name__ == '__main__':
  absltest.main()
