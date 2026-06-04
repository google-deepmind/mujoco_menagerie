#!/usr/bin/env python3
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

"""Regenerate the top-level LICENSE file from individual model licenses.

The top-level LICENSE file is a concatenation of all individual model LICENSE
files plus the base project license. Run this script whenever a model is added,
removed, or has its license changed.

Usage:
  python regenerate_license.py          # Regenerate LICENSE in-place
  python regenerate_license.py --check  # Check if LICENSE is up to date
"""

import argparse
import pathlib
import sys

HLINE = '=' * 80 + '\n'


def get_base_license(root: pathlib.Path) -> str:
  """Extract the base project license.

  Looks for opensource/LICENSE first (used in internal repos), then falls back
  to extracting the last section from the existing concatenated LICENSE.

  Args:
    root: The root directory of the repository.

  Returns:
    The base project license text.
  """
  opensource_license = root / 'opensource' / 'LICENSE'
  if opensource_license.exists():
    return opensource_license.read_text()

  # Fall back to extracting from the existing concatenated LICENSE.
  existing = root / 'LICENSE'
  if existing.exists():
    sections = existing.read_text().split(HLINE + '\n')
    if sections:
      return sections[-1]

  print('ERROR: Cannot find base license.', file=sys.stderr)
  print(
    'Expected either opensource/LICENSE or an existing top-level LICENSE.',
    file=sys.stderr,
  )
  sys.exit(1)


def generate_license(root: pathlib.Path) -> str:
  """Generate the concatenated LICENSE content.

  Args:
    root: The root directory of the repository.

  Returns:
    The concatenated LICENSE content.
  """
  license_files = sorted(
    root.glob('*/LICENSE'),
    key=lambda f: f.parent.name,
  )
  license_files = [f for f in license_files if f.parent.name != 'opensource']

  base_license = get_base_license(root)

  out = ''
  for lf in license_files:
    out += HLINE
    out += f"License for contents in the directory '{lf.parent.name}/'\n"
    out += HLINE + '\n'
    out += lf.read_text() + '\n\n'

  out += HLINE
  out += 'The following license applies to all other contents\n'
  out += HLINE + '\n'
  out += base_license

  return out


def main():
  parser = argparse.ArgumentParser(
    description='Regenerate the top-level LICENSE file.'
  )
  parser.add_argument(
    '--check',
    action='store_true',
    help='Check if the LICENSE is up to date without modifying it.',
  )
  args = parser.parse_args()

  root = pathlib.Path(__file__).resolve().parent
  generated = generate_license(root)

  license_path = root / 'LICENSE'

  if args.check:
    if not license_path.exists():
      print('FAIL: LICENSE file does not exist.', file=sys.stderr)
      sys.exit(1)

    current = license_path.read_text()
    if current != generated:
      print(
        'FAIL: LICENSE file is out of date. '
        "Run 'python regenerate_license.py' to fix.",
        file=sys.stderr,
      )
      sys.exit(1)

    print('OK: LICENSE file is up to date.')
    return

  license_path.write_text(generated)
  print(f'LICENSE file regenerated at {license_path}')


if __name__ == '__main__':
  main()
