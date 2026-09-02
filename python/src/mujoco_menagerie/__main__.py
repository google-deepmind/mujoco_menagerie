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

from __future__ import annotations

import argparse
import sys

import mujoco_menagerie as mm


def main(argv: list[str] | None = None) -> None:
  p = argparse.ArgumentParser(prog='mujoco-menagerie')
  sub = p.add_subparsers(dest='cmd', required=True)
  sub.add_parser('names')
  sub.add_parser('info').add_argument('name')
  path = sub.add_parser(
    'path', help='model directory, or an entry point XML (downloads if needed)'
  )
  path.add_argument('name')
  path.add_argument('entry', nargs='?')
  view = sub.add_parser('view', help='open an entry point in the MuJoCo viewer')
  view.add_argument('name')
  view.add_argument('entry', nargs='?')
  sub.add_parser('prefetch').add_argument(
    'names', nargs='*', help='default: every robot'
  )
  sub.add_parser(
    'prune', help='delete cached models this version no longer references'
  )
  a = p.parse_args(argv)
  try:
    if a.cmd == 'names':
      print(*mm.names(), sep='\n')
    elif a.cmd == 'info':
      r = mm.get(a.name)
      for k, v in vars(r).items():
        print(
          f'{k:16}', ', '.join(e.name for e in v) if k == 'entry_points' else v
        )
      print(f'{"cached":16}', mm.Cache().is_cached(r))
    elif a.cmd == 'path':
      r = mm.get(a.name)
      print(r.xml(a.entry) if a.entry else r.path())
    elif a.cmd == 'view':
      import mujoco.viewer

      mujoco.viewer.launch_from_path(str(mm.get(a.name).xml(a.entry)))
    elif a.cmd == 'prefetch':
      mm.prefetch(a.names)
    elif a.cmd == 'prune':
      print(*mm.prune(), sep='\n')
  except mm.MenagerieError as e:
    sys.exit(f'error: {e}')


if __name__ == '__main__':
  main()
