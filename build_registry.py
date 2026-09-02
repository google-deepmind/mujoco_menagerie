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

# /// script
# requires-python = ">=3.10"
# dependencies = ["mujoco>=3.2.0"]
# ///
from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / 'python' / 'src')]

import catalog  # noqa: E402
from mujoco_menagerie import _archive  # noqa: E402
from mujoco_menagerie import _closure  # noqa: E402
from mujoco_menagerie._registry import SCHEMA  # noqa: E402

SKIP = {'assets', 'test', 'python'}
DEFAULT_OUTPUT = ROOT / 'python/src/mujoco_menagerie/registry.json'
KEEP = ('README.md', 'LICENSE', 'CHANGELOG.md')


def compile_error(xml: pathlib.Path) -> str | None:
  import mujoco

  try:
    mujoco.MjModel.from_xml_path(str(xml))
  except Exception as e:
    return str(e).splitlines()[0]
  return None


def process(
  name: str, oid: str, archives: pathlib.Path | None
) -> tuple[str, dict]:
  d = ROOT / name
  entry_points = {
    x.stem: {
      'kind': 'scene' if 'scene' in x.stem else 'robot',
      'file': x.name,
    }
    for x in sorted(d.glob('*.xml'))
    if compile_error(x) is None
  }
  if not entry_points:
    sys.exit(f'{name}: no top-level XML compiles')
  files = {
    f
    for ep in entry_points.values()
    for f in _closure.closure(d / ep['file'], d)
  }
  files |= {p.resolve() for p in d.glob('*.patch')} | {
    (d / k).resolve() for k in KEEP if (d / k).is_file()
  }
  with tempfile.TemporaryDirectory() as tmp:
    for f in files:
      dst = pathlib.Path(tmp, name, f.relative_to(d))
      dst.parent.mkdir(parents=True, exist_ok=True)
      shutil.copyfile(f, dst)
    broken = [
      ep['file']
      for ep in entry_points.values()
      if compile_error(pathlib.Path(tmp, name, ep['file']))
    ]
  if broken:
    sys.exit(f'{name}: {broken} do not compile from the file closure')
  asset = f'{name}-{oid[:16]}.tar.xz'
  r = {
    'oid': oid,
    'asset': asset,
    'sha256': None,
    'download_size': None,
    'installed_size': sum(f.stat().st_size for f in files),
    'entry_points': entry_points,
  }
  if archives:
    out = archives / asset
    if not out.exists():
      _archive.write_archive(d, files, out, prefix=name)
    r.update(sha256=_archive.sha256_file(out), download_size=out.stat().st_size)
  return name, r


def defaults(name: str, entry_points: dict) -> dict:
  keys = [k for k in catalog.MODEL_MAP if k.startswith(name + '/')]
  if not keys:
    sys.exit(f'{name}: not registered in catalog.MODEL_MAP')

  def rank(k):
    base = pathlib.Path(catalog.preview_path(k, name)).name
    return (base != 'scene.xml', not base.startswith('scene'), keys.index(k))

  primary = min(keys, key=rank)
  preview = pathlib.Path(catalog.preview_path(primary, name)).stem
  scenes = [n for n, ep in entry_points.items() if ep['kind'] == 'scene']
  if 'scene' in entry_points:
    scene = 'scene'
  elif preview in scenes:
    scene = preview
  elif len(scenes) == 1:
    scene = scenes[0]
  elif scenes:
    sys.exit(
      f'{name}: several scenes {scenes} and no scene.xml; pick one in catalog.PREVIEW_OVERRIDES'
    )
  else:
    scene = None
  model = primary.split('/')[1]
  if scene:
    tree = ET.parse(ROOT / name / entry_points[scene]['file'])
    included = next(
      (pathlib.Path(el.get('file')).stem for el in tree.iter('include')), None
    )
    model = included if included in entry_points else model
  if model not in entry_points:
    sys.exit(f'{name}: default model {model!r} is not an entry point')
  return {
    'display_name': catalog.display_name(primary),
    'category': catalog.MODEL_MAP[primary].name.lower(),
    'license': catalog.detect_license(ROOT / name / 'LICENSE'),
    'default_model': model,
    'default_scene': scene,
  }


def main() -> None:
  ap = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
  )
  ap.add_argument(
    '--output',
    type=lambda p: pathlib.Path(p).resolve(),
    default=DEFAULT_OUTPUT,
  )
  ap.add_argument(
    '--archives',
    type=lambda p: pathlib.Path(p).resolve(),
    default=ROOT / 'python/dist-assets',
  )
  ap.add_argument('--no-archives', action='store_true')
  ap.add_argument('--only', nargs='+', metavar='DIR')
  ap.add_argument('--jobs', type=int, default=os.cpu_count())
  ap.add_argument(
    '--allow-dirty',
    action='store_true',
    help='use HEAD tree ids despite uncommitted model changes',
  )
  a = ap.parse_args()
  if a.only and a.output == DEFAULT_OUTPUT:
    sys.exit('--only needs --output, or it would replace the full registry')
  os.chdir(ROOT)  # catalog reads <dir>/README.md relative to cwd

  def git(*args):
    return subprocess.check_output(['git', *args], text=True).strip()

  oids = {}
  for line in git('ls-tree', 'HEAD').splitlines():
    _, kind, oid, path = line.split(maxsplit=3)
    if (
      kind == 'tree'
      and path not in SKIP
      and not path.startswith('.')
      and any(pathlib.Path(path).glob('*.xml'))
    ):
      oids[path] = oid
  dirs = a.only or list(oids)
  if unknown := set(dirs) - set(oids):
    sys.exit(f'unknown model dirs: {sorted(unknown)}')
  dirty = git('status', '--porcelain', '--', *dirs)
  if dirty and not a.allow_dirty:
    sys.exit(
      f'uncommitted changes in model dirs; commit them or pass --allow-dirty:\n{dirty}'
    )
  archives = None if a.no_archives else a.archives
  if archives:
    archives.mkdir(parents=True, exist_ok=True)

  robots = {}
  with concurrent.futures.ProcessPoolExecutor(a.jobs) as pool:
    for name, r in pool.map(
      process, dirs, [oids[d] for d in dirs], [archives] * len(dirs)
    ):
      robots[name] = {**defaults(name, r['entry_points']), **r}
      size = f'{r["installed_size"] / 2**20:7.1f} MB'
      if r['download_size']:
        size += f' -> {r["download_size"] / 2**20:6.1f} MB'
      print(
        f'{name:28}{len(r["entry_points"]):3} entry points{size}', flush=True
      )
  commit = git('rev-parse', 'HEAD')
  registry = {
    'schema': SCHEMA,
    'menagerie_commit': commit,
    'robots': dict(sorted(robots.items())),
  }
  a.output.write_text(json.dumps(registry, indent=1, sort_keys=True) + '\n')
  print(
    f'wrote {a.output}: {len(robots)} robots at {commit[:12]}{" (dirty)" if dirty else ""}'
  )


if __name__ == '__main__':
  main()
