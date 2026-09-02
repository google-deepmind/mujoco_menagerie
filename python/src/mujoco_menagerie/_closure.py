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

import pathlib
import xml.etree.ElementTree as ET
from collections.abc import Iterable

from mujoco_menagerie._errors import AssetCollisionError
from mujoco_menagerie._errors import MenagerieError


def closure(
  entry_xml: pathlib.Path, model_dir: pathlib.Path
) -> list[pathlib.Path]:
  model_dir = pathlib.Path(model_dir).resolve()
  entry_xml = pathlib.Path(entry_xml).resolve()
  roots: dict[pathlib.Path, tuple[ET.Element, pathlib.Path]] = {}
  dirs: dict[
    pathlib.Path, list[pathlib.Path]
  ] = {}  # per main file: meshdir etc.
  pending = [(entry_xml, entry_xml.parent)]
  while pending:
    xml, base = pending.pop()
    if xml in roots:
      continue
    roots[xml] = root, base = ET.parse(xml).getroot(), base
    for c in root.iter('compiler'):
      dirs.setdefault(base, []).extend(
        base / c.get(k)
        for k in ('meshdir', 'texturedir', 'assetdir')
        if c.get(k)
      )
    for el in root.iter():
      if el.tag == 'include':
        pending.append(
          (_find(el.get('file'), [base, xml.parent], model_dir), base)
        )
      elif el.tag == 'model':  # attached submodel: its own main file
        sub = _find(el.get('file'), [base, xml.parent], model_dir)
        pending.append((sub, sub.parent))
  files = set(roots)
  for xml, (root, base) in roots.items():
    for el in root.iter():
      for attr, value in el.attrib.items():
        if (
          attr.startswith('file')
          and value
          and el.tag not in ('include', 'model')
        ):
          files.add(
            _find(value, [*dirs.get(base, []), base, xml.parent], model_dir)
          )
  return sorted(files)


def _find(
  value: str, bases: list[pathlib.Path], model_dir: pathlib.Path
) -> pathlib.Path:
  path = next((b / value for b in bases if (b / value).is_file()), None)
  if path is None:
    raise MenagerieError(f'cannot find {value!r} under {model_dir}')
  path = path.resolve()
  if not path.is_relative_to(model_dir):
    raise MenagerieError(f'{value!r} lies outside {model_dir}')
  return path


def assets_dict(
  files: Iterable[pathlib.Path], model_dir: pathlib.Path
) -> dict[str, bytes]:
  model_dir = pathlib.Path(model_dir).resolve()
  seen: dict[str, str] = {}
  out = {}
  for f in sorted(files):
    rel = f.relative_to(model_dir).as_posix()
    if (dup := seen.setdefault(f.name.casefold(), rel)) != rel:
      raise AssetCollisionError(
        f'{rel!r} and {dup!r} share a basename, which MuJoCo rejects in an '
        'assets dict; load this model from disk with model() or spec()'
      )
    out[rel] = f.read_bytes()
  return out
