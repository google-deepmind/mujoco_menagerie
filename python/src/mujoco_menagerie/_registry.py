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

import dataclasses
import functools
import json
import pathlib
from importlib import resources
from typing import TYPE_CHECKING

from mujoco_menagerie._cache import Cache
from mujoco_menagerie._cache import Progress
from mujoco_menagerie._cache import print_progress
from mujoco_menagerie._closure import assets_dict
from mujoco_menagerie._closure import closure
from mujoco_menagerie._errors import RegistryError
from mujoco_menagerie._errors import UnknownEntryPointError
from mujoco_menagerie._errors import UnknownRobotError

if TYPE_CHECKING:
  import mujoco

SCHEMA = 1


@dataclasses.dataclass(frozen=True)
class EntryPoint:
  name: str
  kind: str  # 'scene' or 'robot'
  file: str

  @property
  def is_mjx(self) -> bool:
    return 'mjx' in self.name.lower()


@dataclasses.dataclass(frozen=True)
class Robot:
  name: str
  display_name: str
  category: str
  license: str
  oid: str  # git tree id of the model directory
  asset: str  # archive file name under the base URL
  sha256: str | None
  download_size: int | None
  installed_size: int
  entry_points: tuple[EntryPoint, ...]
  default_model: str
  default_scene: str | None

  @property
  def entry_names(self) -> tuple[str, ...]:
    return tuple(e.name for e in self.entry_points)

  def entry(self, name: str | None = None) -> EntryPoint:
    if name is None:
      name = self.default_scene or self.default_model
    for e in self.entry_points:
      if e.name == name:
        return e
    raise UnknownEntryPointError(self.name, name, self.entry_names)

  def path(
    self, cache: Cache | None = None, progress: Progress | None = print_progress
  ) -> pathlib.Path:
    return (cache or Cache()).resolve(self, progress)

  def xml(
    self, entry: str | None = None, cache: Cache | None = None
  ) -> pathlib.Path:
    return self.path(cache) / self.entry(entry).file

  def files(
    self, entry: str | None = None, cache: Cache | None = None
  ) -> list[pathlib.Path]:
    root = self.path(cache)
    return closure(root / self.entry(entry).file, root)

  def assets(
    self, entry: str | None = None, cache: Cache | None = None
  ) -> dict[str, bytes]:
    return assets_dict(self.files(entry, cache), self.path(cache))

  def model(
    self, entry: str | None = None, cache: Cache | None = None
  ) -> mujoco.MjModel:
    import mujoco  # deferred: importing this package must not import MuJoCo

    return mujoco.MjModel.from_xml_path(str(self.xml(entry, cache)))

  def spec(
    self, entry: str | None = None, cache: Cache | None = None
  ) -> mujoco.MjSpec:
    import mujoco

    return mujoco.MjSpec.from_file(str(self.xml(entry, cache)))

  @classmethod
  def from_dict(cls, name: str, d: dict) -> Robot:
    entry_points = tuple(
      EntryPoint(k, **v) for k, v in d['entry_points'].items()
    )
    fields = {k: v for k, v in d.items() if k != 'entry_points'}
    return cls(name=name, entry_points=entry_points, **fields)


@dataclasses.dataclass(frozen=True)
class Registry:
  commit: str
  robots: dict[str, Robot]

  def names(self) -> list[str]:
    return sorted(self.robots)

  def get(self, name: str) -> Robot:
    if name not in self.robots:
      raise UnknownRobotError(name, self.robots)
    return self.robots[name]

  @classmethod
  def from_dict(cls, d: dict) -> Registry:
    if d.get('schema') != SCHEMA:
      raise RegistryError(f'unsupported registry schema {d.get("schema")!r}')
    robots = {n: Robot.from_dict(n, r) for n, r in d['robots'].items()}
    return cls(d['menagerie_commit'], robots)

  @classmethod
  def load(cls, path: pathlib.Path) -> Registry:
    return cls.from_dict(json.loads(pathlib.Path(path).read_text()))


@functools.lru_cache(maxsize=1)
def bundled() -> Registry:
  try:
    text = resources.files(__package__).joinpath('registry.json').read_text()
    return Registry.from_dict(json.loads(text))
  except FileNotFoundError:
    raise RegistryError(
      'registry.json is not bundled; in a checkout run `make registry`'
    ) from None
  except (KeyError, TypeError, ValueError) as e:
    raise RegistryError(f'malformed registry.json: {e}') from e
