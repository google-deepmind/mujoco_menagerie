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
from collections.abc import Iterable
from importlib import metadata

from mujoco_menagerie._cache import Cache
from mujoco_menagerie._cache import Progress
from mujoco_menagerie._cache import print_progress
from mujoco_menagerie._errors import AssetCollisionError
from mujoco_menagerie._errors import ChecksumError
from mujoco_menagerie._errors import DownloadError
from mujoco_menagerie._errors import MenagerieError
from mujoco_menagerie._errors import RegistryError
from mujoco_menagerie._errors import UnknownEntryPointError
from mujoco_menagerie._errors import UnknownRobotError
from mujoco_menagerie._registry import EntryPoint
from mujoco_menagerie._registry import Registry
from mujoco_menagerie._registry import Robot
from mujoco_menagerie._registry import bundled

try:
  __version__ = metadata.version('mujoco-menagerie')
except metadata.PackageNotFoundError:
  __version__ = '0.0.0'


def names() -> list[str]:
  """Every robot name, sorted. No I/O."""
  return bundled().names()


def robots(category: str | None = None) -> list[Robot]:
  """Every robot, or those of one category. No I/O."""
  return [
    r for r in map(bundled().get, names()) if category in (None, r.category)
  ]


def get(name: str) -> Robot:
  """Metadata for one robot. No I/O."""
  return bundled().get(name)


def commit() -> str:
  """The Menagerie commit this registry was built from."""
  return bundled().commit


def load(name: str, entry: str | None = None, cache: Cache | None = None):
  """Compiles a robot's entry point; the default scene when `entry` is None."""
  return get(name).model(entry, cache)


def prefetch(
  names_: Iterable[str] | None = None,
  cache: Cache | None = None,
  progress: Progress | None = print_progress,
) -> list[pathlib.Path]:
  """Downloads models ahead of time; all of them when `names_` is None."""
  return [get(n).path(cache, progress) for n in names_ or names()]


def prune(cache: Cache | None = None) -> list[pathlib.Path]:
  """Deletes cached models this version no longer references."""
  return (cache or Cache()).prune(robots())
