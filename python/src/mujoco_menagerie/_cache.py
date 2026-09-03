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

# A model is published or absent: models/<name>-<oid16>/.complete is the only
# existence test, and publishing is one os.replace from a staging dir on the
# same filesystem.

from __future__ import annotations

import contextlib
import json
import os
import pathlib
import shutil
import sys
import tarfile
import time
import urllib.error
import urllib.request
import uuid
from collections.abc import Callable
from collections.abc import Iterable
from typing import TYPE_CHECKING

from mujoco_menagerie._archive import sha256_file
from mujoco_menagerie._errors import ChecksumError
from mujoco_menagerie._errors import DownloadError
from mujoco_menagerie._errors import MenagerieError

if TYPE_CHECKING:
  from mujoco_menagerie._registry import Robot

try:
  import fcntl
except ImportError:
  fcntl = None

DEFAULT_BASE_URL = (
  'https://github.com/google-deepmind/mujoco_menagerie/releases/download/assets'
)
SENTINEL = '.complete'
Progress = Callable[[str, int, int], None]


def default_cache_dir() -> pathlib.Path:
  """Per-platform cache directory used when MENAGERIE_CACHE_DIR is unset."""
  if sys.platform == 'win32':
    local = (
      os.environ.get('LOCALAPPDATA') or pathlib.Path.home() / 'AppData/Local'
    )
    return pathlib.Path(local) / 'mujoco_menagerie' / 'cache'
  if sys.platform == 'darwin':
    return pathlib.Path.home() / 'Library/Caches/mujoco_menagerie'
  cache = os.environ.get('XDG_CACHE_HOME') or pathlib.Path.home() / '.cache'
  return pathlib.Path(cache) / 'mujoco_menagerie'


def print_progress(name: str, done: int, total: int) -> None:
  """Draws a download progress bar on stderr when it is a terminal."""
  if not sys.stderr.isatty():
    return
  bar = '#' * (30 * done // total) if total else ''
  sys.stderr.write(
    f'\r{name:24} [{bar:30}] {done / 2**20:5.1f}/{total / 2**20:.1f} MB'
    + ('\n' if total and done >= total else '')
  )


class Cache:
  """Downloads model archives into a content-addressed directory tree.




  Arguments default to MENAGERIE_CACHE_DIR, MENAGERIE_BASE_URL and MENAGERIE_ROOT.


  """

  def __init__(
    self,
    dir: pathlib.Path | str | None = None,
    base_url: str | None = None,
    root: pathlib.Path | str | None = None,
  ):
    self.dir = pathlib.Path(
      dir or os.environ.get('MENAGERIE_CACHE_DIR') or default_cache_dir()
    )
    self.base_url = (
      base_url or os.environ.get('MENAGERIE_BASE_URL') or DEFAULT_BASE_URL
    ).rstrip('/')
    root = root or os.environ.get('MENAGERIE_ROOT')
    self.root = pathlib.Path(root) if root else None

  def model_path(self, robot: Robot) -> pathlib.Path:
    """Where `robot` lives once published; does not imply it is present."""
    return self.dir / 'models' / f'{robot.name}-{robot.oid[:16]}'

  def is_cached(self, robot: Robot) -> bool:
    """Whether `robot` is published in the cache."""
    return (self.model_path(robot) / SENTINEL).is_file()

  def resolve(
    self, robot: Robot, progress: Progress | None = print_progress
  ) -> pathlib.Path:
    """The model directory, downloading and publishing it first if needed."""
    if self.root:
      path = self.root / robot.name
      if not path.is_dir():
        raise MenagerieError(
          f'{robot.name!r} not found under MENAGERIE_ROOT={self.root}'
        )
      return path
    target = self.model_path(robot)
    if (target / SENTINEL).is_file():
      return target
    if robot.sha256 is None:
      raise DownloadError(
        f'the registry has no archive for {robot.name!r}; set MENAGERIE_ROOT or rebuild it with archives'
      )
    try:
      import lzma  # noqa: F401
    except ImportError:
      raise MenagerieError(
        'this Python was built without lzma support, which the model archives need; reinstall it with xz'
      ) from None
    with self._lock(target.name):
      if not (target / SENTINEL).is_file():
        self._publish(robot, target, progress)
    if not (target / SENTINEL).is_file():
      raise MenagerieError(f'{target} vanished after publishing')
    return target

  def _publish(
    self, robot: Robot, target: pathlib.Path, progress: Progress | None
  ) -> None:
    stage = self.dir / 'tmp' / uuid.uuid4().hex
    stage.mkdir(parents=True)
    try:
      archive = stage / robot.asset
      _download(f'{self.base_url}/{robot.asset}', archive, robot.name, progress)
      if (
        archive.stat().st_size != robot.download_size
        or sha256_file(archive) != robot.sha256
      ):
        raise ChecksumError(
          f'{robot.asset} does not match the digest in the registry'
        )
      with tarfile.open(archive) as tf:
        tf.extractall(stage, filter='data')
      tree = stage / robot.name
      if not tree.is_dir():
        raise DownloadError(f'{robot.asset} has no top-level {robot.name}/')
      files = {
        p.relative_to(tree).as_posix(): sha256_file(p)
        for p in sorted(tree.rglob('*'))
        if p.is_file()
      }
      (tree / SENTINEL).write_text(
        json.dumps({'oid': robot.oid, 'files': files})
      )
      _fsync_tree(tree)
      target.parent.mkdir(parents=True, exist_ok=True)
      with contextlib.suppress(OSError):  # lost a race to another process
        if target.is_dir() and not (target / SENTINEL).is_file():
          os.replace(target, stage / '.stale')  # left behind by a crash
        os.replace(tree, target)
        _fsync(target.parent)
    finally:
      shutil.rmtree(stage, ignore_errors=True)

  def verify(self, robot: Robot) -> list[str]:
    """Files missing or changed since the model was published."""
    target = self.model_path(robot)
    files = json.loads((target / SENTINEL).read_text())['files']
    return [
      f
      for f, h in files.items()
      if not (target / f).is_file() or sha256_file(target / f) != h
    ]

  def prune(self, keep: Iterable[Robot]) -> list[pathlib.Path]:
    """Deletes models not in `keep` and staging dirs idle for an hour.

    Like uninstalling a package: not safe for models other processes are using.
    """
    keep = {self.model_path(r).name for r in keep}
    tmp = self.dir / 'tmp'
    removed = []
    for d in sorted((self.dir / 'models').glob('*')):
      if d.name not in keep:
        tmp.mkdir(parents=True, exist_ok=True)
        with contextlib.suppress(FileNotFoundError):  # another prune got it
          os.replace(d, retired := tmp / uuid.uuid4().hex)
          shutil.rmtree(retired, ignore_errors=True)
          removed.append(d)
    for d in tmp.glob('*'):
      with contextlib.suppress(FileNotFoundError):
        newest = max(p.stat().st_mtime for p in (d, *d.iterdir()))
        if time.time() - newest > 3600:
          shutil.rmtree(d, ignore_errors=True)
    return removed

  @contextlib.contextmanager
  def _lock(self, name: str):
    # Best effort, never relied on. flock, not lockf: lockf is released by any close().
    if fcntl is None:
      yield
      return
    try:
      (self.dir / 'locks').mkdir(parents=True, exist_ok=True)
      fd = os.open(self.dir / 'locks' / name, os.O_RDWR | os.O_CREAT)
    except OSError:  # read-only or lockless filesystem
      yield
      return
    try:
      with contextlib.suppress(OSError):
        fcntl.flock(fd, fcntl.LOCK_EX)
      yield
    finally:
      os.close(fd)


def _download(
  url: str, dest: pathlib.Path, name: str, progress: Progress | None
) -> None:
  for attempt in range(3):
    try:
      req = urllib.request.Request(
        url, headers={'User-Agent': 'mujoco-menagerie'}
      )
      with (
        urllib.request.urlopen(req, timeout=60) as resp,
        open(dest, 'wb') as out,
      ):
        total, done = int(resp.headers.get('Content-Length', 0)), 0
        while chunk := resp.read(1 << 20):
          out.write(chunk)
          done += len(chunk)
          if progress:
            progress(name, done, total)
      return
    except urllib.error.HTTPError as e:
      if e.code < 500 and e.code != 429:
        raise DownloadError(f'{url}: HTTP {e.code}') from e
      err = e
    except OSError as e:
      if isinstance(getattr(e, 'reason', None), FileNotFoundError):
        raise DownloadError(f'{url}: not found') from e
      err = e
    time.sleep(2**attempt)
  raise DownloadError(f'{url}: {err}') from err


def _fsync(path: pathlib.Path | str) -> None:
  with contextlib.suppress(OSError):  # directories cannot be opened on Windows
    fd = os.open(path, os.O_RDONLY)
    try:
      os.fsync(fd)
    finally:
      os.close(fd)


def _fsync_tree(root: pathlib.Path) -> None:
  for dirpath, _, files in os.walk(root):
    for f in files:
      _fsync(pathlib.Path(dirpath, f))
    _fsync(dirpath)
