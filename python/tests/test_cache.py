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
import http.server
import multiprocessing
import os
import stat
import sys
import threading

import pytest
from conftest import SHIPPED
from conftest import make_robot

from mujoco_menagerie import Cache
from mujoco_menagerie import ChecksumError
from mujoco_menagerie import DownloadError
from mujoco_menagerie import MenagerieError
from mujoco_menagerie._cache import SENTINEL
from mujoco_menagerie._cache import fcntl


def _files(d):
  return {p.relative_to(d).as_posix() for p in d.rglob('*') if p.is_file()} - {
    SENTINEL
  }


def test_cold_fetch_publishes_complete_dir(robot, base_url, cache_dir):
  cache = Cache(cache_dir, base_url)
  assert not cache.is_cached(robot)
  path = cache.resolve(robot)
  assert path == cache_dir / 'models' / f'fake-{"a" * 16}'
  assert cache.is_cached(robot) and _files(path) == SHIPPED
  assert not any((cache_dir / 'tmp').iterdir())


def test_warm_read_writes_nothing(robot, base_url, cache_dir):
  path = Cache(cache_dir, base_url).resolve(robot)
  before = {p: p.stat().st_mtime_ns for p in cache_dir.rglob('*')}
  assert Cache(cache_dir, 'file:///nonexistent').resolve(robot) == path
  assert {p: p.stat().st_mtime_ns for p in cache_dir.rglob('*')} == before


@pytest.mark.skipif(
  sys.platform == 'win32' or os.geteuid() == 0, reason='needs POSIX permissions'
)
def test_read_only_cache_serves_warm_reads(robot, base_url, cache_dir):
  Cache(cache_dir, base_url).resolve(robot)
  dirs = [cache_dir, *(p for p in cache_dir.rglob('*') if p.is_dir())]
  for d in dirs:
    os.chmod(d, stat.S_IRUSR | stat.S_IXUSR)
  try:
    assert (Cache(cache_dir, base_url).resolve(robot) / 'scene.xml').read_text()
  finally:
    for d in dirs:
      os.chmod(d, stat.S_IRWXU)


def test_checksum_mismatch_publishes_nothing(robot, base_url, cache_dir):
  with pytest.raises(ChecksumError):
    Cache(cache_dir, base_url).resolve(
      dataclasses.replace(robot, sha256='0' * 64)
    )
  assert not list(cache_dir.glob('models/*')) and not list(
    cache_dir.glob('tmp/*')
  )


def test_corrupt_download_is_rejected(robot, archives, base_url, cache_dir):
  data = bytearray((archives / robot.asset).read_bytes())
  data[len(data) // 2] ^= 0xFF
  (archives / robot.asset).write_bytes(data)
  with pytest.raises(ChecksumError):
    Cache(cache_dir, base_url).resolve(robot)


def test_missing_asset(robot, base_url, cache_dir):
  with pytest.raises(DownloadError, match='nope.tar.xz'):
    Cache(cache_dir, base_url).resolve(
      dataclasses.replace(robot, asset='nope.tar.xz')
    )


def test_registry_without_archives_fails_loudly(robot, base_url, cache_dir):
  with pytest.raises(DownloadError, match='MENAGERIE_ROOT'):
    Cache(cache_dir, base_url).resolve(
      dataclasses.replace(robot, sha256=None, download_size=None)
    )


def test_dir_without_sentinel_is_replaced(robot, base_url, cache_dir):
  cache = Cache(cache_dir, base_url)
  target = cache.model_path(robot)
  target.mkdir(parents=True)
  (target / 'garbage').write_text('crash mid-write')
  assert not cache.is_cached(robot)
  assert _files(cache.resolve(robot)) == SHIPPED


def test_root_serves_a_checkout(robot, fake_model, cache_dir):
  cache = Cache(cache_dir, 'file:///nonexistent', root=fake_model.parent)
  assert cache.resolve(robot) == fake_model and not cache_dir.exists()
  with pytest.raises(MenagerieError, match='MENAGERIE_ROOT'):
    cache.resolve(dataclasses.replace(robot, name='absent'))


def test_environment_supplies_defaults(monkeypatch, tmp_path):
  monkeypatch.setenv('MENAGERIE_CACHE_DIR', str(tmp_path / 'c'))
  monkeypatch.setenv('MENAGERIE_BASE_URL', 'https://mirror/')
  monkeypatch.setenv('MENAGERIE_ROOT', str(tmp_path / 'r'))
  cache = Cache()
  assert (cache.dir, cache.base_url, cache.root) == (
    tmp_path / 'c',
    'https://mirror',
    tmp_path / 'r',
  )
  assert Cache(base_url='file:///x').base_url == 'file:///x'


def test_versions_coexist_and_prune(fake_model, archives, base_url, cache_dir):
  v1 = make_robot(fake_model, archives, oid='1' * 40)
  (fake_model / 'README.md').write_text('# v2\n')
  v2 = make_robot(fake_model, archives, oid='2' * 40)
  cache = Cache(cache_dir, base_url)
  p1, p2 = cache.resolve(v1), cache.resolve(v2)
  assert (p1 / 'README.md').read_text() != (p2 / 'README.md').read_text()
  assert cache.prune(keep=[v2]) == [p1]
  assert not p1.exists() and cache.is_cached(v2)


def _resolve(cache_dir, base_url, robot):
  path = Cache(cache_dir, base_url).resolve(robot)
  return str(path), len(_files(path))


def _serve(directory, hits):
  class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
      hits.append(self.path)

  server = http.server.ThreadingHTTPServer(
    ('127.0.0.1', 0), functools.partial(Handler, directory=directory)
  )
  threading.Thread(target=server.serve_forever, daemon=True).start()
  return server, f'http://127.0.0.1:{server.server_port}'


def test_sixteen_cold_processes_download_once(robot, archives, cache_dir):
  hits = []
  server, url = _serve(archives, hits)
  with multiprocessing.get_context('spawn').Pool(16) as pool:
    results = pool.starmap(_resolve, [(cache_dir, url, robot)] * 16)
  server.shutdown()
  server.server_close()
  assert set(results) == {
    (str(cache_dir / 'models' / f'fake-{"a" * 16}'), len(SHIPPED))
  }
  assert len(list(cache_dir.glob('models/*'))) == 1 and not list(
    cache_dir.glob('tmp/*')
  )
  if fcntl is not None:
    assert hits == [f'/{robot.asset}']


def _reader(cache_dir, base_url, robot, n):
  cache = Cache(cache_dir, base_url)
  for _ in range(n):
    assert cache.resolve(robot) == cache.model_path(robot)


def _pruner(cache_dir, base_url, keep, other, n):
  cache = Cache(cache_dir, base_url)
  for _ in range(n):
    cache.resolve(other)
    cache.prune(keep=[keep])


def test_prune_never_exposes_a_partial_model(
  fake_model, archives, base_url, cache_dir
):
  a, b = (
    make_robot(fake_model, archives, '1' * 40),
    make_robot(fake_model, archives, '2' * 40),
  )
  with multiprocessing.get_context('spawn').Pool(5) as pool:
    jobs = [
      pool.apply_async(_reader, (cache_dir, base_url, a, 15)) for _ in range(4)
    ]
    jobs.append(pool.apply_async(_pruner, (cache_dir, base_url, a, b, 30)))
    seen = 0
    while not all(j.ready() for j in jobs):
      for d in cache_dir.glob('models/*'):
        try:
          entries = os.listdir(d)
        except FileNotFoundError:
          continue
        seen += 1
        assert SENTINEL in entries, entries
    for j in jobs:
      j.get()
  assert seen
