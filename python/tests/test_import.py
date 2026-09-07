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
import subprocess
import sys

SMOKE = pathlib.Path(__file__).with_name('smoke_test.py')


def _run(tmp_path, *args, check=True):
  env = {
    'PATH': '/usr/bin:/bin',
    'HOME': str(tmp_path),
    'MENAGERIE_CACHE_DIR': str(tmp_path / 'nope'),
  }
  return subprocess.run(
    [sys.executable, *args],
    env=env,
    capture_output=True,
    text=True,
    check=check,
  )


def test_smoke(registry_path, tmp_path):
  out = _run(tmp_path, SMOKE).stdout
  assert 'robots' in out and not (tmp_path / 'nope').exists()


def test_cli(registry_path, tmp_path):
  assert (
    'unitree_g1'
    in _run(tmp_path, '-m', 'mujoco_menagerie', 'names').stdout.split()
  )
  info = _run(tmp_path, '-m', 'mujoco_menagerie', 'info', 'unitree_g1').stdout
  assert 'Unitree G1' in info and 'g1_mjx' in info and 'cached' in info
  bad = _run(
    tmp_path, '-m', 'mujoco_menagerie', 'info', 'unitree_g', check=False
  )
  assert bad.returncode and 'Did you mean' in bad.stderr
