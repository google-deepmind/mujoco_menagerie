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
import shutil
import subprocess
import sys
import zipfile

import pytest

PYTHON_DIR = pathlib.Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def wheel(tmp_path_factory):
  if shutil.which('uv') is None:
    pytest.skip('uv not installed')
  if not (PYTHON_DIR / 'src/mujoco_menagerie/registry.json').is_file():
    pytest.skip('registry.json not built; run `make registry`')
  out = tmp_path_factory.mktemp('dist')
  subprocess.run(
    ['uv', 'build', '--wheel', '--out-dir', out, PYTHON_DIR],
    check=True,
    capture_output=True,
  )
  return next(out.glob('*.whl'))


def test_wheel_contents(wheel):
  assert wheel.stat().st_size < 512 * 1024
  names = zipfile.ZipFile(wheel).namelist()
  assert 'mujoco_menagerie/registry.json' in names
  assert not [
    n
    for n in names
    if '__pycache__' in n or n.endswith('.xz') or n.startswith('tests/')
  ]


def test_wheel_installs_and_imports(wheel, tmp_path):
  venv = tmp_path / 'venv'
  python = venv / (
    'Scripts/python.exe' if sys.platform == 'win32' else 'bin/python'
  )
  subprocess.run(
    ['uv', 'venv', '--python', sys.executable, venv],
    check=True,
    capture_output=True,
  )
  subprocess.run(
    ['uv', 'pip', 'install', '--python', python, wheel],
    check=True,
    capture_output=True,
  )
  code = 'import mujoco_menagerie as mm; print(len(mm.names()), mm.__version__)'
  out = subprocess.run(
    [python, '-c', code], check=True, capture_output=True, text=True, env={}
  ).stdout.split()
  assert int(out[0]) >= 60 and out[1] == '0.0.0'
