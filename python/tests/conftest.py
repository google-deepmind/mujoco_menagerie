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
import struct
import zlib

import pytest

from mujoco_menagerie import _archive
from mujoco_menagerie._registry import EntryPoint
from mujoco_menagerie._registry import Robot

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
REGISTRY_PATH = REPO_ROOT / 'python/src/mujoco_menagerie/registry.json'

ROBOT_XML = """<mujoco model="fake">
  <compiler meshdir="assets" texturedir="assets"/>
  <asset>
    <mesh name="cube" file="cube.obj"/>
    <texture name="tex" type="2d" file="tex.png"/>
    <material name="mat" texture="tex"/>
  </asset>
  <worldbody>
    <body name="link" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom type="mesh" mesh="cube" material="mat"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge"/>
  </actuator>
</mujoco>
"""
SCENE_XML = """<mujoco model="fake_scene">
  <include file="fake.xml"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="2 2 0.1"/>
  </worldbody>
</mujoco>
"""
CUBE_OBJ = (
  'v -1 -1 -1\nv 1 -1 -1\nv 1 1 -1\nv -1 1 -1\nv -1 -1 1\nv 1 -1 1\nv 1 1 1\nv -1 1 1\n'
  'f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\n'
  'f 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n'
)


def _png():  # 2x2 RGB
  def chunk(kind, data):
    return (
      len(data).to_bytes(4, 'big')
      + kind
      + data
      + zlib.crc32(kind + data).to_bytes(4, 'big')
    )

  ihdr = struct.pack('>IIBBBBB', 2, 2, 8, 2, 0, 0, 0)
  idat = zlib.compress(
    b'\x00' + b'\xff\x00\x00' * 2 + b'\x00' + b'\x00\xff\x00' * 2
  )
  return (
    b'\x89PNG\r\n\x1a\n'
    + chunk(b'IHDR', ihdr)
    + chunk(b'IDAT', idat)
    + chunk(b'IEND', b'')
  )


TEX_PNG = _png()
SHIPPED = {
  'fake.xml',
  'scene.xml',
  'assets/cube.obj',
  'assets/tex.png',
  'README.md',
  'LICENSE',
}


def write_fake_model(root: pathlib.Path) -> pathlib.Path:
  d = root / 'fake'
  (d / 'assets').mkdir(parents=True)
  (d / 'fake.xml').write_text(ROBOT_XML)
  (d / 'scene.xml').write_text(SCENE_XML)
  (d / 'assets/cube.obj').write_text(CUBE_OBJ)
  (d / 'assets/tex.png').write_bytes(TEX_PNG)
  (d / 'README.md').write_text('# Fake\n')
  (d / 'LICENSE').write_text('MIT\n')
  (d / 'screenshot.png').write_bytes(TEX_PNG)
  return d


def make_robot(
  model_dir: pathlib.Path, archives: pathlib.Path, oid: str = 'a' * 40
) -> Robot:
  files = [model_dir / f for f in SHIPPED]
  asset = f'fake-{oid[:16]}.tar.xz'
  _archive.write_archive(model_dir, files, archives / asset, prefix='fake')
  return Robot(
    name='fake',
    display_name='Fake',
    category='misc',
    license='MIT',
    oid=oid,
    asset=asset,
    sha256=_archive.sha256_file(archives / asset),
    download_size=(archives / asset).stat().st_size,
    installed_size=sum(f.stat().st_size for f in files),
    entry_points=(
      EntryPoint('fake', 'robot', 'fake.xml'),
      EntryPoint('scene', 'scene', 'scene.xml'),
    ),
    default_model='fake',
    default_scene='scene',
  )


@pytest.fixture
def fake_model(tmp_path):
  return write_fake_model(tmp_path / 'src')


@pytest.fixture
def archives(tmp_path):
  (tmp_path / 'archives').mkdir()
  return tmp_path / 'archives'


@pytest.fixture
def robot(fake_model, archives):
  return make_robot(fake_model, archives)


@pytest.fixture
def base_url(archives):
  return archives.as_uri()


@pytest.fixture
def cache_dir(tmp_path):
  return tmp_path / 'cache'


@pytest.fixture
def repo_root():
  if not (REPO_ROOT / 'unitree_g1/scene.xml').is_file():
    pytest.skip('not inside a Menagerie checkout')
  return REPO_ROOT


@pytest.fixture
def registry_path():
  if not REGISTRY_PATH.is_file():
    pytest.skip('registry.json not built; run `make registry`')
  return REGISTRY_PATH
