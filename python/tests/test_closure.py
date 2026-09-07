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

import shutil

import mujoco
import numpy as np
import pytest

from mujoco_menagerie import AssetCollisionError
from mujoco_menagerie import MenagerieError
from mujoco_menagerie._closure import assets_dict
from mujoco_menagerie._closure import closure


def _rel(files, root):
  return sorted(p.relative_to(root).as_posix() for p in files)


def test_closure_follows_includes_and_compiler_dirs(fake_model):
  assert _rel(closure(fake_model / 'scene.xml', fake_model), fake_model) == [
    'assets/cube.obj', 'assets/tex.png', 'fake.xml', 'scene.xml',
  ]  # fmt: skip
  assert _rel(closure(fake_model / 'fake.xml', fake_model), fake_model) == [
    'assets/cube.obj', 'assets/tex.png', 'fake.xml',
  ]  # fmt: skip


def test_missing_and_escaping_references_raise(fake_model, tmp_path):
  (fake_model / 'assets/cube.obj').unlink()
  with pytest.raises(MenagerieError, match='cube.obj'):
    closure(fake_model / 'scene.xml', fake_model)
  (tmp_path / 'outside.xml').write_text('<mujoco/>')
  (fake_model / 'bad.xml').write_text(
    f'<mujoco><include file="{tmp_path / "outside.xml"}"/></mujoco>'
  )
  with pytest.raises(MenagerieError, match='outside'):
    closure(fake_model / 'bad.xml', fake_model)


def test_assets_dict_detects_basename_collisions(fake_model):
  files = closure(fake_model / 'scene.xml', fake_model)
  assert set(assets_dict(files, fake_model)) == {
    'assets/cube.obj',
    'assets/tex.png',
    'fake.xml',
    'scene.xml',
  }
  (fake_model / 'assets/sub').mkdir()
  shutil.copy(
    fake_model / 'assets/cube.obj', fake_model / 'assets/sub/CUBE.obj'
  )
  with pytest.raises(AssetCollisionError, match='CUBE.obj'):
    assets_dict([*files, fake_model / 'assets/sub/CUBE.obj'], fake_model)


@pytest.mark.parametrize(
  'name,entry',
  [
    ('robotis_op3', 'scene.xml'),  # basename collisions
    ('ufactory_lite6', 'scene.xml'),  # visual/ vs collision/
    ('ms_human_700', 'scene.xml'),  # nested includes under assets/
    ('unitree_g1', 'scene_mjx.xml'),
    ('flexiv_rizon4', 'scene.xml'),  # dead stub XMLs under assets/
    ('iit_softfoot', 'scene.xml'),  # <model file> submodel with its own meshdir
  ],
)
def test_real_closure_compiles_in_isolation(repo_root, tmp_path, name, entry):
  src = repo_root / name
  files = closure(src / entry, src)
  for f in files:
    (tmp_path / name / f.relative_to(src)).parent.mkdir(
      parents=True, exist_ok=True
    )
    shutil.copyfile(f, tmp_path / name / f.relative_to(src))
  a = mujoco.MjModel.from_xml_path(str(src / entry))
  b = mujoco.MjModel.from_xml_path(str(tmp_path / name / entry))
  assert a.nbody == b.nbody
  np.testing.assert_array_equal(a.mesh_vert, b.mesh_vert)
  assert name != 'flexiv_rizon4' or not [
    f for f in files if f.suffix == '.xml' and 'assets' in f.parts
  ]


def test_real_assets_dict(repo_root):
  src = repo_root / 'robotis_op3'
  with pytest.raises(AssetCollisionError):
    assets_dict(closure(src / 'op3.xml', src), src)
  src = repo_root / 'universal_robots_ur5e'
  assets = assets_dict(closure(src / 'scene.xml', src), src)
  a = mujoco.MjModel.from_xml_path(str(src / 'scene.xml'))
  b = mujoco.MjModel.from_xml_string(assets['scene.xml'].decode(), assets)
  np.testing.assert_array_equal(a.mesh_vert, b.mesh_vert)
