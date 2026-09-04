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
import re

import pytest

import mujoco_menagerie as mm
from mujoco_menagerie import Registry
from mujoco_menagerie import UnknownEntryPointError
from mujoco_menagerie import UnknownRobotError

CATEGORIES = {
  'arm', 'dual_arm', 'end_effector', 'mobile_manipulator', 'mobile_base',
  'quadruped', 'biped', 'humanoid', 'drone', 'biomechanical', 'misc',
}  # fmt: skip


def _registry(robot):
  d = dataclasses.asdict(robot)
  del d['name']
  d['entry_points'] = {
    e.name: {'kind': e.kind, 'file': e.file} for e in robot.entry_points
  }
  return Registry.from_dict(
    {
      'schema': 1,
      'menagerie_commit': 'c' * 40,
      'robots': {'fake': d, 'fake_hand': d},
    }
  )


def test_round_trip_and_lookup(robot):
  reg = _registry(robot)
  assert reg.names() == ['fake', 'fake_hand'] and reg.get('fake') == robot
  with pytest.raises(UnknownRobotError, match="Did you mean 'fake'"):
    reg.get('fak')
  with pytest.raises(mm.RegistryError):
    Registry.from_dict({'schema': 2, 'robots': {}})


def test_entry_lookup(robot):
  assert robot.entry().name == 'scene' and robot.entry('fake').kind == 'robot'
  assert dataclasses.replace(robot, default_scene=None).entry().name == 'fake'
  with pytest.raises(UnknownEntryPointError, match='scene'):
    robot.entry('scen')
  with pytest.raises(UnknownEntryPointError):
    robot.entry('')


def test_resolution_through_robot(robot, base_url, cache_dir):
  cache = mm.Cache(cache_dir, base_url)
  assert (
    robot.xml(cache=cache).name == 'scene.xml'
    and robot.xml('fake', cache).name == 'fake.xml'
  )
  assert {p.name for p in robot.files(cache=cache)} == {
    'scene.xml',
    'fake.xml',
    'cube.obj',
    'tex.png',
  }
  assert set(robot.assets(cache=cache)) == {
    'scene.xml',
    'fake.xml',
    'assets/cube.obj',
    'assets/tex.png',
  }
  assert (
    robot.model(cache=cache).ngeom == 2
    and robot.model('fake', cache).ngeom == 1
  )
  assert robot.spec(cache=cache).compile().nu == 1


@pytest.fixture
def real(registry_path):
  return Registry.load(registry_path)


def test_real_registry_shape(real, repo_root):
  dirs = {
    d.name
    for d in repo_root.iterdir()
    if any(d.glob('*.xml')) and d.name not in {'assets', 'test', 'python'}
  }
  assert set(real.names()) == dirs and re.fullmatch(
    r'[0-9a-f]{40}', real.commit
  )
  for r in real.robots.values():
    assert (
      re.fullmatch(r'[0-9a-f]{40}', r.oid)
      and r.asset == f'{r.name}-{r.oid[:16]}.tar.xz'
    )
    assert r.entry(r.default_model).kind == 'robot', r.name
    assert (
      r.default_scene is None or r.entry(r.default_scene).kind == 'scene'
    ), r.name
    assert (
      r.installed_size > 0
      and r.license != 'Unknown'
      and r.category in CATEGORIES
    ), r.name
    assert all(
      (repo_root / r.name / e.file).is_file() for e in r.entry_points
    ), r.name


def test_real_registry_defaults(real):
  g1 = real.get('unitree_g1')
  assert (g1.default_scene, g1.default_model, g1.category, g1.license) == (
    'scene',
    'g1',
    'humanoid',
    'BSD-3-Clause',
  )
  assert any(e.is_mjx for e in g1.entry_points)
  assert real.get('pal_talos').default_scene == 'scene_position'
  assert real.get('shadow_hand').default_scene == 'scene_left'
  assert 'keyframes' not in real.get('shadow_hand').entry_names
  assert real.get('franka_emika_panda').default_model == 'panda'
  assert real.get('realsense_d435i').entry().name == 'd435i'


def test_bundled_matches_file(real, registry_path):
  assert mm.names() == real.names() and mm.commit() == real.commit
  assert [r.name for r in mm.robots('humanoid')] == [
    n for n in real.names() if real.robots[n].category == 'humanoid'
  ]
