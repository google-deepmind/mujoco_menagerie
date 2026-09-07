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

import os
import tarfile

from mujoco_menagerie._archive import write_archive


def test_archive_is_reproducible_and_normalised(fake_model, tmp_path):
  files = [p for p in fake_model.rglob('*') if p.is_file()]
  write_archive(fake_model, files, tmp_path / 'a.tar.xz', prefix='fake')
  for f in files:  # perturb what git does not preserve
    os.utime(f, (1, 1))
    os.chmod(f, 0o600)
  write_archive(fake_model, files[::-1], tmp_path / 'b.tar.xz', prefix='fake')
  assert (tmp_path / 'a.tar.xz').read_bytes() == (
    tmp_path / 'b.tar.xz'
  ).read_bytes()
  with tarfile.open(tmp_path / 'a.tar.xz') as tf:
    members = tf.getmembers()
  assert [m.name for m in members] == sorted(m.name for m in members)
  for m in members:
    assert m.name.startswith('fake/') and m.isfile()
    assert (m.mtime, m.uid, m.gid, m.uname, m.gname, m.mode) == (
      0,
      0,
      0,
      '',
      '',
      0o644,
    )
