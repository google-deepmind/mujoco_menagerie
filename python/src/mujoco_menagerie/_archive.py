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

import hashlib
import os
import pathlib
import tarfile
from collections.abc import Iterable

XZ_PRESET = 6  # 8 MiB decoder dictionary; 9 needs 64 MiB per process


def write_archive(
  model_dir: pathlib.Path,
  files: Iterable[pathlib.Path],
  out: pathlib.Path,
  prefix: str,
) -> None:
  model_dir = pathlib.Path(model_dir).resolve()
  members = sorted(
    (pathlib.Path(f).resolve().relative_to(model_dir).as_posix(), f)
    for f in files
  )
  part = out.with_name(out.name + '.part')
  with tarfile.open(part, 'w:xz', preset=XZ_PRESET) as tf:
    for rel, path in members:
      info = tarfile.TarInfo(f'{prefix}/{rel}')
      info.size = os.path.getsize(path)
      with open(path, 'rb') as fh:
        tf.addfile(info, fh)
  os.replace(part, out)


def sha256_file(path: pathlib.Path) -> str:
  h = hashlib.sha256()
  with open(path, 'rb') as fh:
    while chunk := fh.read(1 << 20):
      h.update(chunk)
  return h.hexdigest()
