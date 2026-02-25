# Copyright 2022 DeepMind Technologies Limited
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
"""Tests for all models."""

import pathlib
from typing import List

from absl.testing import absltest
from absl.testing import parameterized
import mujoco

# Internal import.


_ROOT_DIR = pathlib.Path(__file__).parent.parent
_MODEL_DIRS = [f for f in _ROOT_DIR.iterdir() if f.is_dir()]
_MODEL_XMLS: List[pathlib.Path] = []


def _get_xmls(pattern: str) -> List[pathlib.Path]:
  for d in _MODEL_DIRS:
    # Produce tuples of test name and XML path.
    for f in d.glob(pattern):
      test_name = str(f).removeprefix(str(f.parent.parent))
      yield (test_name, f)

_MODEL_XMLS = list(_get_xmls('scene*.xml'))

# Total simulation duration, in seconds.
_MAX_SIM_TIME = 0.1
# Scale for the pseudorandom control noise.
_NOISE_SCALE = 1.0


def _pseudorandom_ctrlnoise(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    i: int,
    noise: float,
) -> None:
  for j in range(model.nu):
    ctrlrange = model.actuator_ctrlrange[j]
    if model.actuator_ctrllimited[j]:
      center = 0.5 * (ctrlrange[1] + ctrlrange[0])
      radius = 0.5 * (ctrlrange[1] - ctrlrange[0])
    else:
      center = 0.0
      radius = 1.0
    data.ctrl[j] = center + radius * noise * (2*mujoco.mju_Halton(i, j+2) - 1)


class ModelsTest(parameterized.TestCase):
  """Tests that MuJoCo models load and do not emit warnings."""

  @parameterized.named_parameters(_MODEL_XMLS)
  def test_compiles_and_steps(self, xml_path: pathlib.Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    i = 0
    while data.time < _MAX_SIM_TIME:
      _pseudorandom_ctrlnoise(model, data, i, _NOISE_SCALE)
      mujoco.mj_step(model, data)
      i += 1
    # Check no warnings were triggered during the simulation.
    if not all(data.warning.number == 0):
      warning_info = '\n'.join([
          f'{mujoco.mjtWarning(enum_value).name}: count={count}'
          for enum_value, count in enumerate(data.warning.number) if count
      ])
      self.fail(f'MuJoCo warning(s) encountered:\n{warning_info}')


if __name__ == '__main__':
  absltest.main()
