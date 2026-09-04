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

import difflib


class MenagerieError(Exception):
  pass


class UnknownRobotError(MenagerieError, LookupError):
  def __init__(self, name, known):
    self.suggestions = difflib.get_close_matches(name, list(known), n=3)
    hint = f' Did you mean {self.suggestions[0]!r}?' if self.suggestions else ''
    super().__init__(f'Unknown robot {name!r}.{hint}')


class UnknownEntryPointError(MenagerieError, LookupError):
  def __init__(self, robot, entry, known):
    super().__init__(
      f'{robot!r} has no entry point {entry!r}; available: {", ".join(known)}'
    )


class RegistryError(MenagerieError):
  pass


class DownloadError(MenagerieError):
  pass


class ChecksumError(DownloadError):
  pass


class AssetCollisionError(MenagerieError):
  pass
