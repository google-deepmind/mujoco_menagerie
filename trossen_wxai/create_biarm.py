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

"""Script to generate a bi-arm MuJoCo model from a single-arm model."""

import os
import mujoco

# Constants
SINGLE_ARM_XML = "wxai_follower.xml"
BI_ARM_XML = "wxai_follower_biarm.xml"

# Arm placement (copied from trossen_ai_bimanual.xml)
LEFT_POS = [-0.4575, -0.019, 0.0]
LEFT_QUAT = [1, 0, 0, 0]  # Identity rotation
RIGHT_POS = [0.4575, -0.019, 0.0]
RIGHT_QUAT = [0, 0, 0, 1]  # 180 degree rotation around Z


def main():
  dir_path = os.path.dirname(__file__)
  xml_path = os.path.join(dir_path, SINGLE_ARM_XML)
  output_path = os.path.join(dir_path, BI_ARM_XML)

  # Load single arm spec
  arm_spec = mujoco.MjSpec.from_file(xml_path)

  # Extract and remove contact excludes
  excludes = []
  for exc in list(arm_spec.excludes):
    excludes.append((exc.bodyname1, exc.bodyname2))
    arm_spec.delete(exc)

  # Delete equality constraints
  for eq in list(arm_spec.equalities):
    arm_spec.delete(eq)

  # Create independent copies of the spec
  arm_xml = arm_spec.to_xml()
  arm_spec_left = mujoco.MjSpec.from_string(arm_xml)
  arm_spec_right = mujoco.MjSpec.from_string(arm_xml)

  # Create bi-arm spec
  biarm_spec = mujoco.MjSpec()
  biarm_spec.modelname = "wxai_biarm"

  # Add sites for attachment
  left_site = biarm_spec.worldbody.add_site(
      name="left_attach", pos=LEFT_POS, quat=LEFT_QUAT
  )
  right_site = biarm_spec.worldbody.add_site(
      name="right_attach", pos=RIGHT_POS, quat=RIGHT_QUAT
  )

  # Attach arms
  biarm_spec.attach(arm_spec_left, site=left_site, prefix="left/")
  biarm_spec.attach(arm_spec_right, site=right_site, prefix="right/")

  # Add excludes back with prefixes
  for body1, body2 in excludes:
    biarm_spec.add_exclude(
        name=f"left_{body1}_{body2}",
        bodyname1=f"left/{body1}",
        bodyname2=f"left/{body2}",
    )
    biarm_spec.add_exclude(
        name=f"right_{body1}_{body2}",
        bodyname1=f"right/{body1}",
        bodyname2=f"right/{body2}",
    )

  # Save to file and post-process
  xml_string = biarm_spec.to_xml()

  # Add equality constraints for gripper coupling (hardcoded for this model)
  equality_xml = """
<equality>
  <joint joint1="left/left_carriage_joint" joint2="left/right_carriage_joint" polycoef="0 1 0 0 0"/>
  <joint joint1="right/left_carriage_joint" joint2="right/right_carriage_joint" polycoef="0 1 0 0 0"/>
</equality>
"""
  xml_string = xml_string.replace("</mujoco>", equality_xml + "</mujoco>")

  with open(output_path, "w") as f:
    f.write(xml_string)

  print(f"Successfully created {output_path}")


if __name__ == "__main__":
  main()
