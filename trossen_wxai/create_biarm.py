# Copyright 2026 DeepMind Technologies Limited
#
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


"""Script to generate a bi-arm MuJoCo model from a single-arm model."""

import os
import mujoco

# Constants
SINGLE_ARM_XML = "wxai_follower.xml"
BI_ARM_XML = "wxai_follower_biarm.xml"

# Arm placement — bases are 82cm apart.
HALF_SPACING = 0.41  # 82cm / 2
LEFT_POS = [-HALF_SPACING, -0.019, 0.0]
LEFT_QUAT = [1, 0, 0, 0]  # Identity rotation
RIGHT_POS = [HALF_SPACING, -0.019, 0.0]
RIGHT_QUAT = [0, 0, 0, 1]  # 180 degree rotation around Z


def main():
  dir_path = os.path.dirname(__file__)
  xml_path = os.path.join(dir_path, SINGLE_ARM_XML)
  output_path = os.path.join(dir_path, BI_ARM_XML)

  os.chdir(dir_path)

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

  # Copy visual settings from the single-arm spec
  biarm_spec.visual.headlight.diffuse = arm_spec.visual.headlight.diffuse
  biarm_spec.visual.headlight.ambient = arm_spec.visual.headlight.ambient
  biarm_spec.visual.headlight.specular = arm_spec.visual.headlight.specular
  biarm_spec.visual.scale.contactwidth = arm_spec.visual.scale.contactwidth
  biarm_spec.visual.scale.contactheight = arm_spec.visual.scale.contactheight
  biarm_spec.visual.scale.forcewidth = arm_spec.visual.scale.forcewidth

  # NOTE: maxhullvert is injected via XML post-processing below,
  # because the MjSpec roundtrip (to_xml/from_string) drops it.
  maxhullvert = max(
      (m.maxhullvert for m in arm_spec.meshes if m.maxhullvert > 0),
      default=-1,
  )

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

  # Inject maxhullvert default (lost during to_xml/from_string roundtrip)
  if maxhullvert > 0:
    xml_string = xml_string.replace(
        "<default>",
        f'<default>\n    <mesh maxhullvert="{maxhullvert}"/>',
        1,  # only the first (top-level) <default>
    )

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
