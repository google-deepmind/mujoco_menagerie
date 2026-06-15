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
"""Mirror the left SHARPA hand into a right-handed SHARPA via Y-plane reflection.

Pipeline:
  1) Y-mirror every mesh in the input asset dir (vertex flip + face winding
     reverse) into a sibling output asset dir, renaming ``left_*`` filenames to
     ``right_*``.
  2) Load the left MJCF into ``MjSpec``, mirror its body tree, joints, geoms,
     sites, inertials, and class defaults through ``M_y = diag(1, -1, 1)``,
     and point its ``meshdir`` at the mirrored asset dir.
  3) Write the resulting XML, then do a global ``left_`` → ``right_`` rename
     across body / joint / site / geom / mesh / material identifiers and the
     model name. Cross-references (actuator joint refs, contact-exclude body
     refs, geom mesh refs) update automatically since the rewrite is uniform.

Verification: at qpos=0, every fingertip on the produced right hand lands at
the M_y mirror of the corresponding left fingertip to <1 μm.  See
``fingertip_check.py`` for the test.

Run (deps are declared in the PEP 723 block below, so ``uv`` handles them):
    uv run make_right.py
    uv run make_right.py --view
"""

# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "mujoco>=3.0",
#   "numpy",
#   "trimesh",
# ]
# ///

from __future__ import annotations

import argparse
import re
from pathlib import Path

import mujoco
import numpy as np
import trimesh

HERE = Path(__file__).parent

# Mirror through the Y=0 plane.
M_Y = np.diag([1.0, -1.0, 1.0])

# Default class names whose geom / joint / site attributes may carry pose data
# that needs mirroring (e.g. ``<geom quat="1 1 -1 1"/>`` inside an
# elastomer_visual default — inherited by every elastomer geom).
_DEFAULT_CLASS_NAMES = (
  'main',
  'sharpa',
  'visual',
  'collision',
  'elastomer_visual',
  'elastomer_collision',
  'CMC_joint',
  'PCMC_joint',
  'MCP_joint',
  'PIP_joint',
  'DIP_joint',
)


def mirror_pos(p):
  p = np.asarray(p, dtype=float).copy()
  p[1] = -p[1]
  return p


def mirror_quat_wxyz(q):
  """Return the wxyz quaternion whose rotation matrix is M_y @ R(q) @ M_y.

  ``mju_quat2Mat`` does not normalize, so MJCF-friendly unnormalized quats like
  ``(1, 1, -1, 1)`` would produce a scaled "rotation" and round-trip to garbage.
  Normalize first.
  """
  q = np.asarray(q, dtype=float)
  n = float(np.linalg.norm(q))
  if n == 0.0:
    return np.array([1.0, 0.0, 0.0, 0.0])
  R = np.zeros(9)
  mujoco.mju_quat2Mat(R, q / n)
  R_new = M_Y @ R.reshape(3, 3) @ M_Y
  out = np.zeros(4)
  mujoco.mju_mat2Quat(out, R_new.ravel())
  return out


def mirror_axis(a):
  """Reflect the axis through M_y, then negate it (``0 0 1`` -> ``0 0 -1``).

  SHARPA does not motion-mirror its two hands: the upstream left and right URDFs
  share one joint convention (``axis="0 0 1"`` with identical ranges), so the
  same encoder sign drives both and positive ``q`` is flexion on each hand. We
  must reproduce that, not the geometric motion-mirror of the left. The fix is a
  pure relabel of the mirrored joint coordinate ``q -> -q``, i.e. negate the
  (M_y-reflected) axis here and keep the range in ``mirror_joint_range``. The
  result matches the upstream right URDF in both reachable configs and qpos sign.
  Negating the axis without restoring the range would flip the motion direction
  but leave the range on the wrong (extension) side."""
  return -mirror_pos(a)


def mirror_joint_range(r):
  """Keep the range identical to the left hand: mirroring is only of the
  geometry; the joint coordinate is relabeled ``q -> -q`` via the axis negation
  in ``mirror_axis``, so the range stays the left's ``[lo, up]``."""
  return np.asarray(r, dtype=float)


def preprocess_meshes(src_dir: Path, dst_dir: Path, force: bool):
  """Y-mirror every .stl/.obj under ``src_dir`` into ``dst_dir`` (recursively).

  Files whose name contains ``left_`` are renamed to ``right_`` in the output;
  side-neutral filenames (e.g. ``palm000.obj``, ``elastomer_HB1_4F.STL``,
  ``DP_HB1_TH.STL``) keep their names — the mirrored geometry just overwrites
  in the dedicated right-side mesh dir.
  """
  dst_dir.mkdir(parents=True, exist_ok=True)
  n = 0
  for asset in sorted(src_dir.rglob('*')):
    if asset.is_dir() or asset.suffix.lower() not in ('.stl', '.obj'):
      continue
    rel = asset.relative_to(src_dir)
    # Rename "left_" → "right_" in the filename and any parent dirs.
    rel_renamed = Path(*[p.replace('left_', 'right_') for p in rel.parts])
    out_path = dst_dir / rel_renamed
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.exists() and not force:
      continue
    m = trimesh.load(str(asset), force='mesh', process=False)
    verts = np.asarray(m.vertices, dtype=float)
    verts[:, 1] *= -1.0
    faces = np.asarray(m.faces, dtype=np.int64)[:, [0, 2, 1]]
    trimesh.Trimesh(vertices=verts, faces=faces, process=False).export(
      str(out_path)
    )
    n += 1
  print(f'preprocessed {n} mesh files into {dst_dir}')


def mirror_body(body):
  """Recursively mirror a body and all its children through the Y=0 plane."""
  body.pos = mirror_pos(body.pos)
  body.quat = mirror_quat_wxyz(body.quat)
  if hasattr(body, 'ipos'):
    body.ipos = mirror_pos(body.ipos)
  if hasattr(body, 'iquat'):
    body.iquat = mirror_quat_wxyz(body.iquat)

  for joint in body.joints:
    if joint.type == mujoco.mjtJoint.mjJNT_FREE:
      continue
    joint.pos = mirror_pos(joint.pos)
    joint.axis = mirror_axis(joint.axis)
    joint.range = mirror_joint_range(joint.range)

  for geom in body.geoms:
    geom.pos = mirror_pos(geom.pos)
    geom.quat = mirror_quat_wxyz(geom.quat)

  for site in body.sites:
    site.pos = mirror_pos(site.pos)
    site.quat = mirror_quat_wxyz(site.quat)

  for child in body.bodies:
    mirror_body(child)


def mirror_defaults(spec):
  """Mirror the inherited geom/joint/site quats and joint ranges in every default class.

  Defaults like ``<default class="elastomer_visual"><geom quat="1 1 -1 1"/></default>``
  are inherited by geoms that don't override them, so the inherited quat must be
  mirrored too — otherwise elastomers point the wrong way after the body flip.
  """
  for cname in _DEFAULT_CLASS_NAMES:
    d = spec.find_default(cname)
    if d is None:
      continue
    d.geom.pos = mirror_pos(d.geom.pos)
    d.geom.quat = mirror_quat_wxyz(d.geom.quat)
    d.joint.pos = mirror_pos(d.joint.pos)
    d.joint.axis = mirror_axis(d.joint.axis)
    d.joint.range = mirror_joint_range(d.joint.range)
    d.site.pos = mirror_pos(d.site.pos)
    d.site.quat = mirror_quat_wxyz(d.site.quat)


def build_mirrored_spec(
  left_xml: Path, right_meshdir_rel: str
) -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(left_xml))
  spec.compiler.meshdir = right_meshdir_rel
  # Repoint mesh ``file=`` attributes at the renamed mirrored files.
  # ``spec.to_xml()`` validates by opening each mesh; the file must exist on
  # disk under the new meshdir, so this rename has to happen before serialize.
  # Mesh ``name`` attributes (and geom ``mesh=`` refs) are renamed later in the
  # XML rewrite pass — they're string-only and don't need to resolve to a file.
  for mesh in spec.meshes:
    if 'left_' in mesh.file:
      mesh.file = mesh.file.replace('left_', 'right_')
  mirror_defaults(spec)
  for body in spec.worldbody.bodies:
    mirror_body(body)
  return spec


# Word-boundary "left_" → "right_" rewrite. We match the literal token; the
# polished left_hand.xml contains "left_" only as a prefix on identifiers and
# mesh filenames, never inside larger words, so a plain substitution is safe.
_LEFT_TOKEN = re.compile(r'left_')
_MODEL_NAME = re.compile(r'(<mujoco[^>]*\bmodel=")left([^"]*")')


def rewrite_left_to_right(xml_text: str) -> str:
  xml_text = _LEFT_TOKEN.sub('right_', xml_text)
  # Catch the model name attribute even if the prefix is "left" (no underscore).
  xml_text = _MODEL_NAME.sub(r'\1right\2', xml_text)
  return xml_text


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--left-xml', type=Path, default=HERE / 'left_hand.xml')
  p.add_argument(
    '--left-meshdir',
    type=Path,
    default=HERE / 'assets' / 'left',
    help='Source mesh dir (relative paths inside the XML are resolved here).',
  )
  p.add_argument('--out-xml', type=Path, default=HERE / 'right_hand.xml')
  p.add_argument(
    '--out-meshdir',
    type=Path,
    default=HERE / 'assets' / 'right',
    help='Destination dir for the Y-mirrored copies of every mesh.',
  )
  p.add_argument(
    '--force-meshes',
    action='store_true',
    help='Re-export mirrored meshes even if the destination already has them.',
  )
  p.add_argument(
    '--view',
    action='store_true',
    help='Launch the MuJoCo viewer on the produced right hand.',
  )
  args = p.parse_args()

  preprocess_meshes(
    args.left_meshdir, args.out_meshdir, force=args.force_meshes
  )

  # meshdir in the output XML is recorded relative to the XML file's parent,
  # matching the left XML's convention (``meshdir="assets/left/"``).
  out_meshdir_rel = (
    args.out_meshdir.relative_to(args.out_xml.parent).as_posix() + '/'
  )
  spec = build_mirrored_spec(args.left_xml, out_meshdir_rel)

  raw_xml = spec.to_xml()
  rewritten = rewrite_left_to_right(raw_xml)
  args.out_xml.write_text(rewritten)
  print(f'wrote {args.out_xml}')

  model = mujoco.MjModel.from_xml_path(str(args.out_xml))
  print(
    f'compiled: nq={model.nq} nv={model.nv} nbody={model.nbody} '
    f'njnt={model.njnt} ngeom={model.ngeom} nsite={model.nsite} nu={model.nu}'
  )

  if args.view:
    from mujoco import viewer as mj_viewer

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mj_viewer.launch(model, data)


if __name__ == '__main__':
  main()
