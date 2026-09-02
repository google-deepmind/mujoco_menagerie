# MuJoCo Menagerie Python Library

```bash
pip install mujoco-menagerie
uvx mujoco-menagerie view unitree_g1
```

```python
import mujoco_menagerie as mm
```

**Browse**

```python
mm.names()
mm.robots('humanoid')  # or arm, quadruped, mobile_manipulator, ...
r = mm.get('franka_emika_panda')  # no download yet
r.display_name, r.license, r.oid
r.entry_names  # ('hand', 'mjx_hand', ..., 'scene'): every top-level XML that compiles
r.default_scene, r.default_model
```

**Load**

```python
model = mm.load('unitree_g1')  # the default scene: floor, lights, robot
model = r.model('mjx_scene')  # any entry point
model = r.model(r.default_model)  # the robot alone
spec = r.spec('panda')  # mujoco.MjSpec, to edit before compiling
```

**Edit**

```python
spec = mm.get('unitree_g1').spec('g1')
for joint in spec.joints:
    if joint.name.endswith('_knee_joint'):
        joint.armature = 0.01
spec.body('pelvis').add_site(name='imu')
model = spec.compile()
```

**Attach a hand to an arm**

```python
arm = mm.get('kuka_iiwa_14').spec()
hand = mm.get('wonik_allegro').spec('left_hand')
hand.body('palm').pos[:] = (0, 0, 0.095)
arm.attach(hand, prefix='allegro/', site=arm.site('attachment_site'))
model = arm.compile()
```

**Place two copies of a robot**

```python
ur5e = mm.get('universal_robots_ur5e')
scene = mujoco.MjSpec()
scene.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # match the arm, or attach warns
for prefix, y in (('left/', -0.5), ('right/', 0.5)):
    arm = ur5e.spec('ur5e')
    frame = scene.worldbody.add_frame(pos=[0, y, 0])
    frame.attach_body(arm.body('base'), prefix, '')
model = scene.compile()
```

**Files**

```python
r.path()  # the model directory
r.xml('scene')  # one XML
r.files('scene')  # every file scene.xml depends on; copy these to vendor, and note r.oid
r.assets('scene')  # the same as {relative path: bytes}, for MjModel.from_xml_string
```

`assets()` raises for `robotis_op3`, `ufactory_lite6` and `sharpa_wave`, whose meshes share basenames MuJoCo cannot tell apart in memory.

**Cache**

```python
mm.prefetch(['unitree_g1'])  # download ahead of time; no argument means everything, 253 MB
mm.prune()  # delete models this version no longer references
mm.load('unitree_g1', cache=mm.Cache(dir='/shared/menagerie'))
```

| | argument | environment | default |
|---|---|---|---|
| cache directory | `mm.Cache(dir=)` | `MENAGERIE_CACHE_DIR` | `~/.cache/mujoco_menagerie` |
| archive source | `mm.Cache(base_url=)` | `MENAGERIE_BASE_URL` | the GitHub release |
| checkout to use instead | `mm.Cache(root=)` | `MENAGERIE_ROOT` | unset |

A warm cache can be read-only. Concurrent cold starts on one model are safe: a model directory is either complete or absent.

**Command line**

```bash
uvx mujoco-menagerie names | info NAME | path NAME [ENTRY] | view NAME [ENTRY] | prefetch [NAME...] | prune
```

**Speed**, laptop, local mirror

| `import mujoco_menagerie` | 0.09 s |
|---|---|
| `mm.load('unitree_g1')`, cold | 0.9 s |
| `mm.load('unitree_g1')`, warm | 0.27 s |
| `mm.prefetch()` | 16 s |

**Develop**

```bash
make registry  # derive python/src/mujoco_menagerie/registry.json from the checkout
make python-test
MENAGERIE_ROOT=. uv run --project python python -c "import mujoco_menagerie as mm; mm.load('unitree_g1')"
```

**Release**

Bump `version` in `python/pyproject.toml` (`2026.9.0`, no leading zero), add to `CHANGELOG.md`, tag `v2026.9.0` and push. `release.yml` uploads the model archives, re-verifies them, then builds, smoke tests and publishes the wheel; it needs a PyPI trusted publisher (workflow `release.yml`, environment `pypi`) and a `pypi` GitHub environment. Manual: `make build`, then `UV_PUBLISH_TOKEN=... make publish`.

The package is Apache-2.0. Each model ships its own `LICENSE`; `r.license` reports it.
