# Frequently Asked Questions

## How do I attach a hand to an arm?

In Menagerie, the convention is to add a site called `"attachment_site"` to each
arm. This site can subsequently be used to rigidly attach an end-effector,
like a gripper, to the arm.

If you are using
[PyMJCF](https://github.com/google-deepmind/dm_control/tree/main/dm_control/mjcf),
you can use the following code snippet to do this. Any `qpos` or `ctrl`
keyframes defined in the arm will be automatically modified to account for the
added degrees of freedom.

```python
def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
  """Attaches a hand to an arm.

  The arm must have a site named "attachment_site".

  Args:
    arm_mjcf: The mjcf.RootElement of the arm.
    hand_mjcf: The mjcf.RootElement of the hand.

  Raises:
    ValueError: If the arm does not have a site named "attachment_site".
  """
  physics = mjcf.Physics.from_mjcf_model(hand_mjcf)

  attachment_site = arm_mjcf.find("site", "attachment_site")
  if attachment_site is None:
    raise ValueError("No attachment site found in the arm model.")

  # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
  arm_key = arm_mjcf.find("key", "home")
  if arm_key is not None:
    hand_key = hand_mjcf.find("key", "home")
    if hand_key is None:
      arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
      arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
    else:
      arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
      arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

  attachment_site.attach(hand_mjcf)
```
