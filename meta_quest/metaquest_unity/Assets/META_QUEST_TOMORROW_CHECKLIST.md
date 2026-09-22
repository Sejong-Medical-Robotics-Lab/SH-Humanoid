# Meta Quest coordinate verification checklist

Current implementation status:

- Controller poses are measured relative to `TrackingSpace`.
- `AxisGuide` is located at the `TrackingSpace` origin.
- Position conversion is a provisional axis reorder only:
  `robotX = unityX`, `robotY = unityZ`, `robotZ = unityY`.
- Axis signs and rotation conversion are **not final** until measured with a real Quest.
- The robot team's existing `base_link` is used as the target frame. Do not redefine it here.

## Tomorrow in the lab

- [ ] Move a controller to the physical right and record which Unity axis increases/decreases.
- [ ] Move a controller upward and record which Unity axis increases/decreases.
- [ ] Move a controller forward and record which Unity axis increases/decreases.
- [ ] Confirm or correct signs in `RobotBaseLinkCoordinates.UnityPositionToRobotCandidate`.
- [ ] Confirm the physical left controller logs as `LEFT` and the right as `RIGHT`.
- [ ] Observe quaternion and Euler logs while rotating one controller about one axis at a time.
- [ ] Derive robot rotation conversion only after these measurements; do not guess it.
- [ ] Confirm the latest URDF joint names before integration.

## URDF naming note

The latest URDF documentation describes seven joints per arm and three wrist joints
(`wrist_yaw`, `wrist_pitch`, `wrist_roll`). An older notice used the single names
`left_wrist_joint` / `right_wrist_joint`; those old single-wrist names are intentionally
not hardcoded in this Quest-side implementation.

## Optional Scene objects

- `Cube`: optional visual test object; safe to keep for now.
- `Directional Light`: useful for visualization; safe to keep.
- Camera Rig tracking objects: do not delete or rearrange without an integration reason.
