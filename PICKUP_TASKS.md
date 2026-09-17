# Minimal left-side pickup

Confirmed hardware convention: with P340 J1 at zero, extension reaches the
robot's left. Use that physical convention rather than mounting-angle labels.

- [x] Add PICKUP teaching mode: base forward/back at 3 cm/s, arm reach and height.
- [x] Keep pickup and folded transport J1 at zero; reject old nonzero-J1 poses.
- [x] Store the same base pose and arm angles in the existing station JSON.
- [x] Reach a staging point 30 cm ahead of or behind the recorded base pose.
- [x] Align along the vehicle's forward/back axis; permit small heading corrections.
- [x] Stop on obstacles, stale feedback, large errors, no progress or timeout.
- [x] Grasp using taught reach/height, fold, return to this run's startup pose.
- [x] Test both travel directions, overshoot reversal, rotation limits and failure stops.
- [ ] Re-teach stations and transport pose; run an empty-gripper hardware check.

Large turns happen at staging. Final alignment uses up to 3 cm/s translation
and 0.05 rad/s rotation by default. It stops on more than 2 cm lateral error or
15 degrees heading error. It requires 1 cm longitudinal / 1 degree heading
alignment before stopping, then checks standstill within 2 cm / 2 degrees.
These are localization tolerances, not guaranteed physical grasp accuracy.

The arm handles lateral reach and height during teaching, then replays those
angles. There is no live object detection or automatic reach adjustment for
moved objects. Re-teach if the item moves. No second station or dropoff profile.

The earlier front-docking draft is replaced. The useful idea retained from
[Nav2 docking](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_docking/)
is coarse travel followed by controlled final alignment; no Nav2 dependency.
