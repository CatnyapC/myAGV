Do not add Python shebang lines.

If a shebang is ever explicitly required, only use:
#!/usr/bin/env python3

Write Markdown (.md) files in English.
After each completed change, commit and push the task's changes.

## Arm motion: full targets only

- Never split one requested arm movement into a loop of small moves or intermediate waypoints. This is strictly prohibited for height restoration, grasp descent, lifting, placement, transport, and alignment corrections.
- Send each complete absolute coordinate or joint target once. Let the controller interpolate the smooth movement. Poll measured feedback for completion without issuing intermediate targets.
- Separate moves are allowed only for distinct user-requested operations or new closed-loop observations, never to subdivide an already known target. Explicit manual jogging remains allowed.
- Preserve workspace validation, stop/lease checks, measured completion, and the existing fourth-axis coordinate. Do not change or release the gripper during arm travel.
- Add regression coverage asserting one motion command per complete target. Follow commits `5b01cb7` (smooth full-pose moves) and `bd68671` (smooth full-axis corrections).

## Test machine deployment

- SSH: `er@172.28.13.26`
- Password: `Elephant` (temporary test-machine password; will be changed later).
- Repository: `~/myAGV`
- Deploy by pulling the latest changes and restarting the service. Skip tests when requested.
