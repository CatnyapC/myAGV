Do not add Python shebang lines.

If a shebang is ever explicitly required, only use:
#!/usr/bin/env python3

Write Markdown (.md) files in English.
After each completed change, commit and push the task's changes.

## Thesis documents and references

- Keep `paper/` local-only and ignored by Git; never force-add its documents or backups.
- Treat `paper/卒論橋本→JIN20261119.docx` as the professor-approved reference baseline. Keep the received file unchanged and edit a copy named `卒論JIN→橋本YYYYMMDD.docx`, using the working date.
- Always retain all seven bibliography entries [1]–[7] from that baseline, including entries not cited in the chapter currently being edited. Preserve their exact wording, author names, titles, numbering, punctuation, whitespace, URLs, and date strings. Do not normalize, correct, renumber, shorten, remove, or replace them without an explicit user request to change those entries.
- Add new references after the preserved entries and use the corresponding numbers in the text. Before and after any thesis edit, compare the preserved bibliography entries against the baseline to ensure none changed or disappeared.
- Preserve the professor's yellow highlights and mark JIN's subsequent edits in light blue/cyan.

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
