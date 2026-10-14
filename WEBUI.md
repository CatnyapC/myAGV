# Local WebUI

Robot live map/pose/camera setup: [HARDWARE_WEBUI.md](HARDWARE_WEBUI.md).
That mode is read-only; the demo workflow below remains separate.

Demo map editor and simulated navigation on the development Mac. No ROS/serial
connection or hardware motion. All displayed poses/routes are explicitly simulated.
The four panels follow `WEBUI_PLAN.md`; task status is in `WEBUI_TASKS.md`.

## Start

Requires Python 3.11+, uv, Node 22.12+ or 24+, npm and PM2. The current Node 26
development environment was checked. Robot ROS/Python compatibility is deferred.

```bash
uv sync --extra web
npm --prefix web ci
npm --prefix web run build
pm2 start ecosystem.config.cjs --only myagv-api,myagv-web
```

Development UI: <http://127.0.0.1:5173>. The API and built static UI are also served
at <http://127.0.0.1:8791>. The API binds to loopback; the development frontend
binds to all interfaces. On a trusted LAN or phone hotspot, open
`http://<Mac-LAN-IP>:5173` from another device. The development frontend proxies
`/api` and WebSocket traffic to the backend; frontend and backend sources stay
separate. For a built UI only, start just `myagv-api` with PM2.

LAN access requires explicitly allowing the frontend's origin on the backend:

```bash
MYAGV_WEB_ORIGIN="http://<LAN-IP>:5173" pm2 restart myagv-api --update-env
```

Replace `<LAN-IP>` with the host's address. Unconfigured LAN hosts and unrelated
origins remain blocked. This enables the demo only; hardware stays unconnected.

```bash
pm2 restart myagv-api myagv-web
pm2 logs myagv-api --lines 20 --nostream
pm2 logs myagv-web --lines 20 --nostream
pm2 stop myagv-api myagv-web
```

Only these two process names belong to this project. PM2's existing process list
is left intact. No login/startup registration or global `pm2 save` is performed.

## Manual acceptance

1. Open the UI yourself. Editing and simulation are available immediately while
   connected and stopped. This is a single-user UI. Closing the last UI WebSocket
   stops simulation; reconnecting never resumes it.
2. The Demo map loads automatically. Menu > Settings > **Auto-align walls**
   aligns dominant walls. Disable it to enter a manual display angle. Changes
   auto-save after 400 ms; **Saved** requires backend acknowledgement. Retry is
   offered on failure. Display rotation never changes map-frame zone coordinates.
3. MAP > More > **No-go zone**: click two opposite corners, then **Save**. Escape
   or Cancel discards the preview. Coordinates can also be entered in view meters
   through the map's expandable coordinate row. Zones must fit the original map
   bounds and be at least one map cell wide/high.
4. Select a saved zone on the map or in OPERATE, then **Delete**. Rotate the map,
   reload the page and restart `myagv-api`; saved zones should stay in place.
5. Shared divider handles work immediately while connected and stopped; hover
   highlights them. No Edit layout is required for resizing. Menu >
   **Edit layout** unlocks only panel repositioning by title.
   Drag a divider to resize adjacent panels together, or focus it and use arrow keys
   (Shift moves two grid units). Settings W/H inputs adjust the same boundaries.
   Resize the window: panels fill
   the available height; short/narrow windows scroll at minimum content sizes.
6. The top-toolbar circular-arrow button **Restore default layout** exits maximization
   and restores the four default panels. It also appears in Menu and Settings.
   Reload to confirm automatic layout persistence. Verify desktop/mobile layout
   and visible STOP yourself.

7. The cyan chassis outline shows simulated position and heading. OPERATE shows
   map-frame X/Y/yaw. MAP > More > **Fit robot** centers it without resetting zoom.
8. Click free map space to preview a cyan goal, or enter X/Y/yaw in OPERATE.
   Click **Simulate** explicitly; the active goal turns orange.
   The cyan line is the actual remaining A* route, simplified into straight
   segments checked against the inflated costmap (including touched cell corners).
   The simulator turns at necessary bends instead of at every grid cell;
   the chassis moves/turns with a breathing glow. Arrival adopts the requested yaw
   and clears the line/glow. Reduced-motion settings disable breathing.
9. While stopped, draw a no-go rectangle across the direct route. MAP > More >
   **Global costmap** shows red blocked/inflated cells. Navigate again: expect a
   detour or No path. Goals in walls, unknown cells, zones or clearance are rejected.
   A zone covering the chassis clearance envelope is rejected. Zone edits and
   layout changes are disabled during navigation. Delete the zone and retry.
10. Try **Cancel**, STOP and closing the last UI tab during simulation.
    They clear navigation without moving on reconnect. MAP > More > **Reset demo
    pose** restores a safe start; service restart re-applies zones before simulation.

11. Menu > **Import reference photo** accepts a phone PNG/JPEG/WebP (up to 16 MiB).
    The browser converts it to an RGB/RGBA PNG up to 1600 pixels on its longest side
    and 4 MiB. Edit name/appearance manually; the reference has no map observation.
12. Use the camera icon in either camera panel header for **Demo capture**. It creates
    a synthetic fixture and saves the simulated base pose/time before generation or
    storage. Select its dashed map marker or photo row to inspect it. In Edit, choose
    the reference's existing item and **Confirm association**. Names alone never merge
    items. This is manual association; VLM feature extraction/matching is deferred.
13. **View point** previews the observation's capture base pose. It is not an object
    position. **Simulate** remains a separate action and checks the current costmap.
    To link an existing validated record from `stations.json`, select a Station in
    Edit and explicitly confirm it. **Approach** previews only the station base pose.
    The file/schema are unchanged; no measured station is invented. Later record
    changes invalidate its digest until explicitly reconfirmed. Fetch stays disabled.
14. Reload/restart to check photos, features and associations. Delete removes the photo
    and any item with no remaining photos. Missing files show Image unavailable.
    Open an editor in two tabs; saving one makes the other form stale and rejects its
    save. Close/reopen it to refresh. Editing/import/capture require a stopped simulator.
15. OPERATE > REQUEST: enter natural language and **Resolve**. A unique match selects
    an existing item and previews its validated station goal when a confirmed,
    available current-map observation exists. Missing prerequisites show a reason
    without a goal. Multiple matches show candidate buttons; no match clears selection.
    Choosing an ambiguous candidate selects it for manual review/Approach.
    Resolve does not start movement; **Simulate** still validates the current costmap.
    STOP/disconnect discards late results; provider failures/timeouts require explicit retry.

Camera panes show selected stored images; live streams and Fetch remain unavailable.
Simulation uses a nominal 0.32 × 0.28 m
chassis and 0.05 m clearance, with conservative cell padding. This is not measured
robot geometry. ROS global/local costmaps, real localization/cameras and hardware
movement remain unconnected. Demo A* does not replace ROS1 move_base.

## Data and test map

- `web_runtime/settings.json`: shared display preferences and revision.
- `web_runtime/no_go_zones.json`: original map-frame rectangle corners and revision.
- `web_runtime/photo_index.json`, `web_runtime/images/`: reference/observation records,
  features, confirmed item/station links and UUID-named normalized PNG files. Limits:
  1000 photos / 500 items. Metadata binds acquisition pose, not storage-time pose.
- Browser localStorage `myagv:local-demo:layout:v1`: validated panel geometry only.

Simulated pose/goals/routes are runtime-only. `/api/navigation-map` returns the
derived source-plus-zones OccupancyGrid; `/api/global-costmap` returns its inflated
simulation grid and applied zone revision. Both preserve source origin/frame.
`/api/global-costmap.png` is an optional transparent overlay in the same display
coordinates. Small `telemetry` WebSocket messages update pose/path without
reloading the raster or resetting map pan/zoom.

Backend JSON uses atomic writes and revision checks. Photo writes have a separate
writer lock and run in a storage thread so STOP/navigation remain responsive.
Runtime data and dependencies are ignored by Git. Invalid saved data fails startup
instead of silently replacing it. The UI does not store credentials.

## LLM configuration

Settings > **Model** also offers **GPT-6.1 Sol · Local Codex**. Use Codex CLI
0.162.1 or newer; version 0.140.0 rejected this model in our deployment. This uses the
Mac's Codex CLI ChatGPT login through a loopback HTTP adapter and an SSH reverse
tunnel. It does not send Codex credentials to the robot or require an OpenRouter
key. Model access depends on the signed-in account: the CLI must successfully
complete inference before a model is considered verified. A model catalog entry
alone does not establish access.

On the Mac, install the web dependencies, install Codex CLI, and run `codex login`
with ChatGPT. Create a private bridge token once:

```bash
uv sync --extra web
mkdir -p ~/.config/myagv
chmod 700 ~/.config/myagv
python3 -c 'import os,secrets; p=os.path.expanduser("~/.config/myagv/codex-token"); fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600); os.write(fd,secrets.token_hex(32).encode()); os.close(fd)'
```

Copy **only this bridge token** over SSH to the robot's
`~/.config/myagv/codex-token`, with directory mode 700 and file mode 600. Keep
Codex's login credentials on the Mac. Configure a dedicated SSH key at
`~/.ssh/myagv-codex-tunnel` for unattended forwarding to `er@172.28.13.26`.
If public-key authentication is disabled, enable it for that account before
starting the tunnel. Our test machine has a `Match User er` override and a
backup at `/etc/ssh/sshd_config.before-myagv-codex`. Its authorized tunnel key
disables shell sessions and restricts remote listening to `127.0.0.1:9008`.
The Mac PM2 config runs these two processes:

```bash
pm2 start ecosystem.codex.config.cjs
pm2 save
```

The adapter listens only on `127.0.0.1:9008`. The tunnel exposes the same loopback
port on the robot. Robot API startup needs `MYAGV_CODEX_TOKEN_FILE` pointing to
its token file; `ecosystem.config.cjs` supplies the default path. Restart an
existing API process with that variable set, preserving its other environment:

```bash
MYAGV_CODEX_TOKEN_FILE="$HOME/.config/myagv/codex-token" pm2 restart myagv-api --update-env
pm2 save
```

Select Local Codex in Settings after deployment. Resolve, photo description,
location estimation, approach and ALIGN use the selected provider. ALIGN freezes
the model and effort for each task and retains the existing measured-Z gate,
image bounds checks, movement limits and STOP checks. Local Codex receives inline
images and an output schema over app-server stdio, with ephemeral threads, shell,
MCP, plugins, hooks and child agents disabled. ALIGN keeps one isolated process
and ephemeral thread per stage. The first turn supplies identity and GOAL;
later turns append only CURRENT and the last measured movement. BASE and ARM
never share a thread. Local Codex ALIGN requires persistent-session metadata;
both the caller and HTTP adapter reject requests that would fall back to one-shot
inference. Sequential turn IDs prevent duplicate or missing rounds
from silently losing context. Stage changes, task completion, STOP, errors and
60-second inference deadlines close the process; abandoned sessions expire after
180 seconds. Other requests use one process per request. The adapter accepts one
request at a time. Offline Macs, broken tunnels, unavailable models, quota errors,
timeouts and incomplete JSON fail explicitly; there is no provider fallback.
Configured means the bridge token was loaded, not that inference is healthy.
ALIGN allows 65 seconds for the HTTP request and 70 seconds in the hardware owner,
so the bridge's bounded inference deadline can finish before either outer timeout.
STOP and lease checks remain active while inference is pending. Deadline logs
record the RPC/inference phase and last protocol event, without image or prompt content.
Settings > **Fast mode**, below **Thinking**, controls the Codex tier and defaults
to on, including for existing saved settings. It auto-saves and applies to the next
task; a running ALIGN keeps its original tier across stages and rounds. The switch
is disabled for OpenRouter models. Turning it off explicitly requests Standard
for the adapter process, thread and turns, overriding inherited Fast defaults.
This leaves the user's global Codex configuration unchanged. Fast mode consumes
more subscription allowance; see [Codex speed](https://learn.chatgpt.com/docs/agent-configuration/speed).

Local Codex ARM alignment first restores the saved calibration GOAL Z within
1 mm. For a new GOAL, two independent X/Y probes of at most 5 mm use measured arm displacement
and visible image bounds to establish a local image-to-millimeter transform,
without manually recording object dimensions. Clipped targets first receive
coarse corrections; failed or ambiguous probes stop the task. GOAL bounds freeze
after the first measurement. Successful alignments save that transform in
`arm_metric_calibrations.json`, keyed by GOAL photo ID and recorded calibration Z.
Subsequent tests with the same GOAL and height reuse it without probe rounds,
including after a service restart. Changing the GOAL or height starts fresh.
Recapture GOAL after changing the camera mount or recorded object pose.
With a known transform and a horizontally clipped target, use the matching
unclipped side edge for a coarse X error; clipped views cannot confirm alignment.
Later model turns return only the full signed X/Y
residual and an alignment flag. The model requires planar error below 1 mm;
the controller independently finishes when `abs(X) + abs(Y) < 1.25 mm`, before
movement limits. Apply the full remaining X/Y correction with no gain reduction
or hardcoded 20 mm ceiling. `max_step_mm=0` disables the optional user step limit;
a positive value retains that explicit limit. X/Y share one complete coordinate
target, preserving Z, the fourth axis and gripper state. All motion still validates
the target workspace, forward-facing pose, measured completion, STOP and lease.
Z restoration also sends its full target once. Round logs expose residuals,
calibration reuse, thread IDs and cached input tokens.
The transform is local to the current object, pose and height; actual accuracy
still depends on image measurement and must be checked on hardware.

Official references: [Codex app-server](https://learn.chatgpt.com/docs/app-server)
and [Sign in with ChatGPT](https://developers.openai.com/siwc/token-sharing-open-source).
The latter is the recommended future route for a separately authorized app.

### OpenRouter

Settings > **Model** selects DeepSeek V4.1 Flash or V4 Pro 0813. **Thinking** selects
Off / Low / High / Max. Flash + Off is the default. Both choices auto-save after
400 ms to backend `settings.json`; Saved requires acknowledgement. They survive
reload/restart and work without a configured key. Display-only legacy settings gain
the defaults when loaded. Changing only LLM options preserves map pan/zoom and the
costmap overlay. Resolve waits for successfully saved settings and rejects late
results if configuration changes during inference.

Official `/api/v1/chat/completions` endpoint; latency routing and schema-support
requirement. Thinking responses are excluded. Total completion caps/timeouts:

| Thinking | Tokens (including reasoning) | Timeout |
| --- | --- | --- |
| Off | 384 | 15 s |
| Low | 2048 | 30 s |
| High | 4096 | 60 s |
| Max | 8192 | 120 s |

The [model catalog](https://openrouter.ai/api/v1/models) was checked on 2026-10-08:
both selected models allow `low`, `high`, `max` and optional reasoning. Generic
gateway effort names are not necessarily supported by each model. Thinking counts
toward the completion budget; incomplete results are rejected. See the
[reasoning documentation](https://openrouter.ai/docs/guides/best-practices/reasoning-tokens).
Only request text and registered names/appearance/IDs
are sent. Photo analysis/association and Fetch are separate milestones.

Ubuntu: the backend reads `llm_api_key` from `$CREDENTIALS_DIRECTORY` supplied by
the planned systemd encrypted-credential deployment. Encryption/provisioning is
not performed by this Mac milestone. When that directory is explicitly set but
unreadable, the backend fails closed rather than using an environment key.

For local development, provide `OPENROUTER_API_KEY` only to the backend. In zsh,
read it without displaying it or putting its literal value in shell history:

```zsh
read -rs 'OPENROUTER_API_KEY?OpenRouter key: '
export OPENROUTER_API_KEY
pm2 restart myagv-api --update-env
unset OPENROUTER_API_KEY
```

Environment variables are not encrypted storage. PM2 retains this development
environment in memory; keep keys out of `ecosystem.config.cjs`, `pm2 save`, logs
and frontend settings. The backend loads credentials once at startup. Settings
shows model/status only; Configured means loaded, not verified by OpenRouter.
Missing key/network leaves manual item selection usable. No key was provided for
implementation checks: mock tests passed; real provider behavior remains user acceptance.

`web_backend/map_data.py` generates a 240 × 200 OccupancyGrid, 0.05 m/cell, with
free/occupied/unknown cells, rotated walls, a divider and furniture. Its origin
has nonzero translation and 17° yaw; wall direction is independently rotated 21°.
The backend detects the dominant orthogonal direction and generates a separate
display PNG/origin. The original cells are unchanged. For weak wall evidence it
uses the original grid view; manual display angle handles ambiguous rooms.

The Demo has no remote-control authentication/TLS. Use network access only on a
trusted LAN or phone hotspot; do not expose it to the public internet.

## Necessary checks

```bash
.venv/bin/python -m unittest test_web_backend test_web_simulation test_web_photos test_web_resolve
npm --prefix web run build
npm --prefix web test
```

These check map/chassis geometry, conservative zone rasterization, inflation,
detour/no-path/deletion, simulated arrival, API guards/STOP/cancel/disconnect,
direct edits/persistence/revisions, photo source separation, delayed acquisition
pose binding, confirmed item/station links, storage failure cleanup and frontend types/build.
The resolver mock checks its outbound contract, ID/schema boundaries, unique/ambiguous/
no-match behavior, goal prerequisites, credential isolation, provider failure/timeout,
concurrency and STOP result rejection. No live OpenRouter inference was performed.
Model/thinking checks cover all offered choices, outgoing reasoning/token arguments,
legacy migration, persistence/restart, invalid/stale saves, unchanged raster/overlay
and cancellation on settings changes.
HTTP, PNG, static asset and WebSocket smoke checks passed for both
PM2 services. Browser visual/interaction checks are deliberately left to the user.
