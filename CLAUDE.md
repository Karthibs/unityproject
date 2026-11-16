# SSY226: DemoStitching on the AgileX NERO arm (Unity)

Course project (SSY226). It implements **DemoStitching** (https://github.com/KilianFt/DemoStitching: chained
LPV-DS over a Gaussian graph built from demonstrations) on a simulated AgileX NERO 7-DOF arm in Unity.
The robot learns from a few demonstrations and then moves between new start/goal pairs by stitching them together.

## Current state (2026-11-16)

The DemoStitching implementation is done and verified. The user is happy with it.

- **Scene:** `Assets/NeroTabletop.unity`. The robot is on the long side of the table: base at Unity
  world (0, 0.74, -0.38), root yaw 168°. There are 4 cubes on the left and 2 trays on the right.
- **Grid navigation (main feature):** the reachable half-ring in front of the robot is split into
  3 rings × 6 sectors = 18 cells, named A1..C6.
  - Rings: r = 0.30 / 0.37 / 0.44 m around (x, z) = (0, -0.38).
  - Sectors: -75..75° in steps of 30°.
  - Rest position: (0, 0.86, -0.02). Travel height: 0.86 m.
  - There is one demonstration per cell, cell → rest, with 4 trajectories each.
  - Python builds routes for all 306 ordered pairs:
    - **78 direct** stitched chains, accepted only if robust.
    - **228 via rest**: cell → rest with the demo LPV-DS, then rest → cell with the reversed demo LPV-DS.
- **UI:** press Play. The grid panel is at the top left of the Game view: pick START, pick END, press GO.
  The old pick-and-place button panels are hidden.
- **Results:**
  - Random benchmark: 20/20 arrived.
  - 36-route test set: 36/36 arrived, of which 11 are direct and 25 via rest.
  - Deviation from the planned path: median 2.6 mm (mean over the path) and 6.5 mm (maximum).
  - Median duration is 4.0 s. End error is 3.8–4.0 mm on every route, which is the tolerance of the final precise move.
  - One safety fallback: **A6 → C5**. Its direct chain detours toward C2, so the runtime safety net finished it via rest.
- **Plots** are in `Verification/plots/` (fig1–fig5 plus `test_metrics.csv`), made by `Tools/demostitching/plot_grid_results.py`.

## Repo layout

| Path | What |
|---|---|
| `Assets/NeroTabletop.unity` | Main scene. All Nero components are on the `NERO Arm` GameObject; `TabletopReset` is on `Workspace` |
| `Assets/Scripts/NeroCartesianController.cs` | IK and joint control (see below) |
| `Assets/Scripts/NeroChainedDS.cs` | Exact C# port of DemoStitching's `ChainedSegmentedDS`. Matches the Python rollout to 0.000 mm |
| `Assets/Scripts/NeroGridNavigator.cs` | Grid, route loading, route execution with safety net, OnGUI grid panel, benchmarks, run logging |
| `Assets/Scripts/NeroDemoRecorder.cs` | Records demonstrations to `DemoData/<name>/demonstration_i/trajectory_j.json` (Unity world frame, m, m/s) |
| `Assets/Scripts/NeroPickPlace.cs`, `NeroStitchedTransport.cs` | Earlier stage: pick-and-place with 2 stitched demos, robot on the short side. Panels hidden |
| `Assets/Scripts/NeroDiagLogger.cs`, `NeroReachTest.cs` | Diagnostics. Not in the scene |
| `Assets/Editor/NeroTabletopBuilder.cs` | Original scene builder (from the initial project) |
| `DemoData/nero_grid/` | 18 grid demos × 4 trajectories |
| `DemoData/chained_ds_grid_unity.json` | **The route export the navigator loads** (v2, robust). `*.v1_repaired`, `*.before_repair` and `*.v2` are backups |
| `DemoData/nero_tabletop_v2_shortside/` etc. | Backups of the earlier pick-and-place demos |
| `Tools/demostitching/` | Python pipeline. Its `README.md` has the step-by-step commands. `requirements.txt` has the exact versions |
| `Verification/` | Logs, benchmark (`grid_benchmark.txt`), executed paths (`grid_runs/S_E.csv`), plots |

## Controller (`NeroCartesianController`): do not regress these

- DLS IK is evaluated at the **commanded** configuration, with FK on the measured chain. IK on the measured state caused a limit cycle.
- Null-space posture aims joint 1 at the target azimuth, which avoids IK local minima. Joints near their limits are locked.
- Velocity mode uses orientation weight 0.3. `Mode.Joint` is a synchronised joint-space move.
- `GoToNatural` = SolveIKNatural → joint-space move → Pose settle. Before any Cartesian motion, it
  **pre-positions in joint space into the natural posture with the natural gripper yaw**
  (`NaturalYawDeg`, `followNaturalYaw`). Without it the wrist flips and motions shiver: tracking error was 350 mm, versus 2.6 mm with it.
- Physics: 100 Hz fixed step, TGS solver, 12 position / 4 velocity iterations. Lower values let the fingers penetrate the cubes.

## DemoStitching pipeline (Python)

- Settings:
  - `ds_method="chain"`, `data_position_scale=15`, transition trigger `distance_ratio`.
  - DS frame = 15 × (x, z, y_up): `convert_xzy.py` reorders the axes so dims 0 and 1 are the table plane and trims the stationary tails. The export stores `axisPerm [0,2,1]`.
- `export_grid.py` accepts a direct chain only if all of these hold:
  - its rollout converges;
  - it stays inside r = 0.25–0.50 m and height 0.82–1.0 m;
  - it has ≤ 4 demo switches;
  - it is robust to 4 starts perturbed by ±1 cm, using the same time scaling and speed floor as Unity.
  Otherwise the route goes via rest using per-demo models.
- Runtime settings (in `NeroGridNavigator`, saved in the scene): `speedScale 2`, `minPathSpeed 0.04` (star-shaped demos stall at the hub without this floor),
  `handoffDistance 0.035` (the last 3.5 cm is a precise Pose move), `legTimeout 25`, `safeRadius (0.26, 0.49)`,
  `abortSpeed 0.6`. Safety net: if a chain leaves the safe ring or times out, the leg is aborted and finished via rest.
- Scripts `chdir` into `~/DemoStitching` and expect the converted datasets in
  `~/DemoStitching/dataset/stitching/nero_grid_xzy` (and `nero_tabletop_xzy`). Paths passed as output must be **absolute**.

Rebuild routes and plots (env python = `<conda env>/bin/python`, run with `-s`):
```bash
python -s Tools/demostitching/convert_xzy.py DemoData/nero_grid ~/DemoStitching/dataset/stitching/nero_grid_xzy
python -s Tools/demostitching/export_grid.py dataset/stitching/nero_grid_xzy 15 "$PWD/DemoData/chained_ds_grid_unity.json"
python -s Tools/demostitching/plot_grid_results.py "$PWD" "$PWD/Verification/plots"
```
Refitting on another machine can give slightly different clusters, and so a slightly different direct/via-rest split.
Back up the current JSON before overwriting it.

## Testing in Unity

- **Manual:** use the grid panel.
- **Scripted:** in Play mode, call
  `Object.FindFirstObjectByType<NeroGridNavigator>().RunRandom(20)` or `RunPairs("A1-B2,...")`, for example through Unity MCP.
  - Each run writes `Verification/grid_runs/S_E.csv`. The header line is `# start=.. end=.. kind=.. chains=.. arrived=.. fallback=.. duration_s=.. end_error_m=..`, followed by `x,y,z` rows at dt 0.01 in the Unity world frame.
  - A list run also writes `Verification/grid_benchmark.txt`.
- The 36-route test set used for the plots:
  `A1-B2,A2-B3,A3-B4,A4-B5,A5-B6,A6-C1,B1-C2,B2-C3,B3-C4,B4-C5,B5-C6,B6-A1,C1-A2,C2-A3,C3-A4,C4-A5,C5-A6,C6-B1,A1-B6,A2-C1,A3-C2,A4-C3,A5-C4,A6-C5,B1-C6,B2-A1,B3-A2,B4-A3,B5-A4,B6-A5,C1-A6,C2-B1,C3-B2,C4-B3,C5-B4,C6-B5`

## Unity MCP workflow (com.unity.ai.assistant relay)

- The MCP server only loads if the Unity Editor is open when the Claude session starts. Otherwise reconnect with `/mcp`.
- Every script recompile and every enter/exit of Play mode drops the connection ("Unity not detected").
  Wait until `tail -2 Logs/relay.txt` shows `connection.established`, then retry.
- Stop Play mode before recompiling. Scripts recompiled during Play leave components in a bad state.
- In Play mode a call can hang until the user interacts with the Unity window. If a call hangs for more than 2 minutes, ask the user to click into Unity.
- Deleting files inside RunCommand triggers an approval prompt that fails over MCP. Delete from the shell instead.
- `Unity_Camera_Capture` failed in this project. To get an image, render a Camera to a RenderTexture and save a PNG.
- `GameObject.Find("link1")` can hit a visual mesh with the same name. Get the ArticulationBodies from the robot's `GetComponentsInChildren` instead.
- Set Unity's AI Assistant **Interaction Mode** to "No Throttling". It is a per-machine editor pref.
- **Save the scene** (Ctrl/Cmd+S) after editing it through MCP. Unsaved changes do not reach git.

## First-time setup on a new machine (macOS)

1. **Unity:** install editor **6000.3.25f1** (Apple silicon build) through Unity Hub, add this folder as a project, and open it.
   The first import rebuilds `Library/` and takes several minutes. Open `Assets/NeroTabletop.unity` and press Play.
   The grid panel should appear. Only the route JSON is needed: the Unity side does not need Python.
2. **Python (only for refitting or plots):**
   ```bash
   # Miniforge/conda, Python 3.9 (the env used so far was 3.9.25)
   conda create -n demostitch python=3.9 -y
   conda run -n demostitch pip install -r Tools/demostitching/requirements.txt
   git clone https://github.com/KilianFt/DemoStitching ~/DemoStitching
   conda run -n demostitch python -s Tools/demostitching/convert_xzy.py DemoData/nero_grid ~/DemoStitching/dataset/stitching/nero_grid_xzy
   ```
   The DemoStitching repo itself was not modified, so a plain clone is enough.
   `Tools/demostitching/README.md` says `~/anaconda3/envs/demostitch`. Use whatever path conda gives on the Mac.
3. **Unity MCP:** enable the MCP server in Unity's AI Assistant settings. It installs a relay under `~/.unity/relay/`.
   On Linux this was `relay_linux --mcp`; on macOS use the Mac binary (`ls ~/.unity/relay`):
   `claude mcp add unity-mcp -s user -- ~/.unity/relay/<mac relay binary> --mcp`. Then set Interaction Mode to "No Throttling".
4. **Claude Code:** the user allowed running the env's python without prompts. On Linux the rule was
   `Bash(/home/karthibs/anaconda3/envs/demostitch/bin/python *)` in `.claude/settings.local.json`, which is not in git. Re-create it with the Mac path.
5. **Re-verify:** physics on ARM is not bit-identical to x86. Re-run the 36-route test set, then regenerate the plots
   before quoting numbers measured on the Mac.

## Open items / possible next steps

- **VLM with a depth camera** (user asked; not started). Two questions are still open for the user:
  - Camera placement: fixed top-down, wrist-mounted, or both?
  - Which VLM: a cloud model or a local one (e.g. Qwen2.5-VL)?
  `Assets/ObservationRGB.renderTexture` exists from the initial setup. A depth image can come from a Camera with `DepthTextureMode.Depth` plus a shader or replacement pass.
- Pick-and-place on top of the grid routes. A denser grid.

## Conventions

- Commit only when the user asks. Plots follow the dataviz skill:
  - Palette slots 1–3: blue `#2a78d6`, orange `#eb6834`, aqua `#1baf7a`.
  - Surface `#fcfcfb`.
  - One measure per panel, and every figure has a legend.
- The original project came from github.com/jinlungao/SSY226_SIMULATION (the first two commits). The user's own copy is github.com/Karthibs/unityproject.
