# DemoStitching pipeline for the NERO tabletop (step 3)

Python env: `~/anaconda3/envs/demostitch` (Python 3.9, repo requirements). Repo: `~/DemoStitching`.
Run every script with `~/anaconda3/envs/demostitch/bin/python -s <script> ...`.

1. Record demos in Unity (Demo recorder panel) -> `DemoData/nero_tabletop/` (Unity world frame, m, m/s).
2. `check_demos.py DemoData/nero_tabletop out.png` — sanity checks + plot.
3. `convert_xzy.py DemoData/nero_tabletop ~/DemoStitching/dataset/stitching/nero_tabletop_xzy`
   — reorders axes to (x, z, y_up) so dims 0/1 are the table plane, trims the stationary end tail.
4. `probe_graph.py dataset/stitching/nero_tabletop_xzy 1 5 15` — Gaussian-graph connectivity per scale.
5. `run_stitch.py chain <out_dir> 0 5` — full DemoStitching evaluation (all start/goal permutations).
6. `export_chain.py dataset/stitching/nero_tabletop_xzy 15 DemoData/chained_ds_nero.json`, then `to_unity_json.py DemoData/chained_ds_nero.json DemoData/chained_ds_nero_unity.json`
   — builds the chained DS for every cube->tray task and exports it for Unity.
7. `plot_export.py DemoData/chained_ds_nero.json DemoData/nero_tabletop out.png`

Chosen setting: `ds_method="chain"`, `data_position_scale=15` (DS frame = 15 * (x, z, y)). Demos v2: crossing hub at x=-0.13 (0.32 m from base) to stay clear of the arm's stretched-out singularity.

## Grid navigation (robot on the long side, 2026-11-06)

The reachable half-ring in front of the robot is split into 3 rings x 6 sectors = 18 cells
(A1..C6, `NeroGridNavigator`). One demonstration per cell goes cell -> rest position.

1. Record: Demo recorder configured with 18 `Cell Xn -> Grid Rest` demos, path shape `PolarDirect`,
   4 trajectories each -> `DemoData/nero_grid/`.
2. `convert_xzy.py DemoData/nero_grid ~/DemoStitching/dataset/stitching/nero_grid_xzy`
3. `export_grid.py dataset/stitching/nero_grid_xzy 15 <abs path>/DemoData/chained_ds_grid_unity.json`
   — for every ordered cell pair: one stitched chain (demo i, then reversed demo j) if its rollout
   converges and stays inside r = 0.25..0.50 m; otherwise two chains via the rest position.
4. In Unity: grid panel (top left of the Game view) -> pick START and END cell -> GO.
