"""Build DemoStitching chained DSs for the NERO tabletop tasks and export them for Unity.

For every (start object, end tray) task it:
  1. fits per-demo LPV-DS + the Gaussian graph (repo code, same as main_stitch),
  2. builds the chained DS (ds_method="chain", ChainedSegmentedDS),
  3. simulates it from the task start (Python reference rollout),
  4. writes a JSON the Unity runtime (NeroChainedDS.cs) evaluates.

DS frame: p = scale * (x_unity, z_unity, y_unity);  v_unity_mps = perm^-1(v_ds) / scale.

Usage: python export_chain.py <dataset_rel_path> <scale> <out_json> [seed]
"""
import json, os, sys, time, contextlib, io
import numpy as np

REPO = os.path.expanduser("~/DemoStitching")
os.environ.setdefault("MPLBACKEND", "Agg")
os.chdir(REPO)
sys.path.insert(0, REPO)

from configs import StitchConfig
import src.graph_utils as gu
from src.util.load_tools import get_demonstration_set
from src.util.ds_tools import apply_lpvds_demowise, get_gaussian_directions
from src.stitching.ds_stitching import construct_stitched_ds
from src.stitching.chaining import ChainedSegmentedDS

ds_path, scale, out_json = sys.argv[1], float(sys.argv[2]), sys.argv[3]
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 42
np.random.seed(seed)

plan = json.load(open(os.path.join(ds_path, "demo_plan.json")))
cfg = StitchConfig(dataset_path=ds_path, ds_method="chain", data_position_scale=scale, seed=seed)
demo_set = get_demonstration_set(ds_path, position_scale=scale, velocity_scale=scale)

t0 = time.time()
with contextlib.redirect_stdout(io.StringIO()):
    ds_set, reversed_ds_set, norm_demo_set = apply_lpvds_demowise(demo_set, cfg.damm)
gaussians = {
    (i, j): {"mu": mu, "sigma": s, "direction": d, "prior": p}
    for i, ds in enumerate(ds_set)
    for j, (mu, s, d, p) in enumerate(zip(ds.damm.Mu, ds.damm.Sigma, get_gaussian_directions(ds), ds.damm.Prior))
}
gg = gu.GaussianGraph(param_dist=cfg.param_dist, param_cos=cfg.param_cos, bhattacharyya_threshold=cfg.bhattacharyya_threshold)
gg.add_gaussians(gaussians, reverse_gaussians=cfg.reverse_gaussians)
print(f"fit: K per demo {[d.damm.K for d in ds_set]}, graph {gg}, {time.time()-t0:.1f}s")

demos = plan["demonstrations"]
# Unique task endpoints: group demo starts by start object and demo ends by end object.
start_pts, end_pts = {}, {}
for d, dm in zip(demo_set, demos):
    start_pts.setdefault(dm["start_object"], []).extend(t.x[0] for t in d.trajectories)
    end_pts.setdefault(dm["end_object"], []).extend(t.x[-1] for t in d.trajectories)
start_objs = list(start_pts)
end_objs = list(end_pts)
starts = {o: np.mean(start_pts[o], axis=0) for o in start_objs}
ends = {o: np.mean(end_pts[o], axis=0) for o in end_objs}
demonstrated = {(dm["start_object"], dm["end_object"]) for dm in demos}


def seg_payload(seg):
    gl = seg.damm.gaussian_lists
    return {
        "x_att": np.asarray(seg.x_att, float).reshape(-1).tolist(),
        "priors": [float(g["prior"]) for g in gl],
        "mus": [np.asarray(g["mu"], float).tolist() for g in gl],
        "sigmas": [np.asarray(g["sigma"], float).tolist() for g in gl],
        "A": [np.asarray(a, float).tolist() for a in seg.A],
    }


tasks = []
for so in start_objs:
    for eo in end_objs:
        name = f"{so} -> {eo}"
        kind = "demonstrated" if (so, eo) in demonstrated else "stitched"
        init, att = starts[so], ends[eo]
        with contextlib.redirect_stdout(io.StringIO()):
            res = construct_stitched_ds(cfg, gg, norm_demo_set, ds_set, reversed_ds_set, init, att)
        chain, _, path, stats = res
        if not isinstance(chain, ChainedSegmentedDS):
            print(f"[{kind}] {name}: construction FAILED ({stats.get('construction_error')})")
            tasks.append({"name": name, "kind": kind, "ok": False, "error": str(stats.get("construction_error"))})
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            traj, _ = chain.sim(init[None, :], dt=0.01)
        dist = float(np.linalg.norm(traj[-1] - att)) / scale
        converged = dist < 0.01
        demo_ids = [int(n[0]) for n in path]
        print(f"[{kind}] {name}: path demos {demo_ids}, {chain.n_systems} subsystems, "
              f"sim {len(traj)} steps, final error {dist*1000:.1f} mm -> {'OK' if converged else 'NOT CONVERGED'}")
        assert not chain.has_init_boundary and not chain.has_attractor_boundary, "boundary systems not exported"
        tasks.append({
            "name": name, "kind": kind, "ok": bool(converged),
            "start_object": so, "end_object": eo,
            "initial": init.tolist(), "attractor": att.tolist(),
            "path_demos": demo_ids,
            "segments": [seg_payload(s) for s in chain.segment_ds_seq],
            "transition": {
                "ratio_start_nodes": np.asarray(chain.transition_ratio_start_nodes).tolist(),
                "ratio_nodes": np.asarray(chain.transition_ratio_nodes).tolist(),
                "edge_ratios": np.asarray(chain.transition_edge_ratios).tolist(),
                "times": np.asarray(chain.transition_times).tolist(),
            },
            "reference_rollout": traj[:: max(1, len(traj) // 400)].tolist(),
            "reference_final_error_m": dist,
        })

out = {
    "format": "nero_chained_ds_v1",
    "frame": "ds = scale * (unity_x, unity_z, unity_y)",
    "scale": scale,
    "axis_perm": [0, 2, 1],
    "dt_reference": 0.01,
    "trigger": cfg.chain.transition_trigger_method,
    "velocity_max": cfg.chain.velocity_max,
    "dataset": ds_path,
    "tasks": tasks,
}
os.makedirs(os.path.dirname(os.path.abspath(out_json)), exist_ok=True)
json.dump(out, open(out_json, "w"))
ok = sum(t["ok"] for t in tasks)
print(f"exported {ok}/{len(tasks)} converged tasks -> {out_json}")
