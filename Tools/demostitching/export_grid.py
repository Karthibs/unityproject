"""Build DemoStitching routes between all grid cells and export them for Unity (NeroGridNavigator).

Dataset: one demonstration per cell, cell -> rest position ("Grid Rest").
For every ordered pair of cells (i, j):
  * direct: one chained DS from cell i to cell j (graph path through demo i and the
    reversed demo j); accepted if the Python rollout converges and stays inside the
    safe workspace ring,
  * otherwise via_rest: two chained DSs, cell i -> rest and rest -> cell j.

Usage: python export_grid.py <dataset_rel_path> <scale> <out_unity_json> [rmin rmax]
"""
import json, math, os, sys, time, contextlib, io
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
rmin = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
rmax = float(sys.argv[5]) if len(sys.argv) > 5 else 0.50
BASE = np.array([0.0, -0.38])          # robot base (unity x, z)
np.random.seed(42)

plan = json.load(open(os.path.join(ds_path, "demo_plan.json")))
cfg = StitchConfig(dataset_path=ds_path, ds_method="chain", data_position_scale=scale, seed=42)
demo_set = get_demonstration_set(ds_path, position_scale=scale, velocity_scale=scale)
cells = [d["start_object"].replace("Cell ", "") for d in plan["demonstrations"]]

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
print(f"fit: {len(cells)} demos, K {[d.damm.K for d in ds_set]}, {gg}, {time.time()-t0:.0f}s", flush=True)

starts = [np.mean([t.x[0] for t in d.trajectories], axis=0) for d in demo_set]
rest = np.mean([t.x[-1] for d in demo_set for t in d.trajectories], axis=0)
seg_cache = {}


def flat(a):
    out = []
    def rec(x):
        if isinstance(x, (list, tuple, np.ndarray)):
            for y in x:
                rec(y)
        else:
            v = float(x)
            out.append(v if math.isfinite(v) else -1.0)
    rec(a)
    return out


SPEED, FLOOR, VMAX = 2.0, 0.04, 0.40     # same time scaling / speed floor / speed cap as NeroGridNavigator
MAX_SWITCHES = 4                         # demo changes allowed along the graph path
N_PERTURB, PERTURB = 4, 0.01             # extra rollouts from perturbed starts (+-1 cm)


def robust_rollout(chain, x0, att):
    """Rollout like the Unity runtime (time scaled, speed floor, speed cap). Returns (ok, reason)."""
    chain.reset_runtime(0, 0.0)
    x = np.asarray(x0, float).copy()
    t = 0.0
    for _ in range(int(25.0 / 0.01)):
        d = np.linalg.norm(x - att) / scale
        if d < 0.006:
            return True, ""
        _, v, _ = chain.step_once(x, dt=0.0, current_time=t * SPEED)
        v = np.asarray(v, float) * SPEED / scale            # m/s
        sp = np.linalg.norm(v)
        if d < 0.02 and sp < 0.03 * SPEED:
            return True, ""
        if not np.all(np.isfinite(v)):
            return False, "non-finite"
        if d > 0.04 and 1e-9 < sp < FLOOR:
            v *= FLOOR / sp
        if sp > VMAX:
            v *= VMAX / sp
        x = x + 0.01 * v * scale
        t += 0.01
        r = np.linalg.norm(x[:2] / scale - BASE)
        if r < rmin or r > rmax or not (0.82 <= x[2] / scale <= 1.0):
            return False, f"leaves workspace (r {r:.2f})"
    return False, "timeout"


def build(init, att, robust=True):
    """Chained DS from init to att (DS frame) + Python rollout check. Returns (task_dict, info) or (None, reason)."""
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            chain, _, path, stats = construct_stitched_ds(cfg, gg, norm_demo_set, ds_set, reversed_ds_set, init, att,
                                                          segment_ds_lookup=seg_cache)
        except Exception as e:  # noqa: BLE001
            return None, f"exception {type(e).__name__}"
    if not isinstance(chain, ChainedSegmentedDS):
        return None, "construction failed"
    with contextlib.redirect_stdout(io.StringIO()):
        chain.max_iter = 3000
        traj, _ = chain.sim(init[None, :], dt=0.01)
    err = float(np.linalg.norm(traj[-1] - att)) / scale
    xz = traj[:, :2] / scale
    r = np.linalg.norm(xz - BASE[None, :], axis=1)
    h = traj[:, 2] / scale
    if err > 0.01:
        return None, f"not converged ({err*1000:.0f} mm)"
    if r.min() < rmin or r.max() > rmax:
        return None, f"leaves workspace (r {r.min():.2f}-{r.max():.2f} m)"
    if h.min() < 0.82 or h.max() > 1.0:
        return None, f"height out of range ({h.min():.2f}-{h.max():.2f} m)"
    demos_on_path = [int(n[0]) for n in path]
    switches = sum(1 for a, b in zip(demos_on_path, demos_on_path[1:]) if a != b)
    if robust and switches > MAX_SWITCHES:
        return None, f"{switches} demo switches"
    if robust:
        rng = np.random.default_rng(0)
        for k in range(N_PERTURB + 1):
            x0 = init.copy()
            if k > 0:
                x0[:2] += rng.uniform(-PERTURB, PERTURB, 2) * scale
            ok, why = robust_rollout(chain, x0, att)
            if not ok:
                return None, f"not robust ({why}, start {k})"
    tr = {
        "ratioStartNodes": flat(chain.transition_ratio_start_nodes), "ratioNodes": flat(chain.transition_ratio_nodes),
        "edgeRatios": flat(chain.transition_edge_ratios), "times": flat(chain.transition_times),
    }
    task = {
        "initial": flat(init), "attractor": flat(att), "pathDemos": [int(n[0]) for n in path],
        "segments": [{
            "xAtt": flat(np.asarray(s.x_att).reshape(-1)), "K": len(s.damm.gaussian_lists),
            "priors": flat([g["prior"] for g in s.damm.gaussian_lists]),
            "mus": flat([g["mu"] for g in s.damm.gaussian_lists]),
            "sigmas": flat([g["sigma"] for g in s.damm.gaussian_lists]),
            "A": flat(s.A),
        } for s in chain.segment_ds_seq],
        "referenceRollout": flat(traj[:: max(1, len(traj) // 200)]),
        "referenceFinalErrorM": err, **tr,
    }
    return task, {"steps": len(traj), "err": err, "rmin": float(r.min()), "rmax": float(r.max())}


tasks, routes = [], []


def add_task(task, name, kind):
    task.update({"name": name, "kind": kind, "startObject": "", "endObject": ""})
    tasks.append(task)
    return len(tasks) - 1


def primitive_task(lpvds, init, att):
    """One-segment chain from a per-demo LPV-DS (DemoStitching's demo / reversed-demo model)."""
    x = np.asarray(init, float).reshape(1, -1)
    traj = [x[0].copy()]
    for _ in range(4000):
        if np.linalg.norm(x[0] - att) < 0.01 * scale:
            break
        v, _ = lpvds.velocity(x)
        if not np.all(np.isfinite(v)):
            return None, "non-finite"
        x = x + 0.01 * v.T
        traj.append(x[0].copy())
    traj = np.array(traj)
    err = float(np.linalg.norm(traj[-1] - att)) / scale
    r = np.linalg.norm(traj[:, :2] / scale - BASE[None, :], axis=1)
    if err > 0.012 or r.min() < rmin or r.max() > rmax:
        return None, f"err {err*1000:.0f} mm, r {r.min():.2f}-{r.max():.2f}"
    gl = lpvds.damm.gaussian_lists
    return {
        "initial": flat(init), "attractor": flat(np.asarray(lpvds.x_att).reshape(-1)), "pathDemos": [],
        "segments": [{"xAtt": flat(np.asarray(lpvds.x_att).reshape(-1)), "K": len(gl),
                      "priors": flat([g["prior"] for g in gl]), "mus": flat([g["mu"] for g in gl]),
                      "sigmas": flat([g["sigma"] for g in gl]), "A": flat(lpvds.A)}],
        "ratioStartNodes": [], "ratioNodes": [], "edgeRatios": [], "times": [],
        "referenceRollout": flat(traj[:: max(1, len(traj) // 200)]), "referenceFinalErrorM": err,
    }, f"{len(traj)} steps"


# Per-cell demo models: cell -> rest (demo) and rest -> cell (reversed demo). Used as the
# via-rest route and as the runtime safety fallback in Unity.
to_rest, from_rest = {}, {}
for i, ci in enumerate(cells):
    t1, w1 = primitive_task(ds_set[i], starts[i], np.asarray(ds_set[i].x_att).reshape(-1))
    t2, w2 = primitive_task(reversed_ds_set[i], rest, starts[i])
    to_rest[i] = add_task(t1, f"{ci}->rest", "demonstrated") if t1 else None
    from_rest[i] = add_task(t2, f"rest->{ci}", "reversed") if t2 else None
    if t1 is None or t2 is None:
        print(f"  demo model problem for {ci}: to rest {w1}, from rest {w2}", flush=True)
print(f"demo models: {sum(v is not None for v in to_rest.values())}/18 to rest, "
      f"{sum(v is not None for v in from_rest.values())}/18 from rest", flush=True)

n_direct = n_via = n_fail = 0
reasons = {}
t0 = time.time()
for i, ci in enumerate(cells):
    for j, cj in enumerate(cells):
        if i == j:
            continue
        task, info = build(starts[i], starts[j])
        if task is not None:
            routes.append({"start": ci, "end": cj, "kind": "direct", "chains": [add_task(task, f"{ci}->{cj}", "stitched")]})
            n_direct += 1
            continue
        key = info.split(" (")[0]
        reasons[key] = reasons.get(key, 0) + 1
        if to_rest[i] is not None and from_rest[j] is not None:
            routes.append({"start": ci, "end": cj, "kind": "via_rest", "chains": [to_rest[i], from_rest[j]]})
            n_via += 1
        else:
            n_fail += 1
            print(f"  NO ROUTE {ci}->{cj}: {info}", flush=True)
    print(f"  {ci}: done ({n_direct} direct, {n_via} via rest, {n_fail} failed, {time.time()-t0:.0f}s)", flush=True)
print("direct-chain rejections by reason:", reasons, flush=True)

out = {
    "format": "nero_grid_routes_v1", "scale": scale, "axisPerm": [0, 2, 1], "dtReference": 0.01,
    "velocityMax": 0.0, "tasks": tasks, "routes": routes,
    "restFallback": {c: [to_rest[i], from_rest[i]] for i, c in enumerate(cells)},
}
assert cfg.chain.transition_trigger_method == "distance_ratio"
json.dump(out, open(out_json, "w"))
print(f"routes: {n_direct} direct, {n_via} via rest, {n_fail} failed; {len(tasks)} chains -> {out_json}")
