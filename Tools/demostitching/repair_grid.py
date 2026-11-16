"""Fill in grid routes that export_grid.py could not build, using other DemoStitching options.

For each missing (start, end) pair it tries, in order:
  1. chain, triplet_fit_data_mode = all_nodes
  2. chain_all (re-cluster Gaussians along the path)
  3. the same two variants for the rest -> end leg (route via rest)
and reports why candidates fail (convergence / workspace / stability of the segment A matrices).

Usage: python repair_grid.py <dataset_rel_path> <scale> <unity_json (read + updated in place)>
"""
import json, os, sys, copy, contextlib, io
import numpy as np

sys.argv_backup = list(sys.argv)
ds_path, scale, path_json = sys.argv[1], float(sys.argv[2]), sys.argv[3]
# Reuse the fitting + build machinery of export_grid.py (fits the same seeded models).
sys.argv = [sys.argv[0], ds_path, str(scale), "/dev/null"]
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "export_grid.py")).read()
src = src[: src.index("tasks, routes = [], []")]          # only the setup + build() part
exec(compile(src, "export_grid.py", "exec"))

data = json.load(open(path_json))
have = {(r["start"], r["end"]) for r in data["routes"]}
missing = [(ci, cj) for ci in cells for cj in cells if ci != cj and (ci, cj) not in have]
print(f"{len(missing)} missing routes: {sorted(set(e for _, e in missing))} as end cells", flush=True)


def stability_report(init, att):
    with contextlib.redirect_stdout(io.StringIO()):
        chain, _, path, _ = construct_stitched_ds(cfg, gg, norm_demo_set, ds_set, reversed_ds_set, init, att,
                                                  segment_ds_lookup={})
    if chain is None:
        return "no chain"
    worst = []
    for s in chain.segment_ds_seq:
        P = getattr(s.ds_opt, "P", np.eye(3))
        w = max(float(np.max(np.linalg.eigvalsh(A.T @ P + P @ A))) for A in s.A)
        worst.append(round(w, 4))
    return f"path demos {[int(n[0]) for n in path]}, max eig(A^T P + P A) per segment {worst} (must be < 0)"


variants = [("chain/all_nodes", dict(ds_method="chain", triplet="all_nodes")),
            ("chain_all", dict(ds_method="chain_all", triplet=None))]
base_cfg = copy.deepcopy(cfg)


def try_variants(init, att):
    global cfg, seg_cache
    for name, v in variants:
        cfg = copy.deepcopy(base_cfg)
        cfg.ds_method = v["ds_method"]
        if v["triplet"]:
            cfg.chain.triplet_fit_data_mode = v["triplet"]
        seg_cache = {}
        task, info = build(init, att)
        if task is not None:
            cfg = base_cfg
            return task, name
        print(f"     {name}: {info}", flush=True)
    cfg = base_cfg
    return None, None


idx = {c: k for k, c in enumerate(cells)}
end_cells = sorted(set(e for _, e in missing))


def primitive_task(lpvds, init, att, name, kind):
    """One-segment chain from a per-demo LPV-DS (DemoStitching's own demo / reversed-demo model)."""
    class _C:  # minimal stand-in so build()'s payload code can be reused
        pass
    with contextlib.redirect_stdout(io.StringIO()):
        x = np.asarray(init, float).reshape(1, -1)
        traj = [x[0].copy()]
        for _ in range(3000):
            if np.linalg.norm(x[0] - att) < 0.01 * scale:   # 1 cm in metres
                break
            v, _ = lpvds.velocity(x)
            if not np.all(np.isfinite(v)):
                return None, "non-finite velocity"
            x = x + 0.01 * v.T
            traj.append(x[0].copy())
    traj = np.array(traj)
    err = float(np.linalg.norm(traj[-1] - att)) / scale
    r = np.linalg.norm(traj[:, :2] / scale - BASE[None, :], axis=1)
    if err > 0.012:
        return None, f"not converged ({err*1000:.0f} mm)"
    if r.min() < rmin or r.max() > rmax:
        return None, f"leaves workspace (r {r.min():.2f}-{r.max():.2f})"
    gl = lpvds.damm.gaussian_lists
    task = {
        "name": name, "kind": kind, "startObject": "", "endObject": "",
        "initial": flat(init), "attractor": flat(np.asarray(lpvds.x_att).reshape(-1)), "pathDemos": [],
        "segments": [{"xAtt": flat(np.asarray(lpvds.x_att).reshape(-1)), "K": len(gl),
                      "priors": flat([g["prior"] for g in gl]), "mus": flat([g["mu"] for g in gl]),
                      "sigmas": flat([g["sigma"] for g in gl]), "A": flat(lpvds.A)}],
        "ratioStartNodes": [], "ratioNodes": [], "edgeRatios": [], "times": [],
        "referenceRollout": flat(traj[:: max(1, len(traj) // 200)]), "referenceFinalErrorM": err,
    }
    return task, f"{len(traj)} steps, err {err*1000:.1f} mm, r {r.min():.2f}-{r.max():.2f}"


fixed = 0
prim_to_rest, prim_from_rest = {}, {}
for ci, cj in missing:
    i, j = idx[ci], idx[cj]
    if i not in prim_to_rest:
        t1, info1 = primitive_task(ds_set[i], starts[i], np.asarray(ds_set[i].x_att).reshape(-1), f"{ci}->rest (demo model)", "demonstrated")
        print(f"   primitive {ci}->rest: {info1}", flush=True)
        if t1: data["tasks"].append(t1)
        prim_to_rest[i] = len(data["tasks"]) - 1 if t1 else None
    if j not in prim_from_rest:
        t2, info2 = primitive_task(reversed_ds_set[j], rest, starts[j], f"rest->{cj} (reversed demo model)", "reversed")
        print(f"   primitive rest->{cj}: {info2}", flush=True)
        if t2: data["tasks"].append(t2)
        prim_from_rest[j] = len(data["tasks"]) - 1 if t2 else None
    if prim_to_rest[i] is not None and prim_from_rest[j] is not None:
        data["routes"].append({"start": ci, "end": cj, "kind": "via_rest", "chains": [prim_to_rest[i], prim_from_rest[j]]})
        fixed += 1
        print(f"-- {ci} -> {cj}: fixed (via rest, demo models)", flush=True)
    else:
        print(f"-- {ci} -> {cj}: still no route", flush=True)

json.dump(data, open(path_json, "w"))
kinds = {}
for r in data["routes"]:
    kinds[r["kind"]] = kinds.get(r["kind"], 0) + 1
print(f"\nrepaired {fixed}/{len(missing)}; routes now {len(data['routes'])}/{len(cells)*(len(cells)-1)} {kinds}")
