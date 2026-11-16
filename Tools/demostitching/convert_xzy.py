"""Reorder recorded Unity demos (x, y_up, z) -> (x, z, y_up) so dims 0,1 are the table plane.

Usage: python convert_xzy.py <src_dir> <dst_dir>
"""
import json, os, sys, shutil

src, dst = sys.argv[1], sys.argv[2]
if os.path.isdir(dst):
    shutil.rmtree(dst)
os.makedirs(dst)
perm = [0, 2, 1]
n = 0
for d in sorted(os.listdir(src)):
    p = os.path.join(src, d)
    if not (os.path.isdir(p) and d.startswith("demonstration_")):
        continue
    os.makedirs(os.path.join(dst, d))
    for f in sorted(os.listdir(p)):
        t = json.load(open(os.path.join(p, f)))
        # Trim the stationary tail (arm resting at the goal): identical end points
        # otherwise form a zero-size Gaussian. Keep up to the last "moving" sample + 1.
        sp = [sum(c * c for c in v) ** 0.5 for v in t["x_dot"]]
        thr = 0.03 * max(sp)
        last = max(i for i, s in enumerate(sp) if s >= thr)
        keep = min(len(sp), last + 2)
        t["x"], t["x_dot"] = t["x"][:keep], t["x_dot"][:keep]
        t["x_dot"][-1] = [0.0, 0.0, 0.0]
        t["x"] = [[v[i] for i in perm] for v in t["x"]]
        t["x_dot"] = [[v[i] for i in perm] for v in t["x_dot"]]
        t["frame"] = "table_xzy: (unity_x, unity_z, unity_y_up), metres"
        json.dump(t, open(os.path.join(dst, d, f), "w"))
        n += 1
plan = json.load(open(os.path.join(src, "demo_plan.json")))
plan["frame"] = "table_xzy: (unity_x, unity_z, unity_y_up), metres"
for dm in plan["demonstrations"]:
    for k in ("start_anchor", "end_anchor", "hub"):
        dm[k] = [dm[k][i] for i in perm]
json.dump(plan, open(os.path.join(dst, "demo_plan.json"), "w"), indent=1)
print(f"converted {n} trajectories -> {dst}")
