"""Sanity checks + plot for recorded NERO demonstrations (DemoStitching format)."""
import json, os, sys
import numpy as np

root = sys.argv[1]
out_png = sys.argv[2]
demos = sorted(d for d in os.listdir(root) if d.startswith("demonstration_"))
data = {}
for d in demos:
    trajs = []
    for f in sorted(os.listdir(os.path.join(root, d))):
        t = json.load(open(os.path.join(root, d, f)))
        x, xd = np.array(t["x"]), np.array(t["x_dot"])
        trajs.append((x, xd, t))
    data[d] = trajs

for d, trajs in data.items():
    print(f"== {d}: {len(trajs)} trajectories, task {trajs[0][2]['source_task']}")
    for i, (x, xd, t) in enumerate(trajs):
        L = np.sum(np.linalg.norm(np.diff(x, axis=0), axis=1))
        sp = np.linalg.norm(xd, axis=1)
        dt = t["duration_s"] / (len(x) - 1)
        # consistency of x_dot with finite differences of x
        fd = np.gradient(x, axis=0) / dt
        rel = np.linalg.norm(fd[1:-1] - xd[1:-1]) / np.linalg.norm(xd[1:-1])
        # smoothness: max change of direction between samples (moving part only)
        mv = sp > 0.05 * sp.max()
        u = xd[mv] / sp[mv, None]
        turn = np.degrees(np.arccos(np.clip(np.sum(u[1:] * u[:-1], axis=1), -1, 1))).max()
        print(f"  traj {i}: n={len(x)} len={L:.3f} m  T={t['duration_s']:.2f}s  vmax={sp.max():.3f} m/s  "
              f"v_end={sp[-1]:.4f}  start={np.round(x[0],3)} end={np.round(x[-1],3)}  "
              f"track_err={t['max_tracking_error_m']*1000:.1f}mm  xdot-vs-fd={rel:.3f}  max_turn={turn:.1f}deg  "
              f"y∈[{x[:,1].min():.3f},{x[:,1].max():.3f}]")

# overlap between every pair of demos: how many points lie within 2 cm, and at what angle
keys = list(data)
for a in range(len(keys)):
    for b in range(a + 1, len(keys)):
        A = np.vstack([x for x, _, _ in data[keys[a]]]); Av = np.vstack([xd for _, xd, _ in data[keys[a]]])
        B = np.vstack([x for x, _, _ in data[keys[b]]]); Bv = np.vstack([xd for _, xd, _ in data[keys[b]]])
        D = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
        ia, ib = np.nonzero(D < 0.02)
        msg = f"overlap {keys[a]} vs {keys[b]}: min {D.min()*1000:.1f} mm, {len(set(ia))}/{len(set(ib))} pts within 20 mm"
        if len(ia):
            va, vb = Av[ia], Bv[ib]
            ok = (np.linalg.norm(va, axis=1) > 1e-3) & (np.linalg.norm(vb, axis=1) > 1e-3)
            if ok.any():
                cosang = np.sum(va[ok] * vb[ok], axis=1) / (np.linalg.norm(va[ok], axis=1) * np.linalg.norm(vb[ok], axis=1))
                msg += f", median angle {np.degrees(np.arccos(np.clip(np.median(cosang), -1, 1))):.0f} deg"
        print(msg)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig = plt.figure(figsize=(13, 5.5))
ax1 = fig.add_subplot(1, 2, 1)
ax2 = fig.add_subplot(1, 2, 2, projection="3d")
colors = ["tab:red", "tab:blue", "tab:green", "tab:orange"]
for k, (d, trajs) in enumerate(data.items()):
    for i, (x, xd, t) in enumerate(trajs):
        lab = f"{d} ({t['source_task']})" if i == 0 else None
        ax1.plot(x[:, 0], x[:, 2], color=plt.cm.tab20(k % 20), lw=1, label=lab if len(data) <= 6 else None)
        ax1.plot(x[0, 0], x[0, 2], "o", color=plt.cm.tab20(k % 20), ms=4)
        ax1.plot(x[-1, 0], x[-1, 2], "x", color=plt.cm.tab20(k % 20), ms=6)
        ax2.plot(x[:, 0], x[:, 2], x[:, 1], color=plt.cm.tab20(k % 20), lw=1)
plan = json.load(open(os.path.join(root, "demo_plan.json")))
bx, bz = float(os.environ.get("NERO_BASE_X", "0.0")), float(os.environ.get("NERO_BASE_Z", "-0.38"))
ax1.plot(bx, bz, "ks", ms=10, label="robot base")
th = np.linspace(0, 2 * np.pi, 200)
for r in (0.25, 0.5):
    ax1.plot(bx + r * np.cos(th), bz + r * np.sin(th), "k:", lw=0.7)
ax1.set_xlabel("x (m)"); ax1.set_ylabel("z (m)"); ax1.set_aspect("equal"); ax1.grid(alpha=0.3)
ax1.set_title("Top view (o start, x end; dotted = 0.25/0.5 m reach ring)")
ax1.legend(fontsize=8, loc="lower left")
ax2.set_xlabel("x"); ax2.set_ylabel("z"); ax2.set_zlabel("y (height)"); ax2.set_title("3D")
plt.tight_layout(); plt.savefig(out_png, dpi=110)
print("plot ->", out_png)
