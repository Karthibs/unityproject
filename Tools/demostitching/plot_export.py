"""Top-view plot of exported chained-DS rollouts vs. recorded demos (Unity x/z, metres)."""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

exp = json.load(open(sys.argv[1]))
demo_dir = sys.argv[2]
out = sys.argv[3]
s = exp["scale"]

n = len(exp["tasks"]); cols = 4; rows_ = (n + cols - 1) // cols
fig, axes = plt.subplots(rows_, cols, figsize=(20, 5.2 * rows_), sharex=True, sharey=True)
axes = np.atleast_1d(axes).ravel()
demo_cols = ["tab:red", "tab:blue", "tab:green", "tab:purple"]
for ax, task in zip(axes, exp["tasks"]):
    for k, d in enumerate(sorted(x for x in os.listdir(demo_dir) if x.startswith("demonstration_"))):
        for f in sorted(os.listdir(os.path.join(demo_dir, d))):
            x = np.array(json.load(open(os.path.join(demo_dir, d, f)))["x"])
            ax.plot(x[:, 0], x[:, 2], color=demo_cols[k], alpha=0.25, lw=1)
    r = np.array(task["reference_rollout"]) / s     # ds frame (x, z, y)/scale
    ax.plot(r[:, 0], r[:, 1], "k-", lw=2.2, label="stitched DS rollout")
    ax.plot(*np.array(task["initial"])[:2] / s, "go", ms=9, label="start")
    ax.plot(*np.array(task["attractor"])[:2] / s, "m*", ms=14, label="goal")
    bx, bz = float(os.environ.get("NERO_BASE_X", "0.0")), float(os.environ.get("NERO_BASE_Z", "-0.38"))
    ax.plot(bx, bz, "ks", ms=9)
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(bx + 0.25 * np.cos(th), bz + 0.25 * np.sin(th), "k:", lw=0.6)
    ax.set_title(f"{task['kind'].upper()}: {task['name']}\npath demos {task['path_demos']}, "
                 f"final err {task['reference_final_error_m']*1000:.1f} mm", fontsize=9)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("x (m)")
axes[0].set_ylabel("z (m)")
axes[0].legend(fontsize=8, loc="lower left")
plt.suptitle("DemoStitching chained DS (black) over recorded demos (colour = demo); base = black square, dotted = 0.25 m")
plt.tight_layout()
plt.savefig(out, dpi=100)
print("plot ->", out)
