"""Figures for the NERO grid DemoStitching results.

  1  fig1_recorded_demos.png      recorded demonstrations (top view, height profile, tracking error)
  2  fig2_stitched_routes.png     stitched routes as planned by DemoStitching (Python rollouts)
  3  fig3_route_types.png         which of the 306 routes are direct stitched chains vs via rest
  4  fig4_tested_vs_planned.png   routes executed on the simulated robot vs the plan
  5  fig5_test_metrics.png        per-route deviation from plan, duration and end error

Usage: python plot_grid_results.py <project_dir> <out_dir>
Frame: Unity world, top view = (x, z); robot base at (0, -0.38).
"""
import csv, glob, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon

proj, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)

# ---- style: reference palette (validated slots 1-3 for all-pairs forms), recessive axes
SURFACE, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#a3a29d", "#e4e3df"
RING = {"A": "#2a78d6", "B": "#eb6834", "C": "#1baf7a"}          # categorical slots 1-3
DIRECT, VIAREST = "#2a78d6", "#eb6834"                            # slot 1 / slot 2
CONTEXT = "#cfcec9"
plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.titlecolor": INK, "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.labelsize": 9, "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 8,
    "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "font.family": "DejaVu Sans",
})

BASE = np.array([0.0, -0.38])
RADII, ANGLES, HALF_R, HALF_A = [0.30, 0.37, 0.44], [-75, -45, -15, 15, 45, 75], 0.035, 15
CELLS = [f"{r}{c + 1}" for r in "ABC" for c in range(6)]
REST = np.array([0.0, -0.02])


def polar(r, deg):
    t = np.radians(deg)
    return np.array([BASE[0] + r * np.sin(t), BASE[1] + r * np.cos(t)])


def cell_center(name):
    return polar(RADII["ABC".index(name[0])], ANGLES[int(name[1]) - 1])


def draw_workspace(ax, labels=True, highlight=()):
    for i, name in enumerate(CELLS):
        r, a = RADII[i // 6], ANGLES[i % 6]
        a0, a1 = a - HALF_A, a + HALF_A
        pts = [polar(r - HALF_R, d) for d in np.linspace(a0, a1, 12)] + [polar(r + HALF_R, d) for d in np.linspace(a1, a0, 12)]
        hl = name in highlight
        ax.add_patch(Polygon(pts, closed=True, fill=False, lw=1.4 if hl else 0.6, ec=INK2 if hl else GRID, zorder=1))
        if labels:
            c = cell_center(name)
            ax.text(c[0], c[1], name, ha="center", va="center", fontsize=6.5, color=MUTED, zorder=1)
    ax.plot(*BASE, marker="s", ms=9, color=INK, zorder=5)
    ax.annotate("robot base", BASE, xytext=(0, -12), textcoords="offset points", ha="center", fontsize=7, color=INK2)
    ax.plot(*REST, marker="*", ms=11, color=INK, mec=SURFACE, mew=1.0, zorder=6)
    ax.set_aspect("equal")
    ax.set_xlim(-0.52, 0.52); ax.set_ylim(-0.46, 0.13)
    ax.set_xlabel("x (m)"); ax.set_ylabel("z (m)")


# ---------------------------------------------------------------- data
demo_root = os.path.join(proj, "DemoData", "nero_grid")
demos = {}   # cell -> list of (x[n,3], t[n], track_err)
for d in sorted(glob.glob(os.path.join(demo_root, "demonstration_*"))):
    for f in sorted(glob.glob(os.path.join(d, "trajectory_*.json"))):
        t = json.load(open(f))
        cell = t["source_task"].split("_")[0]
        x = np.array(t["x"])
        tt = np.linspace(0, t["duration_s"], len(x))
        demos.setdefault(cell, []).append((x, tt, t["max_tracking_error_m"]))

exp = json.load(open(os.path.join(proj, "DemoData", "chained_ds_grid_unity.json")))
scale = exp["scale"]
tasks, routes = exp["tasks"], exp["routes"]
route_of = {(r["start"], r["end"]): r for r in routes}


def rollout_xz(task):
    r = np.array(task["referenceRollout"]).reshape(-1, 3) / scale     # ds frame (x, z, y)
    return r[:, [0, 1]], r[:, 2]


def planned_path(route):
    xz = [rollout_xz(tasks[c])[0] for c in route["chains"]]
    return np.vstack(xz)


def demo_ids_for_route(route):
    ids = set()
    for c in route["chains"]:
        ids.update(tasks[c].get("pathDemos", []))
    return ids


runs = {}
for f in sorted(glob.glob(os.path.join(proj, "Verification", "grid_runs", "*.csv"))):
    with open(f) as fh:
        meta = dict(kv.split("=", 1) for kv in fh.readline()[2:].split())
        rows = list(csv.DictReader(fh))
    p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    runs[(meta["start"], meta["end"])] = {"meta": meta, "p": p}


def deviation(executed_xz, planned_xz):
    # distance from each executed point to the densified planned polyline
    dense = []
    for a, b in zip(planned_xz[:-1], planned_xz[1:]):
        n = max(2, int(np.linalg.norm(b - a) / 0.002))
        dense.append(np.linspace(a, b, n, endpoint=False))
    dense = np.vstack(dense + [planned_xz[-1:]])
    d = np.sqrt(((executed_xz[:, None, :] - dense[None, :, :]) ** 2).sum(-1)).min(1)
    return d.mean(), d.max()


# ---------------------------------------------------------------- figure 1: recorded demos
fig = plt.figure(figsize=(17, 5.6))
gs = fig.add_gridspec(1, 3, width_ratios=[1.7, 1, 1], wspace=0.25)
ax = fig.add_subplot(gs[0])
draw_workspace(ax)
for cell, trajs in demos.items():
    for x, _, _ in trajs:
        ax.plot(x[:, 0], x[:, 2], color=RING[cell[0]], lw=1.0, alpha=0.85, zorder=3)
        ax.plot(x[0, 0], x[0, 2], "o", color=RING[cell[0]], ms=3.5, zorder=4)
ax.set_title("Recorded demonstrations (18 cells → rest, 72 trajectories)")
ax.legend(handles=[Line2D([], [], color=RING[k], lw=2, label=f"ring {k} (r = {RADII['ABC'.index(k)]:.2f} m)") for k in "ABC"]
          + [Line2D([], [], marker="*", color=INK, ls="", ms=10, label="rest position")], loc="upper left")

ax = fig.add_subplot(gs[1])
for cell, trajs in demos.items():
    for x, tt, _ in trajs:
        ax.plot(tt / tt[-1], x[:, 1], color=RING[cell[0]], lw=0.9, alpha=0.7)
ax.set_title("Height profile")
ax.set_xlabel("normalised time"); ax.set_ylabel("tool height y (m)")
ax.legend(handles=[Line2D([], [], color=RING[k], lw=2, label=f"ring {k}") for k in "ABC"], loc="upper right")

ax = fig.add_subplot(gs[2])
for k, cell in enumerate(CELLS):
    errs = [e * 1000 for _, _, e in demos.get(cell, [])]
    ax.plot([k] * len(errs), errs, "o", color=RING[cell[0]], ms=5, alpha=0.85)
ax.set_xticks(range(18)); ax.set_xticklabels(CELLS, rotation=90)
ax.set_ylim(0, max(4.5, ax.get_ylim()[1]))
ax.set_title("Tracking error while recording")
ax.set_ylabel("max |executed − reference| (mm)")
fig.savefig(os.path.join(out, "fig1_recorded_demos.png"), dpi=130, bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------- figure 2: stitched routes (planned)
def longest(kind, n):
    cand = [r for r in routes if r["kind"] == kind]
    cand.sort(key=lambda r: -np.linalg.norm(cell_center(r["start"]) - cell_center(r["end"])))
    picked, used = [], set()
    for r in cand:
        if r["start"] in used:
            continue
        picked.append(r); used.add(r["start"])
        if len(picked) == n:
            break
    return picked

gallery = longest("direct", 3) + longest("via_rest", 3)
fig, axes = plt.subplots(2, 3, figsize=(15, 7.6))
for ax, r in zip(axes.ravel(), gallery):
    draw_workspace(ax, labels=False, highlight=(r["start"], r["end"]))
    si, ei = CELLS.index(r["start"]), CELLS.index(r["end"])
    path_ids = demo_ids_for_route(r) if r["kind"] == "direct" else {si, ei}
    for k, cell in enumerate(CELLS):
        for x, _, _ in demos.get(cell, []):
            if k == si:
                col, lw, z = DIRECT, 1.2, 3
            elif k == ei:
                col, lw, z = VIAREST, 1.2, 3
            elif k in path_ids:
                col, lw, z = MUTED, 0.9, 2
            else:
                col, lw, z = CONTEXT, 0.6, 2
            ax.plot(x[:, 0], x[:, 2], color=col, lw=lw, alpha=0.9, zorder=z)
    p = planned_path(r)
    ax.plot(p[:, 0], p[:, 1], color=INK, lw=2.2, zorder=5)
    s, e = cell_center(r["start"]), cell_center(r["end"])
    ax.plot(*s, "o", ms=8, color=INK, mec=SURFACE, zorder=6)
    ax.plot(*e, "X", ms=9, color=INK, mec=SURFACE, zorder=6)
    nseg = sum(len(tasks[c]["segments"]) for c in r["chains"])
    label = f"direct stitched chain · {nseg} segments" if r["kind"] == "direct" else "via rest · demo model + reversed demo model"
    ax.set_title(f"{r['start']} → {r['end']}\n{label}", fontsize=9.5)
    ax.set_ylim(-0.44, 0.12)
fig.legend(handles=[
    Line2D([], [], color=INK, lw=2.2, label="stitched path (DemoStitching rollout)"),
    Line2D([], [], color=DIRECT, lw=1.5, label="demo of the start cell"),
    Line2D([], [], color=VIAREST, lw=1.5, label="demo of the end cell (used reversed)"),
    Line2D([], [], color=MUTED, lw=1.2, label="other demos on the graph path"),
    Line2D([], [], color=CONTEXT, lw=1.2, label="unused demos"),
    Line2D([], [], marker="o", color=INK, ls="", label="start"), Line2D([], [], marker="X", color=INK, ls="", label="end"),
], loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.02))
fig.suptitle("Stitched routes planned by DemoStitching (Python rollouts): top = direct, bottom = via rest", fontsize=13, fontweight="bold")
fig.tight_layout(rect=(0, 0.07, 1, 0.98))
fig.savefig(os.path.join(out, "fig2_stitched_routes.png"), dpi=130, bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------- figure 3: route types (306 routes)
M = np.full((18, 18), np.nan)
for r in routes:
    M[CELLS.index(r["start"]), CELLS.index(r["end"])] = 1.0 if r["kind"] == "direct" else 0.0
fig, ax = plt.subplots(figsize=(7.6, 7))
from matplotlib.colors import ListedColormap
ax.imshow(M, cmap=ListedColormap([VIAREST, DIRECT]), vmin=0, vmax=1)
# 2 px surface gap between cells
for k in range(19):
    ax.axhline(k - 0.5, color=SURFACE, lw=2); ax.axvline(k - 0.5, color=SURFACE, lw=2)
ax.set_xticks(range(18)); ax.set_xticklabels(CELLS, rotation=90)
ax.set_yticks(range(18)); ax.set_yticklabels(CELLS)
ax.set_xlabel("end cell"); ax.set_ylabel("start cell"); ax.grid(False)
nd, nv = int(np.nansum(M == 1)), int(np.nansum(M == 0))
ax.set_title(f"Route type for all 306 start → end pairs")
ax.legend(handles=[Patch(color=DIRECT, label=f"direct stitched chain ({nd})"),
                   Patch(color=VIAREST, label=f"via rest ({nv})")], loc="upper center",
          bbox_to_anchor=(0.5, -0.1), ncol=2)
fig.savefig(os.path.join(out, "fig3_route_types.png"), dpi=130, bbox_inches="tight")
plt.close(fig)

# ---------------------------------------------------------------- figures 4 + 5: tested on the robot
if runs:
    stats = []
    for (s, e), run in runs.items():
        r = route_of.get((s, e))
        if r is None or len(run["p"]) < 2:
            continue
        exe = run["p"][:, [0, 2]]
        mean_d, max_d = deviation(exe, planned_path(r))
        m = run["meta"]
        stats.append(dict(s=s, e=e, kind=r["kind"], mean=mean_d * 1000, max=max_d * 1000,
                          dur=float(m["duration_s"]), err=float(m["end_error_m"]) * 1000,
                          ratio=float(np.sum(np.linalg.norm(np.diff(exe, axis=0), axis=1)) /
                                      max(1e-6, np.sum(np.linalg.norm(np.diff(planned_path(r), axis=0), axis=1)))),
                          arrived=m["arrived"] == "True", fallback=m["fallback"] == "True"))

    # fig 4: overview of all executed paths + a gallery of executed vs planned
    show = sorted(stats, key=lambda x: -x["max"])[:3] + sorted(stats, key=lambda x: x["mean"])[:3]
    show = [show[i] for i in (3, 4, 5, 0, 1, 2)]   # best first, then worst
    fig = plt.figure(figsize=(18, 7.4))
    gs = fig.add_gridspec(2, 5, width_ratios=[1, 1, 0.08, 1, 1], hspace=0.42, wspace=0.12)
    ax = fig.add_subplot(gs[:, 0:2])
    draw_workspace(ax)
    for st in stats:
        p = runs[(st["s"], st["e"])]["p"]
        ax.plot(p[:, 0], p[:, 2], color=DIRECT if st["kind"] == "direct" else VIAREST, lw=1.1, alpha=0.8, zorder=3)
    ok = sum(st["arrived"] for st in stats)
    ax.set_title(f"All {len(stats)} tested routes on the robot\n(arrived {ok}/{len(stats)}, safety fallback {sum(st['fallback'] for st in stats)})")
    ax.legend(handles=[Line2D([], [], color=DIRECT, lw=2, label=f"direct ({sum(st['kind']=='direct' for st in stats)})"),
                       Line2D([], [], color=VIAREST, lw=2, label=f"via rest ({sum(st['kind']=='via_rest' for st in stats)})")],
              loc="upper left")
    show = show[:2] + show[3:5]          # 2 closest + 2 largest deviation
    for k, st in enumerate(show):
        ax = fig.add_subplot(gs[k // 2, 3 + k % 2])
        r = route_of[(st["s"], st["e"])]
        draw_workspace(ax, labels=False, highlight=(st["s"], st["e"]))
        pl = planned_path(r)
        ex = runs[(st["s"], st["e"])]["p"]
        ax.plot(pl[:, 0], pl[:, 1], color=INK, lw=1.4, ls=(0, (4, 2)), zorder=4)
        ax.plot(ex[:, 0], ex[:, 2], color=DIRECT if st["kind"] == "direct" else VIAREST, lw=2, zorder=5)
        ax.plot(*cell_center(st["s"]), "o", ms=7, color=INK, mec=SURFACE, zorder=6)
        ax.plot(*cell_center(st["e"]), "X", ms=8, color=INK, mec=SURFACE, zorder=6)
        tag = "closest to plan" if k < 2 else "largest deviation"
        fb = " · safety fallback" if st["fallback"] else ""
        ax.set_title(f"{st['s']} → {st['e']} ({st['kind'].replace('_', ' ')}) · {tag}{fb}\nmean {st['mean']:.1f} mm, max {st['max']:.1f} mm", fontsize=8.5)
        ax.set_ylim(-0.44, 0.12)
        ax.set_xlabel(""); ax.set_ylabel("")
    fig.legend(handles=[Line2D([], [], color=INK, lw=1.4, ls=(0, (4, 2)), label="planned (DemoStitching rollout)"),
                        Line2D([], [], color=DIRECT, lw=2, label="executed, direct route"),
                        Line2D([], [], color=VIAREST, lw=2, label="executed, via rest")],
               loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.04))
    fig.suptitle("Tested on the simulated NERO: executed tool paths vs DemoStitching plan", fontsize=13, fontweight="bold")
    fig.savefig(os.path.join(out, "fig4_tested_vs_planned.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    # fig 5: metrics, one measure per panel, routes sorted by mean deviation
    order = sorted(stats, key=lambda x: x["mean"])
    labels = [f"{st['s']}→{st['e']}" + (" (fallback)" if st["fallback"] else "") for st in order]
    cols = [DIRECT if st["kind"] == "direct" else VIAREST for st in order]
    y = np.arange(len(order))
    fig, axes = plt.subplots(1, 3, figsize=(15, 0.26 * len(order) + 1.8), sharey=True)
    ax = axes[0]
    ax.hlines(y, [st["mean"] for st in order], [st["max"] for st in order], color=GRID, lw=2)
    ax.plot([st["mean"] for st in order], y, "o", color=INK, ms=4, label="mean")
    ax.scatter([st["max"] for st in order], y, c=cols, s=30, zorder=3, edgecolors=SURFACE, linewidths=1.5, label="max")
    ax.set_xlabel("deviation from planned path (mm)"); ax.set_title("Path deviation")
    ax.legend(handles=[Line2D([], [], marker="o", color=INK, ls="", ms=4, label="mean"),
                       Line2D([], [], marker="o", color=MUTED, ls="", ms=6, label="max (colour = route type)")], loc="lower right")
    ax = axes[1]
    ax.barh(y, [st["dur"] for st in order], color=cols, height=0.7)
    ax.set_xlabel("time from start cell to end cell (s)"); ax.set_title("Duration")
    ax = axes[2]
    ax.axvline(1.0, color=INK2, lw=1)
    ax.plot([st["ratio"] for st in order], y, "o", color=INK2, ms=0)
    ax.scatter([st["ratio"] for st in order], y, c=cols, s=30, edgecolors=SURFACE, linewidths=1.5, zorder=3)
    ax.set_xlabel("executed / planned path length (1 = same length)"); ax.set_title("Path length")
    axes[0].set_yticks(y); axes[0].set_yticklabels(labels, fontsize=7); axes[0].invert_yaxis()
    for a in axes:
        a.grid(axis="y", visible=False)
    fig.legend(handles=[Patch(color=DIRECT, label="direct stitched chain"), Patch(color=VIAREST, label="via rest")],
               loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    errs = [st["err"] for st in stats]
    fig.suptitle(f"Robot test metrics, {len(stats)} routes (end error {min(errs):.1f}-{max(errs):.1f} mm for every route)",
                 fontsize=12, fontweight="bold", y=1.02)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(out, "fig5_test_metrics.png"), dpi=130, bbox_inches="tight")
    plt.close(fig)

    with open(os.path.join(out, "test_metrics.csv"), "w") as fh:
        fh.write("start,end,kind,arrived,fallback,duration_s,mean_dev_mm,max_dev_mm,end_error_mm\n")
        for st in order:
            fh.write(f"{st['s']},{st['e']},{st['kind']},{st['arrived']},{st['fallback']},{st['dur']:.2f},{st['mean']:.1f},{st['max']:.1f},{st['err']:.1f}\n")
    d = np.array([st["mean"] for st in stats]); m = np.array([st["max"] for st in stats])
    print(f"tested {len(stats)} routes: arrived {ok}, mean deviation median {np.median(d):.1f} mm, "
          f"max deviation median {np.median(m):.1f} mm (worst {m.max():.1f}), duration median {np.median([s['dur'] for s in stats]):.1f} s")
else:
    print("no executed runs in Verification/grid_runs yet - figures 4 and 5 skipped")
print("figures ->", out)
