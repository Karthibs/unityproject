"""Probe DemoStitching's Gaussian graph for the NERO demos across scale settings.

For each setting: Gaussians per demo, edges, cross-demo edges, and the shortest
path for all start->end tasks (does it switch demos?).
Usage: python probe_graph.py <dataset_rel_path> <scale> [<scale> ...]
"""
import os, sys, time, itertools
import numpy as np

REPO = os.path.expanduser("~/DemoStitching")
os.environ.setdefault("MPLBACKEND", "Agg")
os.chdir(REPO)
sys.path.insert(0, REPO)
import contextlib, io

from configs import StitchConfig
import src.graph_utils as gu
from src.util.load_tools import get_demonstration_set
from src.util.ds_tools import apply_lpvds_demowise, get_gaussian_directions

ds_path = sys.argv[1]
scales = [float(s) for s in sys.argv[2:]] or [1.0]
names = {0: "red->trayB", 1: "blue->trayA"}

for scale in scales:
    np.random.seed(42)
    cfg = StitchConfig(dataset_path=ds_path, data_position_scale=scale)
    demo_set = get_demonstration_set(ds_path, position_scale=scale, velocity_scale=scale)
    t0 = time.time()
    with contextlib.redirect_stdout(io.StringIO()):
        ds_set, rev, norm = apply_lpvds_demowise(demo_set, cfg.damm)
    gaussians = {
        (i, j): {"mu": mu, "sigma": s, "direction": d, "prior": p}
        for i, ds in enumerate(ds_set)
        for j, (mu, s, d, p) in enumerate(zip(ds.damm.Mu, ds.damm.Sigma, get_gaussian_directions(ds), ds.damm.Prior))
    }
    gg = gu.GaussianGraph(param_dist=cfg.param_dist, param_cos=cfg.param_cos, bhattacharyya_threshold=cfg.bhattacharyya_threshold)
    gg.add_gaussians(gaussians, reverse_gaussians=cfg.reverse_gaussians)
    E = list(gg.graph.edges)
    cross = [e for e in E if e[0][0] != e[1][0]]
    K = [ds.damm.K for ds in ds_set]
    sig = [np.sqrt(np.linalg.eigvalsh(s).max()) / scale for ds in ds_set for s in ds.damm.Sigma]
    print(f"\n=== scale {scale}: K per demo {K}, graph {len(gg.graph.nodes)} nodes / {len(E)} edges, "
          f"cross-demo edges {len(cross)}, Gaussian max std {min(sig)*100:.1f}-{max(sig)*100:.1f} cm (fit {time.time()-t0:.1f}s)")
    starts = [np.mean([t.x[0] for t in d.trajectories], axis=0) for d in demo_set]
    ends = [np.mean([t.x[-1] for t in d.trajectories], axis=0) for d in demo_set]
    for si, ei in itertools.product(range(len(starts)), range(len(ends))):
        path = gg.shortest_path(starts[si], ends[ei])
        label = f"start of demo{si} -> end of demo{ei}" + (" (demonstrated)" if si == ei else " (STITCH)")
        if path is None:
            print(f"  {label}: NO PATH")
            continue
        demos_used = [n[0] for n in path]
        switches = sum(1 for a, b in zip(demos_used, demos_used[1:]) if a != b)
        rev_n = sum(1 for n in path if n in gg.gaussian_reversal_map)
        print(f"  {label}: {len(path)} nodes, demos {demos_used}, switches {switches}, reversed nodes {rev_n}")
