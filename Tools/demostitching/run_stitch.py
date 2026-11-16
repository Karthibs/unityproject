"""Driver: run DemoStitching's main_stitch on the NERO tabletop demos.

Usage: python run_stitch.py <ds_method> <out_dir> [save_fig 0/1] [scale] [dataset]
"""
import os, sys, collections

REPO = os.path.expanduser("~/DemoStitching")
os.environ.setdefault("MPLBACKEND", "Agg")
os.chdir(REPO)
sys.path.insert(0, REPO)

from configs import StitchConfig  # noqa: E402
import main_stitch  # noqa: E402

method = sys.argv[1] if len(sys.argv) > 1 else "chain"
out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/stitch_out"
save_fig = bool(int(sys.argv[3])) if len(sys.argv) > 3 else True
scale = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
ds_path = sys.argv[5] if len(sys.argv) > 5 else "dataset/stitching/nero_tabletop_xzy"

cfg = StitchConfig(
    dataset_path=ds_path,
    data_position_scale=scale,
    ds_method=method,
    save_fig=save_fig,
    combination_timeout_s=180,
    save_folder_override=out.rstrip("/") + "/",
)
rows = main_stitch.main(cfg)
print("STATUS", collections.Counter(r.get("combination_status") for r in rows[1:]))
for r in rows[1:]:
    print("combo", r.get("combination_id"), r.get("combination_status"), r.get("combination_failure_reason") or "",
          "init=(%.3f,%.3f)" % (r.get("initial_x"), r.get("initial_y")), "att=(%.3f,%.3f)" % (r.get("attractor_x"), r.get("attractor_y")),
          "dist_to_att=%.4f" % r.get("distance_to_attractor_mean", float("nan")), "steps=%s" % r.get("trajectory_length_mean"),
          "rmse=%.3f" % r.get("prediction_rmse", float("nan")), "cos=%.3f" % r.get("cosine_dissimilarity", float("nan")), flush=True)
