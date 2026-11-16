"""Flatten chained_ds_nero.json into a JsonUtility-friendly file for Unity (NeroChainedDS.cs).

Nested lists become flat float arrays (row-major); non-finite numbers become -1
(the C# side treats a negative edge ratio as "never trigger").
Usage: python to_unity_json.py <in_json> <out_json>
"""
import json, math, sys

src = json.load(open(sys.argv[1]))


def flat(a):
    out = []
    def rec(x):
        if isinstance(x, (list, tuple)):
            for y in x:
                rec(y)
        else:
            v = float(x)
            out.append(v if math.isfinite(v) else -1.0)
    rec(a)
    return out


tasks = []
for t in src["tasks"]:
    if not t.get("ok"):
        continue
    tr = t["transition"]
    tasks.append({
        "name": t["name"], "kind": t["kind"],
        "startObject": t["start_object"], "endObject": t["end_object"],
        "initial": flat(t["initial"]), "attractor": flat(t["attractor"]),
        "pathDemos": [int(i) for i in t["path_demos"]],
        "segments": [{
            "xAtt": flat(s["x_att"]), "K": len(s["priors"]),
            "priors": flat(s["priors"]), "mus": flat(s["mus"]),
            "sigmas": flat(s["sigmas"]), "A": flat(s["A"]),
        } for s in t["segments"]],
        "ratioStartNodes": flat(tr["ratio_start_nodes"]),
        "ratioNodes": flat(tr["ratio_nodes"]),
        "edgeRatios": flat(tr["edge_ratios"]),
        "times": flat(tr["times"]),
        "referenceRollout": flat(t["reference_rollout"]),
        "referenceFinalErrorM": float(t["reference_final_error_m"]),
    })

assert src["trigger"] == "distance_ratio", "only the distance_ratio trigger is ported"
out = {
    "format": "nero_chained_ds_unity_v1",
    "scale": float(src["scale"]),
    "axisPerm": [int(i) for i in src["axis_perm"]],
    "dtReference": float(src["dt_reference"]),
    "velocityMax": float(src["velocity_max"]) if src.get("velocity_max") else 0.0,
    "tasks": tasks,
}
json.dump(out, open(sys.argv[2], "w"))
print(f"wrote {len(tasks)} tasks -> {sys.argv[2]}")
