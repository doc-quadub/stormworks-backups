"""python3 road_eval.py plan.json road_dir : poles on asphalt, with how many of the along-track samples are asphalt."""
import sys, json, math, collections
sys.path.insert(0, ".")
import catenary as C, signs
plan = json.load(open(sys.argv[1])); roads = C.RoadMask(sys.argv[2])
S = plan["supports"]; hist = collections.Counter(); rows = []
for i, sp in enumerate(S):
    if sp.get("dropped") or not sp.get("pole") or sp.get("pole_owner") is not None: continue
    x, z = sp["pole"][0], sp["pole"][2]
    if not any(roads.asphalt(x + dx, z + dz) for dx, dz in ((0, 0), (0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4))): continue
    t = sp["t"]
    n = sum(roads.asphalt(x + t[0] * d, z + t[1] * d) for d in C.ROAD_SAMPLES)
    hist[n] += 1; rows.append((n, i, round(x), round(z)))
print(hist)
for r in sorted(rows)[:40]: print(r)
