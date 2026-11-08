"""python3 ground_diag.py rom plan.json x0 z0 x1 z1 : physics surface heights at the pole spots in a box."""
import json, sys, math
sys.path.insert(0, ".")
import signs
rom, plan_path = sys.argv[1], sys.argv[2]
x0, z0, x1, z1 = map(float, sys.argv[3:7])
plan = json.load(open(plan_path)); S = plan["supports"]
g = signs.Ground(rom)
for i, sp in enumerate(S):
    if sp.get("dropped") or not sp.get("pole") or sp.get("pole_owner") is not None: continue
    x, _, z = sp["pole"]
    if not (min(x0, x1) <= x <= max(x0, x1) and min(z0, z1) <= z <= max(z0, z1)): continue
    hs = sorted(round(h - sp["track"][1], 1) for h in g.heights(x, z))
    if hs and max(hs) > 1.5:
        print(i, round(x), round(z), "side", sp["side"], "ground-rail", hs)
