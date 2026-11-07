import json, sys, os, math
sys.path.insert(0, '.')
import build_world, catenary
rom = os.path.expandvars('$HOME/mnt/Stormworks/rom')
scan = json.load(open('work/scan.json'))
plan = catenary.plan_catenary(rom, scan['tiles'], lambda t: build_world.prepare_graph(rom, t, scan))
json.dump(plan, open('work/catenary.json', 'w'))
groups = []
for sp in plan["supports"]:
    if not sp.get("dropped") or sp.get("overlap_of") is not None:
        continue
    p = sp["track"]
    for g in groups:
        if math.hypot(g["last"][0] - p[0], g["last"][2] - p[2]) < 120:
            g["n"] += 1; g["last"] = p; break
    else:
        groups.append({"first": p, "last": p, "n": 1})
print("trimmed groups:")
for g in groups:
    print("  %3d supports  (%.0f, %.0f) .. (%.0f, %.0f)" % (g["n"], g["first"][0], g["first"][2], g["last"][0], g["last"][2]))
