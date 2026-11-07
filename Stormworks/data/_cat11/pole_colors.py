"""python3 pole_colors.py rom plan.json out.json [budget] : terrain colour under every pole / DPR pole / portal post (resumable)."""
import sys, os, math, json, time
sys.path.insert(0, ".")
from road_map_lib import col
rom, plan_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
budget = float(sys.argv[4]) if len(sys.argv) > 4 else 35
plan = json.load(open(plan_path))
done = json.load(open(out)) if os.path.exists(out) else {}
pts = []
for i, sp in enumerate(plan["supports"]):
    if sp.get("pole") and not sp.get("dropped") and sp.get("pole_owner") is None:
        pts.append(("s%d" % i, sp["pole"][0], sp["pole"][2]))
for i, po in enumerate(plan["portals"]):
    if po.get("unused") or po.get("kind") == "truss": continue
    for j, q in enumerate(po["poles"]): pts.append(("p%d_%d" % (i, j), q[0], q[2]))
for i, f in enumerate(plan.get("feeder_poles", [])):
    if f["kind"] == "pole": pts.append(("f%d" % i, f["pos"][0], f["pos"][1]))
pts.sort(key=lambda p: (math.floor((p[1] + 8500) / 1000), math.floor((p[2] + 12500) / 1000)))
t0 = time.time()
for key, x, z in pts:
    if key in done: continue
    if time.time() - t0 > budget: break
    cs = set()
    for dx, dz in ((0, 0), (0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4)):
        c = col(rom, x + dx, z + dz)
        if c: cs.add(tuple(c))
    done[key] = [x, z, sorted(cs)]
json.dump(done, open(out, "w"))
print("done", len(done), "of", len(pts))
