"""Render the planned catenary (plus terrain) around a point.
python3 render_plan.py <rom> <catenary.json> cx,cz radius eye_x,eye_y,eye_z tgt_x,tgt_y,tgt_z out.png [fov]"""
import json, math, sys
sys.path.insert(0, ".")
import rails, catenary, zrender, render_world
rom, plan_path = sys.argv[1], sys.argv[2]
cx, cz = map(float, sys.argv[3].split(",")); radius = float(sys.argv[4])
eye = tuple(map(float, sys.argv[5].split(","))); tgt = tuple(map(float, sys.argv[6].split(",")))
out = sys.argv[7]; fov = float(sys.argv[8]) if len(sys.argv) > 8 else 55
plan = json.load(open(plan_path))
S = plan["supports"]; portals = plan["portals"]
def near(p, extra=0.0):
    return math.hypot(p[0] - cx, p[2] - cz) <= radius + extra
b = rails.Builder()
for portal in portals:
    if near(portal["centre"], 20):
        catenary.add_portal(b, portal, (0.0, 0.0))
for sp in S:
    if sp.get("dropped") or not near(sp["track"], 10):
        continue
    if sp["portal"] is None:
        catenary.add_support(b, sp, (0.0, 0.0), S)
    else:
        catenary.add_portal_hanger(b, sp, portals[sp["portal"]], (0.0, 0.0))
for span in plan["spans"]:
    a, c = S[span["from"]], S[span["to"]]
    if near(a["track"], 80) and near(c["track"], 80):
        catenary.add_span(b, a, c, (0.0, 0.0), span)
quads = []
for pts, n, col, _ in b.polys:
    for i in range(1, len(pts) - 1):
        quads.append((pts[0], pts[i], pts[i + 1], pts[i + 1], n, col))
T = render_world.tris_world("nowork", rom, cx, cz, radius + 60, terrain=True)
zrender.render(T + quads, eye, tgt, out, fov=fov, title=out)
print(out, len(quads), len(T))
