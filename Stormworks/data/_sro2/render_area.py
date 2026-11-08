"""python3 render_area.py catenary.json cx,cz radius eye tgt out [fov]: overhead line incl. DPR, rails as strips."""
import json, math, sys
sys.path.insert(0, ".")
import rails, catenary, zrender
plan = json.load(open(sys.argv[1])); cx, cz = map(float, sys.argv[2].split(",")); radius = float(sys.argv[3])
eye = tuple(map(float, sys.argv[4].split(","))); tgt = tuple(map(float, sys.argv[5].split(","))); out = sys.argv[6]
fov = float(sys.argv[7]) if len(sys.argv) > 7 else 50
S, P = plan["supports"], plan["portals"]
near = lambda p, e=0: math.hypot(p[0] - cx, p[2] - cz) <= radius + e
b = rails.Builder()
for portal in P:
    if near(portal["centre"], 20) and not portal.get("unused"): catenary.add_portal(b, portal, (0.0, 0.0))
for sp in S:
    if sp.get("dropped") or not near(sp["track"], 10): continue
    if sp["portal"] is None: catenary.add_support(b, sp, (0.0, 0.0), S)
    else: catenary.add_portal_hanger(b, sp, P[sp["portal"]], (0.0, 0.0))
for span in plan["spans"]:
    a, c = S[span["from"]], S[span["to"]]
    if near(a["track"], 80) and near(c["track"], 80):
        catenary.add_span(b, a, c, (0.0, 0.0), span)
        for off in (-0.8, 0.8):
            ta, tc = a["track"], c["track"]
            d = (tc[0] - ta[0], tc[2] - ta[2]); L = math.hypot(*d) or 1
            r = (d[1] / L * off, -d[0] / L * off)
            pts = [(ta[0] + r[0], ta[1], ta[2] + r[1]), (tc[0] + r[0], tc[1], tc[2] + r[1]),
                   (tc[0] + r[0] * 1.1, tc[1], tc[2] + r[1] * 1.1), (ta[0] + r[0] * 1.1, ta[1], ta[2] + r[1] * 1.1)]
            b.polygon(pts, (90, 90, 95, 255), (0, 0), (0, 1, 0))
FP = plan.get("feeder_poles", [])
for fp in FP:
    if near((fp["pos"][0], 0, fp["pos"][1]), 10): catenary.add_feeder_pole(b, fp, (0.0, 0.0))
for span in plan.get("feeder_spans", []):
    fa, fb = FP[span["from"]], FP[span["to"]]
    if near((fa["pos"][0], 0, fa["pos"][1]), 80): catenary.add_feeder_span(b, fa, fb, (0.0, 0.0))
quads = []
for pts, n, col, _ in b.polys:
    for i in range(1, len(pts) - 1):
        quads.append((pts[0], pts[i], pts[i + 1], pts[i + 1], n, col))
zrender.render(quads, eye, tgt, out, fov=fov, title=out); print(out, len(quads))
