"""python3 dpr_near.py <catenary.json> x0 x1 z0 z1: per DPR span, closest approach (plan) of its wires to any
electrified track axis, and the wire height above that track's rail there."""
import sys, json, math
sys.path.insert(0, ".")
import catenary, signs
plan = json.load(open(sys.argv[1]))
x0, x1, z0, z1 = map(float, sys.argv[2:6])
sup = plan["supports"]
idx = signs.SegmentIndex()
for s in plan["spans"]:
    a, b = sup[s["from"]], sup[s["to"]]
    ta, tb = a["track"], b["track"]
    idx.add((ta[0], ta[2]), (tb[0], tb[2]), (ta[1], tb[1]))
fps = plan["feeder_poles"]
for sp in plan["feeder_spans"]:
    fa, fb = fps[sp["from"]], fps[sp["to"]]
    if not (x0 <= fa["pos"][0] <= x1 and z0 <= fa["pos"][1] <= z1):
        continue
    pa, pb = catenary.feeder_points(fa), catenary.feeder_points(fb)
    if math.dist(pa[0], pb[0]) + math.dist(pa[1], pb[1]) > math.dist(pa[0], pb[1]) + math.dist(pa[1], pb[0]):
        pb = pb[::-1]
    length = math.dist(fa["pos"], fb["pos"])
    sag = catenary.FEEDER_SAG_70M * (length / 70.0) ** 2
    best = None
    for a, b in zip(pa, pb):
        for k in range(21):
            f = k / 20
            w = catenary._lerp(a, b, f)
            y = w[1] - 4 * sag * f * (1 - f)
            for sa, sb, (ya, yb) in idx.near(w[0], w[2], 8.0):
                dx, dz = sb[0] - sa[0], sb[1] - sa[1]
                L2 = dx * dx + dz * dz or 1
                t = max(0, min(1, ((w[0] - sa[0]) * dx + (w[2] - sa[1]) * dz) / L2))
                d = math.hypot(sa[0] + dx * t - w[0], sa[1] + dz * t - w[2])
                rail = ya + (yb - ya) * t
                if best is None or d < best[0]:
                    best = (d, y - rail, w[0], w[2], f)
    print("span %4d-%4d len %4.0f sag %.2f  closest %.1f m, wire %.2f above rail at (%.0f,%.0f) f=%.2f" % (
        sp["from"], sp["to"], length, sag, *(best or (99, 0, 0, 0, 0))))
