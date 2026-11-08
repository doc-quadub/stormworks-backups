"""python3 dpr_conflict.py <catenary.json> [x0 x1 z0 z1]: DPR wire samples that come close (in plan) to an
electrified track's contact line and are not well above its messenger."""
import sys, json, math
sys.path.insert(0, ".")
import catenary, signs
plan = json.load(open(sys.argv[1]))
box = list(map(float, sys.argv[2:6])) if len(sys.argv) > 5 else None
sup = plan["supports"]
idx = signs.SegmentIndex()
for s in plan["spans"]:
    a, b = sup[s["from"]], sup[s["to"]]
    ta, tb = a["track"], b["track"]
    idx.add((ta[0], ta[2]), (tb[0], tb[2]), (ta[1], tb[1]))
fps = plan["feeder_poles"]
hits = []
for sp in plan["feeder_spans"]:
    fa, fb = fps[sp["from"]], fps[sp["to"]]
    pa, pb = catenary.feeder_points(fa), catenary.feeder_points(fb)
    if math.dist(pa[0], pb[0]) + math.dist(pa[1], pb[1]) > math.dist(pa[0], pb[1]) + math.dist(pa[1], pb[0]):
        pb = pb[::-1]
    length = math.dist(fa["pos"], fb["pos"])
    sag = catenary.FEEDER_SAG_70M * (length / 70.0) ** 2
    worst = None
    for a, b in zip(pa, pb):
        for k in range(0, 21):
            f = k / 20
            w = catenary._lerp(a, b, f)
            y = w[1] - 4 * sag * f * (1 - f)
            for sa, sb, (ya, yb) in idx.near(w[0], w[2], 5.0):
                dx, dz = sb[0] - sa[0], sb[1] - sa[1]
                L2 = dx * dx + dz * dz or 1
                t = max(0, min(1, ((w[0] - sa[0]) * dx + (w[2] - sa[1]) * dz) / L2))
                d = math.hypot(sa[0] + dx * t - w[0], sa[1] + dz * t - w[2])
                if d > 4.0:
                    continue
                rail = ya + (yb - ya) * t
                margin = y - (rail + catenary.CONTACT_WIRE_HEIGHT)
                if worst is None or margin < worst[0]:
                    worst = (margin, d, w[0], w[2], y, rail)
    if worst and worst[0] < 2.5:
        if box and not (box[0] <= worst[2] <= box[1] and box[2] <= worst[3] <= box[3]):
            continue
        hits.append((worst, fa["kind"], fb["kind"], round(fa["bracket_y"] - fa["base_y"], 2), round(fb["bracket_y"] - fb["base_y"], 2), round(length)))
hits.sort()
print(len(hits))
for h in hits[:60]:
    (m, d, x, z, y, rail), ka, kb, ha, hb, L = h
    print("above CW %+.2f  plan dist %.1f  at (%.0f,%.0f) wire %.2f rail %.2f  %s/%s h %.2f/%.2f len %d" % (m, d, x, z, y, rail, ka, kb, ha, hb, L))
