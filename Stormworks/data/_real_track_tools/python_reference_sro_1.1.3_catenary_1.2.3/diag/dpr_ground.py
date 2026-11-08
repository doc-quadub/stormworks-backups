"""python3 dpr_ground.py rom plan.json : DPR spans whose wires come near or into the ground."""
import sys, json, math
sys.path.insert(0, ".")
import catenary as C, signs
plan = json.load(open(sys.argv[2])); g = signs.Ground(sys.argv[1])
FP = plan["feeder_poles"]; bad = []
for sp in plan["feeder_spans"]:
    a, b = FP[sp["from"]], FP[sp["to"]]
    length = math.dist(a["pos"], b["pos"]); sag = C.FEEDER_SAG_70M * (length / 70) ** 2
    pa, pb = C.feeder_points(a), C.feeder_points(b)
    if math.dist(pa[0], pb[0]) + math.dist(pa[1], pb[1]) > math.dist(pa[0], pb[1]) + math.dist(pa[1], pb[0]): pb = pb[::-1]
    worst = 99
    for p, q in zip(pa, pb):
        for k in range(1, 20):
            f = k / 20; w = C._lerp(p, q, f); y = w[1] - 4 * sag * f * (1 - f)
            H = g.heights(w[0], w[2])
            below = [h for h in H if h <= y]
            near = [h for h in H if y < h < y + 1.0]
            if near or (H and not below): worst = min(worst, -1.0)
            elif below: worst = min(worst, y - max(below))
    if worst < 1.0: bad.append((round(worst, 1), [round(v) for v in a["pos"]]))
print(len(plan["feeder_spans"]), "spans, near/in ground:", sorted(bad)[:20], len(bad))
