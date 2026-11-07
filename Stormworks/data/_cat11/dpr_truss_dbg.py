"""Why DPR spans next to the truss bridges are refused: for each truss DPR candidate in a plan, try the
nearest pole DPR supports and report the ground test."""
import sys, json, math
sys.path.insert(0, ".")
import catenary as C, signs
rom = sys.argv[1]; plan = json.load(open(sys.argv[2])); g = signs.Ground(rom)
FP = plan["feeder_poles"]
truss = [f for f in FP if f["kind"] == "truss"]
for f in truss:
    for q in FP:
        if q is f or q["kind"] == "truss": continue
        d = math.dist(f["pos"], q["pos"])
        if d > 85 or d < 15: continue
        if f["out"][0] * q["out"][0] + f["out"][1] * q["out"][1] < 0.3: continue
        length = d; sag = C.FEEDER_SAG_70M * (length / 70) ** 2
        pa, pb = C.feeder_points(f), C.feeder_points(q)
        bad = []
        for a, b in zip(pa, pb):
            for k in range(1, 20):
                fr = k / 20; w = C._lerp(a, b, fr); y = w[1] - 4 * sag * fr * (1 - fr)
                H = g.heights(w[0], w[2])
                if any(y - 1.5 < h < y + 1.0 for h in H) or not any(h <= y - 1.5 for h in H):
                    bad.append((fr, round(y, 1), [round(h, 1) for h in H][:5]))
        print([round(v) for v in f["pos"]], "->", [round(v) for v in q["pos"]], round(d), "dy", round(q["bracket_y"] - f["bracket_y"], 1), "bad", bad[:4])
