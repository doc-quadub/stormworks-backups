"""Dry run: overhead line plan for the whole Sawyer mainland, with diagnostics.
usage: python3 diag_full.py <rom_dir> <work_dir_with_scan.json> <out_json>"""
import json, math, sys, time
sys.path.insert(0, ".")
import build_world, catenary, signs, rails

rom, work, out = sys.argv[1:4]
scan = json.load(open(work + "/scan.json"))
t0 = time.time()
plan = catenary.plan_catenary(rom, scan["tiles"], lambda tile: build_world.prepare_graph(rom, tile, scan))
t1 = time.time()
print("plan time %.1f s" % (t1 - t0))

net = signs.build_network(scan["tiles"], lambda tile: build_world.prepare_graph(rom, tile, scan))
ground = signs.Ground(rom)
edges = {(a, b) for a in net.links for b in net.links[a] if a < b}
sub = {}
for a, b in sorted(edges):
    sub.setdefault(a, []).append(b); sub.setdefault(b, []).append(a)
chains = rails.build_chains({k: (net.nodes_world[k], v) for k, v in sub.items()})
total_len = 0.0
covered = []          # [x0, z0, x1, z1, length, min_cover_above_rail, max_cover]
chain_lengths = []
for chain, _ in chains:
    pts = [net.nodes_world[k] for k in chain]
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.dist((a[0], a[2]), (b[0], b[2])))
    L = cum[-1]; total_len += L; chain_lengths.append(L)
    n = max(1, int(L // 3.0)); cur = None
    for i in range(n + 1):
        q = catenary._polyline_at(pts, cum, L * i / n)[0]
        hs = [h - q[1] for h in ground.heights(q[0], q[2]) if q[1] + catenary.COVER_LOW < h < q[1] + catenary.COVER_HIGH]
        if hs:
            if cur is None:
                cur = [q[0], q[2], q[0], q[2], 0.0, min(hs), max(hs), L * i / n]
            cur[2], cur[3] = q[0], q[2]; cur[5] = min(cur[5], min(hs)); cur[6] = max(cur[6], max(hs))
            cur[4] = L * i / n - cur[7]
        elif cur is not None:
            covered.append(cur[:7]); cur = None
    if cur is not None:
        covered.append(cur[:7])
print("mainland track length %.1f km, chains %d" % (total_len / 1000, len(chains)))
covered.sort(key=lambda c: -c[4])
print("covered stretches", len(covered), "total %.0f m" % sum(c[4] for c in covered))
for c in covered[:40]:
    print("  (%.0f,%.0f)-(%.0f,%.0f) len %.0f cover %.1f..%.1f" % tuple(c))

# poles: how far below the rail is the ground at the pole
S = plan["supports"]
air = []
for i, sp in enumerate(S):
    if sp.get("portal") is not None or "pole" not in sp:
        continue
    rail_y = sp["track"][1]
    hs = ground.heights(sp["pole"][0], sp["pole"][2])
    below = [rail_y - h for h in hs if h <= rail_y + 3.0]
    gap = min(below) if below else 999.0
    if gap > 3.0:
        air.append((gap, sp["pole"][0], sp["pole"][2], i))
air.sort(reverse=True)
print("poles with ground more than 3 m below rail:", len(air))
for g in air[:30]:
    print("  gap %.1f at (%.0f, %.0f) support %d" % g)
runs = {}
for sp in S:
    runs.setdefault(sp["run"], []).append(sp["s"])
rl = sorted((max(v) - min(v) for v in runs.values()), reverse=True)
print("wire runs", len(rl), "longest", [round(x) for x in rl[:15]], "over 1600 m:", sum(1 for x in rl if x > 1600))
json.dump({"plan": plan, "covered": covered, "air": air, "chain_lengths": chain_lengths,
           "pos": {"%s|%s" % k: v for k, v in net.nodes_world.items()},
           "links": {"%s|%s" % k: ["%s|%s" % o for o in v] for k, v in net.links.items()}},
          open(out, "w"))
