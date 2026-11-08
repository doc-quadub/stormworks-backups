"""python3 track_quality.py <rom> <scan.json> <family prefix, e.g. arid_island> [out.json]
Track geometry check of the processed (smoothed, raised) graph of every tile of a family:
length, kinks at nodes, grade changes, S-bends, turnouts that diverge very slowly, unlinked nodes
close together. World coordinates (in-game X, Y)."""
import sys, json, math, re, collections, time
sys.path.insert(0, ".")
import build_world, smooth

ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom, scan_path, family = sys.argv[1], sys.argv[2], sys.argv[3]
out_path = sys.argv[4] if len(sys.argv) > 4 else None
scan = json.load(open(scan_path))
tiles = [t for t in scan["tiles"] if re.fullmatch(family + r"_\d+_\d+", t)] if family != "all" else list(scan["tiles"])

def origin(tile):
    m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
    if m is None or m.group(1) not in ORIGINS:
        return 0, 0                      # local coordinates (modular and other tiles)
    ox, oz = ORIGINS[m.group(1)]
    return 1000 * int(m.group(2)) + ox, 1000 * int(m.group(3)) + oz

def d2(a, b):
    return math.hypot(b[0] - a[0], b[2] - a[2])

def heading(a, b):
    return math.atan2(b[2] - a[2], b[0] - a[0])

def turn(h1, h2):
    return (h2 - h1 + math.pi) % (2 * math.pi) - math.pi

length = vanilla_length = 0.0
kinks, grades, sbends, slow, close = [], [], [], [], []
t0 = time.time()
for tile in tiles:
    ox, oz = origin(tile)
    van = build_world.read_tracks(build_world.tile_xml_path(rom, tile))
    seen = set()
    for k, (p, links) in van.items():
        for o in links:
            if o in van and frozenset((k, o)) not in seen:
                seen.add(frozenset((k, o))); vanilla_length += d2(p, van[o][0])
    nodes, node_y, inserted, border_tangents, stray = build_world.prepare_graph(rom, tile, scan)
    stray = set(stray)
    seen = set()
    for k, (p, links) in nodes.items():
        if k in stray: continue
        for o in links:
            if o in nodes and frozenset((k, o)) not in seen:
                seen.add(frozenset((k, o))); length += d2(p, nodes[o][0])
    pairs = smooth.node_pairs(nodes)
    W = lambda p: (round(p[0] + ox), round(p[2] + oz))
    for k, (p, links) in nodes.items():
        if k in stray: continue
        nb = [o for o in dict.fromkeys(links) if o in nodes and o != k]
        junction = smooth.is_junction(nodes, k)
        for a in nb:
            b = pairs[k].get(a)
            if b is None or a > b: continue
            pa, pb = nodes[a][0], nodes[b][0]
            l1, l2 = d2(pa, p), d2(p, pb)
            if l1 < 0.05 or l2 < 0.05: continue
            th = abs(turn(heading(pa, p), heading(p, pb)))
            ya, yk, yb = node_y.get(a, pa[1]), node_y.get(k, p[1]), node_y.get(b, pb[1])
            g1, g2 = (yk - ya) / l1, (yb - yk) / l2
            if math.degrees(th) > 1.0:
                kinks.append((round(math.degrees(th), 2), round(l1, 1), round(l2, 1), junction, k.startswith("train_rt"), W(p), tile, k))
            if abs(g2 - g1) > 0.006:
                grades.append((round(abs(g2 - g1) * 100, 2), round(l1, 1), round(l2, 1), round(yb - ya, 2), W(p), tile, k))
    # S-bends: a node turning one way next to a node turning the other way, both > 0.4 deg, within 15 m
    for k, (p, links) in nodes.items():
        nb = [o for o in dict.fromkeys(links) if o in nodes and o != k]
        if len(nb) != 2 or smooth.is_junction(nodes, k): continue
        a, b = nb
        t1 = turn(heading(nodes[a][0], p), heading(p, nodes[b][0]))
        nb2 = [o for o in dict.fromkeys(nodes[b][1]) if o in nodes and o != b and o != k]
        if len(nb2) != 1 or smooth.is_junction(nodes, b): continue
        t2 = turn(heading(p, nodes[b][0]), heading(nodes[b][0], nodes[nb2[0]][0]))
        if t1 * t2 < 0 and min(abs(t1), abs(t2)) > math.radians(0.4) and d2(p, nodes[b][0]) < 15:
            sbends.append((round(math.degrees(min(abs(t1), abs(t2))), 2), round(d2(p, nodes[b][0]), 1), W(p), tile, k, b))
    # turnouts: angle between the two branches that leave on the same side, and their gap 20 m on
    for k, (p, links) in nodes.items():
        nb = [o for o in dict.fromkeys(links) if o in nodes and o != k]
        if not smooth.is_junction(nodes, k): continue
        dirs = [(o, heading(p, nodes[o][0])) for o in nb]
        for i in range(len(dirs)):
            for j in range(i + 1, len(dirs)):
                ang = abs(turn(dirs[i][1], dirs[j][1]))
                if ang < math.radians(90):
                    def walk(start, dist):
                        prev, cur, s = k, start, d2(p, nodes[start][0])
                        while s < dist:
                            nx = [o for o in nodes[cur][1] if o in nodes and o != prev and o != cur]
                            if len(nx) != 1: break
                            prev, cur = cur, nx[0]; s += d2(nodes[prev][0], nodes[cur][0])
                        return nodes[cur][0]
                    gap10 = d2(walk(dirs[i][0], 10), walk(dirs[j][0], 10))
                    gap25 = d2(walk(dirs[i][0], 25), walk(dirs[j][0], 25))
                    slow.append((round(math.degrees(ang), 2), round(gap10, 2), round(gap25, 2), W(p), tile, k, dirs[i][0], dirs[j][0]))
    # unlinked nodes closer than 1.5 m
    grid = collections.defaultdict(list)
    for k, (p, _) in nodes.items():
        grid[(int(p[0] // 2), int(p[2] // 2))].append(k)
    for k, (p, links) in nodes.items():
        for ix in (-1, 0, 1):
            for iz in (-1, 0, 1):
                for o in grid[(int(p[0] // 2) + ix, int(p[2] // 2) + iz)]:
                    if o > k and o not in links and k not in nodes[o][1] and d2(p, nodes[o][0]) < 1.5:
                        dv = d2(van[k][0], van[o][0]) if k in van and o in van else None
                        if dv is None or dv > d2(p, nodes[o][0]) + 0.05:
                            close.append((round(d2(p, nodes[o][0]), 2), None if dv is None else round(dv, 2), W(p), tile, k, o))
print("%s: %d tiles, track %.1f km (vanilla graph %.1f km), %.0f s" % (family, len(tiles), length / 1000, vanilla_length / 1000, time.time() - t0))
def show(name, rows, n=15):
    rows = sorted(rows, reverse=True)
    print("%s: %d" % (name, len(rows)))
    for r in rows[:n]: print("   ", r)
show("kinks > 1 deg (deg, l1, l2, junction, inserted node, X/Y)", kinks)
print("   kinks > 2 deg:", sum(r[0] > 2 for r in kinks), " > 3 deg:", sum(r[0] > 3 for r in kinks), " at junction:", sum(r[3] for r in kinks))
show("grade changes > 0.6 % (dg %, l1, l2, dy)", grades)
print("   > 1.5 %:", sum(r[0] > 1.5 for r in grades), " > 3 %:", sum(r[0] > 3 for r in grades))
show("S-bends", sbends)
slow_rows = sorted(slow)
print("turnouts: %d, diverging < 2 deg: %d, < 1 deg: %d" % (len(slow), sum(r[0] < 2 for r in slow), sum(r[0] < 1 for r in slow)))
for r in slow_rows[:15]: print("   ", r)
show("unlinked nodes brought closer than 1.5 m by the processing (dist, vanilla dist, X/Y, tile, a, b)", [(-r[0],) + r[1:] for r in close], 40)
if out_path:
    json.dump({"kinks": kinks, "grades": grades, "sbends": sbends, "turnouts": slow, "close": close}, open(out_path, "w"))
