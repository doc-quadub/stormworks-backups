"""python3 branch_map.py <rom> <scan.json> <tile> <x> <z> <radius>: track pieces (between switches and
ends) of the processed graph near a world point: ends, length, and the switch/end nodes."""
import sys, json, math, re
sys.path.insert(0, ".")
import build_world, smooth, rails
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom, scan_path, tile = sys.argv[1], sys.argv[2], sys.argv[3]
X, Z, R = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
scan = json.load(open(scan_path))
m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
nodes, node_y, ins, bt, stray = build_world.prepare_graph(rom, tile, scan)
nodes = {k: v for k, v in nodes.items() if k not in stray}
for chain, through in rails.build_chains(nodes):
    pts = [nodes[k][0] for k in chain]
    if min(math.hypot(p[0] + ox - X, p[2] + oz - Z) for p in pts) > R: continue
    L = sum(math.dist((a[0], a[2]), (b[0], b[2])) for a, b in zip(pts, pts[1:]))
    deg = lambda k: len([o for o in nodes[k][1] if o in nodes and o != k])
    print("%-14s (%7.1f,%8.1f) d%d -> %-14s (%7.1f,%8.1f) d%d  len %6.1f" % (
        chain[0], pts[0][0] + ox, pts[0][2] + oz, deg(chain[0]), chain[-1], pts[-1][0] + ox, pts[-1][2] + oz, deg(chain[-1]), L))
