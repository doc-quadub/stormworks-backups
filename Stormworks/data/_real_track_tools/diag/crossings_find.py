"""python3 crossings_find.py <rom> <scan.json> : level crossings (road colour on both sides of a track
for a short stretch) on the mega and arid tiles, by candidate road colour."""
import sys, os, re, json, math, struct, collections
import numpy as np
sys.path.insert(0, ".")
import build_world, rails
from swmesh import read_vertex_array
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
CAND = [(45, 41, 45), (58, 58, 58), (36, 29, 25), (0, 0, 0), (77, 77, 62), (88, 62, 45), (240, 240, 240)]
rom, scan = sys.argv[1], json.load(open(sys.argv[2]))
class Colours:
    def __init__(self, tile):
        self.tris = collections.defaultdict(list)
        text = build_world.read_text(build_world.tile_xml_path(rom, tile))
        for fn, mx in build_world.mesh_entries(text):
            if "mainland/" not in fn: continue
            data, v = read_vertex_array(os.path.join(rom, *fn.split("/")))
            n = len(v); off = 14 + 28 * n
            cnt = struct.unpack_from("<I", data, off)[0]
            idx = np.frombuffer(data, dtype="<u2", count=cnt, offset=off + 4).reshape(-1, 3)
            P = [build_world.transform_point((float(v["x"][i]), float(v["y"][i]), float(v["z"][i])), mx) for i in range(n)]
            C = [(int(v["r"][i]), int(v["g"][i]), int(v["b"][i])) for i in range(n)]
            for a, b, c in idx:
                pa, pb, pc = P[a], P[b], P[c]
                for ix in range(int(min(pa[0], pb[0], pc[0]) // 8), int(max(pa[0], pb[0], pc[0]) // 8) + 1):
                    for iz in range(int(min(pa[2], pb[2], pc[2]) // 8), int(max(pa[2], pb[2], pc[2]) // 8) + 1):
                        self.tris[(ix, iz)].append((pa, pb, pc, C[a]))
    def at(self, x, z):
        best = None
        for a, b, c, col in self.tris.get((int(x // 8), int(z // 8)), ()):
            d = (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])
            if abs(d) < 1e-12: continue
            l1 = ((x - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (z - a[2])) / d; l2 = ((b[0] - a[0]) * (z - a[2]) - (x - a[0]) * (b[2] - a[2])) / d
            if l1 < 0 or l2 < 0 or l1 + l2 > 1: continue
            y = a[1] + l1 * (b[1] - a[1]) + l2 * (c[1] - a[1])
            if best is None or y > best[0]: best = (y, col)
        return best[1] if best else None
found = collections.defaultdict(list)
fam = sys.argv[3]
part = sys.argv[4] if len(sys.argv) > 4 else "0/1"
pi, pn = map(int, part.split("/"))
todo = [t for t in scan["tiles"] if t.startswith(fam)]
for tile in todo[pi::pn]:
    m = re.fullmatch(r"(mega_island|arid_island)_(\d+)_(\d+)", tile)
    if not m: continue
    ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
    nodes, node_y, ins, bt, stray = build_world.prepare_graph(rom, tile, scan)
    nodes = {k: v for k, v in nodes.items() if k not in stray}
    cols = Colours(tile)
    for chain, _ in rails.build_chains(nodes):
        pts = rails.densify(rails.chain_points(chain, nodes, node_y), 1.0)
        runs = collections.defaultdict(list)
        for i in range(1, len(pts) - 1):
            p = pts[i]; t = rails.horizontal_dir(pts[i - 1], pts[i + 1]); n = (-t[2], t[0])
            sides = []
            for sgn in (1, -1):
                sides.append({cols.at(p[0] + n[0] * sgn * d, p[2] + n[1] * sgn * d) for d in (4.5, 6.0)})
            for c in CAND:
                hit = c in sides[0] and c in sides[1]
                runs[c].append(hit)
        for c, flags in runs.items():
            i = 0
            while i < len(flags):
                if flags[i]:
                    j = i
                    while j < len(flags) and flags[j]: j += 1
                    if 3 <= j - i <= 30:
                        p = pts[(i + j) // 2 + 1]
                        found[c].append((j - i, round(p[0] + ox), round(p[2] + oz), tile))
                    i = j
                else:
                    i += 1
for c in CAND:
    rows = found.get(c, [])
    print(c, len(rows), rows[:12])
