"""python3 colour_map.py <rom> <tile> <x> <z> <half_cells> <step>: ascii map of terrain colours (world X/Y)."""
import sys, os, re, struct, collections
import numpy as np
sys.path.insert(0, ".")
import build_world
from swmesh import read_vertex_array
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom = sys.argv[1]
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
tile = sys.argv[2]
X, Z, H, ST = float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5]), float(sys.argv[6])
m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
c = Colours(tile); legend = {}; chars = "abcdefghijklmnopqrstuvwxyz0123456789"
for iz in range(H, -H - 1, -1):
    row = ""
    for ix in range(-H, H + 1):
        col = c.at(X + ix * ST - ox, Z + iz * ST - oz)
        if col not in legend: legend[col] = chars[len(legend)]
        row += legend[col]
    print(row)
print(legend)
