"""python3 map_lines.py <rom> <tile> <world x> <world z> : for each map-geometry strip layer, the
nearest centreline segment to the point (segments joining successive cross pairs, jumps skipped)."""
import sys, math, re
sys.path.insert(0, ".")
from maproads import load
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom, tile, X, Z = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
x, z = X - ox, Z - oz
def seg_d(p, a, b):
    dx, dz = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dz * dz
    t = 0 if L2 == 0 else max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dz)
for li, v in enumerate(L := load(rom, tile)):
    mids = []
    for i in range(0, len(v) - 1, 2):
        a, b = v[i], v[i + 1]
        if math.hypot(a[0] - b[0], a[2] - b[2]) < 8:
            mids.append(((a[0] + b[0]) / 2, (a[2] + b[2]) / 2, a[3]))
        else:
            mids.append(None)
    best = None; count = 0
    for p, q in zip(mids, mids[1:]):
        if p is None or q is None or math.hypot(p[0] - q[0], p[1] - q[1]) > 60: continue
        count += 1
        d = seg_d((x, z), p, q)
        if best is None or d < best[0]: best = (round(d, 1), [round(p[0] + ox), round(p[1] + oz)], [round(q[0] + ox), round(q[1] + oz)])
    print("layer", li, "segments", count, "nearest", best)
