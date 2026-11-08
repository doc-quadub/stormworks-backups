"""python3 road_strip_probe.py <rom> <tile> <local x> <local z>: strip quads of the map geometry
containing the point, with their vertices (x, y, z, u, v)."""
import sys, math
sys.path.insert(0, ".")
from maproads import load, point_in
rom, tile, x, z = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
L = load(rom, tile)
for li, v in enumerate(L):
    for i in range(len(v) - 2):
        if point_in((x, z), v[i], v[i + 1], v[i + 2]):
            print("layer", li, "tri", i, [tuple(round(c, 2) for c in v[j]) for j in range(max(0, i - 2), min(len(v), i + 5))])
