import sys, math, statistics
sys.path.insert(0, ".")
from maproads import load, ribbons, point_in
rom = sys.argv[1]
for arg in sys.argv[2:]:
    x, z = map(float, arg.split(","))
    tx = int(math.floor((x + 8500) / 1000)); tz = int(math.floor((z + 12500) / 1000))
    L = load(rom, "mega_island_%d_%d" % (tx, tz)); p = (x - (tx * 1000 - 8000), z - (tz * 1000 - 12000))
    res = []
    for li, layer in enumerate(L):
        q = ribbons(layer)
        if not q: continue
        w = statistics.median(e[4] for e in q)
        inside = any(point_in(p, a, b, c) or point_in(p, a, c, d) for a, b, c, d, _ in q)
        near = min(min(math.hypot(v[0] - p[0], v[2] - p[1]) for v in e[:4]) for e in q)
        res.append((li, len(q), round(w, 1), inside, round(near, 1)))
    print(arg, res)
