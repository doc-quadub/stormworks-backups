"""Along every mainland track (3 m steps): what is under / beside / above it.
Classes: truss (dark truss colour 9-14 m above the rail within 10 m), deck (no ground within 3 m
below the rail at 3.3 m on both sides), cliff (one side only)."""
import sys, math, json, os, collections
sys.path.insert(0, ".")
from swmesh import read_mesh_full
import signs
R = os.path.expandvars("$HOME/mnt/Stormworks/rom")
D = json.load(open("work/diag.json"))
pos, links = D["pos"], D["links"]
ground = signs.Ground(R)
TRUSS = (45, 34, 32)
cache = {}
def truss_grid(tile):
    if tile not in cache:
        g = collections.defaultdict(list)
        tx, tz = map(int, tile.split("_")[-2:])
        ox, oz = tx * 1000 - 8000, tz * 1000 - 12000
        try:
            m = read_mesh_full(R + "/meshes/mainland/%s.mesh" % tile)
            for v in m.vertices:
                if (int(v[3]), int(v[4]), int(v[5])) == TRUSS:
                    x, z = v[0] + ox, v[2] + oz
                    g[(int(x // 10), int(z // 10))].append((x, v[1], z))
        except Exception as e:
            print("no mesh", tile, e)
        cache[tile] = g
    return cache[tile]
def tile_of(x, z):
    return "mega_island_%d_%d" % (int(math.floor((x + 8500) / 1000)), int(math.floor((z + 12500) / 1000)))
res = []
seen = set()
for k, ls in links.items():
    for o in ls:
        if (o, k) in seen or o not in pos: continue
        seen.add((k, o))
        p, q = pos[k], pos[o]
        L = math.hypot(q[0] - p[0], q[2] - p[2])
        if L < 1e-6: continue
        d = ((q[0] - p[0]) / L, (q[2] - p[2]) / L); n = (d[1], -d[0])
        for i in range(max(1, int(L // 3))):
            f = i * 3 / L
            x, y, z = p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f, p[2] + (q[2] - p[2]) * f
            g = truss_grid(tile_of(x, z))
            truss = False
            for cx in (int(x // 10) - 1, int(x // 10), int(x // 10) + 1):
                for cz in (int(z // 10) - 1, int(z // 10), int(z // 10) + 1):
                    for (vx, vy, vz) in g.get((cx, cz), ()):
                        if 9 < vy - y < 14 and math.hypot(vx - x, vz - z) < 10:
                            truss = True; break
            sides = []
            for s in (1, -1):
                hs = ground.heights(x + n[0] * s * 3.3, z + n[1] * s * 3.3)
                sides.append(any(y - 3.0 <= h <= y + 3.0 for h in hs))
            cls = "truss" if truss else ("deck" if not any(sides) else ("cliff" if not all(sides) else "ok"))
            res.append((cls, x, z, y))
c = collections.Counter(r[0] for r in res)
print({k: round(v * 3) for k, v in c.items()}, "metres")
# cluster non-ok samples
clusters = []
for cls in ("truss", "deck", "cliff"):
    pts = [r for r in res if r[0] == cls]
    used = [False] * len(pts)
    for i, r in enumerate(pts):
        if used[i]: continue
        group = [r]; used[i] = True; j = 0
        while j < len(group):
            a = group[j]; j += 1
            for m, b in enumerate(pts):
                if not used[m] and math.hypot(a[1] - b[1], a[2] - b[2]) < 12:
                    used[m] = True; group.append(b)
        clusters.append((cls, len(group) * 3, sum(g[1] for g in group) / len(group), sum(g[2] for g in group) / len(group), min(g[3] for g in group)))
clusters.sort(key=lambda c: (c[0], -c[1]))
for c in clusters:
    if c[1] >= 9:
        print("%-6s %5d m track  at (%.0f, %.0f) rail %.1f" % c)
json.dump(res, open("work/bridges.json", "w"))
