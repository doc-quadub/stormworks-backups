import sys, math, json, os, collections
sys.path.insert(0, ".")
from swmesh import read_mesh_full
R = os.path.expandvars("$HOME/mnt/Stormworks/rom")
D = json.load(open("work/diag.json"))
pos = D["pos"]; links = D["links"]
def probe(wx, wz, half=40):
    k = min(pos, key=lambda k: math.hypot(pos[k][0] - wx, pos[k][2] - wz))
    p = pos[k]; o = links[k][0]; q = pos[o]
    d = (q[0] - p[0], q[2] - p[2]); L = math.hypot(*d); d = (d[0] / L, d[1] / L); n = (d[1], -d[0])
    tx, tz = int(math.floor((p[0] + 8500) / 1000)), int(math.floor((p[2] + 12500) / 1000))
    m = read_mesh_full(R + "/meshes/mainland/mega_island_%d_%d.mesh" % (tx, tz))
    ox, oz = tx * 1000 - 8000, tz * 1000 - 12000
    cols = collections.Counter(); pts = collections.defaultdict(list)
    for v in m.vertices:
        x, y, z = v[0] + ox, v[1], v[2] + oz
        a = (x - p[0]) * d[0] + (z - p[2]) * d[1]; b = (x - p[0]) * n[0] + (z - p[2]) * n[1]
        if abs(a) < half and abs(b) < 12 and y > p[1] - 3:
            c = tuple(int(c) for c in v[3:6]); cols[c] += 1; pts[c].append((a, b, y - p[1]))
    print("node", k, "rail_top %.2f" % p[1])
    for c, cnt in cols.most_common(6):
        P = pts[c]
        print("  ", c, cnt, "along %.1f..%.1f lateral %.2f..%.2f above rail %.2f..%.2f" % (
            min(t[0] for t in P), max(t[0] for t in P), min(t[1] for t in P), max(t[1] for t in P),
            min(t[2] for t in P), max(t[2] for t in P)))
        if c == (45, 34, 32):
            hs = sorted(set(round(t[2], 1) for t in P)); bs = sorted(set(round(t[1], 1) for t in P))
            print("     heights", hs[:40]); print("     laterals", bs[:40])
for a in sys.argv[1:]:
    probe(*map(float, a.split(",")))
