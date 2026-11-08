"""python3 road_map.py rom x z half plan.json : ascii map of terrain colours around a point, poles marked."""
import sys, os, math, json
sys.path.insert(0, ".")
from swmesh import read_mesh_full
rom = sys.argv[1]; X, Z, H = float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
STEP = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
plan = json.load(open(sys.argv[5]))
grids = {}
def load(tile):
    if tile in grids: return grids[tile]
    m = read_mesh_full(os.path.join(rom, "meshes", "mainland", tile + ".mesh"))
    V, I = m.vertices, m.indices; g = {}
    for i in range(0, len(I) - 2, 3):
        a, b, c = V[I[i]], V[I[i+1]], V[I[i+2]]
        for ix in range(int(math.floor(min(a[0], b[0], c[0]) / 8)), int(math.floor(max(a[0], b[0], c[0]) / 8)) + 1):
            for iz in range(int(math.floor(min(a[2], b[2], c[2]) / 8)), int(math.floor(max(a[2], b[2], c[2]) / 8)) + 1):
                g.setdefault((ix, iz), []).append((a, b, c))
    grids[tile] = g; return g
def col(x, z):
    tx = int(math.floor((x + 8500) / 1000)); tz = int(math.floor((z + 12500) / 1000))
    g = load("mega_island_%d_%d" % (tx, tz)); lx, lz = x - (tx * 1000 - 8000), z - (tz * 1000 - 12000)
    best = None
    for a, b, c in g.get((int(math.floor(lx / 8)), int(math.floor(lz / 8))), ()):
        d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
        if abs(d) < 1e-12: continue
        l1 = ((lx-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(lz-a[2]))/d; l2 = ((b[0]-a[0])*(lz-a[2]) - (lx-a[0])*(b[2]-a[2]))/d
        if l1 < -1e-6 or l2 < -1e-6 or l1+l2 > 1+1e-6: continue
        y = a[1] + l1*(b[1]-a[1]) + l2*(c[1]-a[1])
        if best is None or y > best[0]: best = (y, a[3:6])
    return best[1] if best else None
legend = {}
chars = "abcdefghijklmnopqrstuvwxyz"
poles = [(sp["pole"][0], sp["pole"][2]) for sp in plan["supports"] if sp.get("pole") and not sp.get("dropped") and sp.get("pole_owner") is None]
for iz in range(-H, H + 1):
    row = ""
    for ix in range(-H, H + 1):
        x, z = X + ix * STEP, Z - iz * STEP
        if any(abs(px - x) < STEP / 2 and abs(pz - z) < STEP / 2 for px, pz in poles): row += "P"; continue
        c = col(x, z)
        if c not in legend: legend[c] = chars[len(legend)]
        row += legend[c]
    print(row)
print(legend)
