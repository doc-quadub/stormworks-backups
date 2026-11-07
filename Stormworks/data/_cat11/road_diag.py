"""python3 road_diag.py rom x,z [x,z ...] : vertex colours of the terrain mesh triangles at and around points."""
import sys, os, math, collections
sys.path.insert(0, ".")
from swmesh import read_mesh_full
rom = sys.argv[1]
cache = {}
def tri_at(x, z):
    tx = int(math.floor((x + 8500) / 1000)); tz = int(math.floor((z + 12500) / 1000))
    tile = "mega_island_%d_%d" % (tx, tz)
    ox, oz = tx * 1000 - 8000, tz * 1000 - 12000
    if tile not in cache:
        m = read_mesh_full(os.path.join(rom, "meshes", "mainland", tile + ".mesh"))
        cache[tile] = m
    m = cache[tile]; lx, lz = x - ox, z - oz
    V = m.vertices; I = m.indices; best = None
    for i in range(0, len(I) - 2, 3):
        a, b, c = V[I[i]], V[I[i+1]], V[I[i+2]]
        if max(a[0], b[0], c[0]) < lx or min(a[0], b[0], c[0]) > lx or max(a[2], b[2], c[2]) < lz or min(a[2], b[2], c[2]) > lz: continue
        d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
        if abs(d) < 1e-12: continue
        l1 = ((lx-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(lz-a[2]))/d; l2 = ((b[0]-a[0])*(lz-a[2]) - (lx-a[0])*(b[2]-a[2]))/d
        if l1 < -1e-6 or l2 < -1e-6 or l1+l2 > 1+1e-6: continue
        y = a[1] + l1*(b[1]-a[1]) + l2*(c[1]-a[1])
        if best is None or y > best[0]: best = (round(y, 2), a[3:6], b[3:6], c[3:6])
    return best
for arg in sys.argv[2:]:
    x, z = map(float, arg.split(","))
    print("at", arg, tri_at(x, z))
    for dx in (-6, -3, 3, 6):
        print("  dx", dx, tri_at(x + dx, z), " dz", dx, tri_at(x, z + dx))
