import sys, os, math
sys.path.insert(0, ".")
from swmesh import read_mesh_full
rom = sys.argv[1]
for arg in sys.argv[2:]:
    x, z = map(float, arg.split(","))
    tx = int(math.floor((x + 8500) / 1000)); tz = int(math.floor((z + 12500) / 1000))
    tile = "mega_island_%d_%d" % (tx, tz)
    m = read_mesh_full(os.path.join(rom, "meshes", "mainland", tile + ".mesh"))
    print(tile, [(s.index_start, s.index_count, s.shader, s.name) for s in m.submeshes])
    lx, lz = x - (tx * 1000 - 8000), z - (tz * 1000 - 12000)
    V, I = m.vertices, m.indices
    for si, s in enumerate(m.submeshes):
        for i in range(s.index_start, s.index_start + s.index_count - 2, 3):
            a, b, c = V[I[i]], V[I[i+1]], V[I[i+2]]
            if max(a[0], b[0], c[0]) < lx or min(a[0], b[0], c[0]) > lx or max(a[2], b[2], c[2]) < lz or min(a[2], b[2], c[2]) > lz: continue
            d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
            if abs(d) < 1e-12: continue
            l1 = ((lx-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(lz-a[2]))/d; l2 = ((b[0]-a[0])*(lz-a[2]) - (lx-a[0])*(b[2]-a[2]))/d
            if l1 < -1e-6 or l2 < -1e-6 or l1+l2 > 1+1e-6: continue
            print("  sub", si, "y", round(a[1] + l1*(b[1]-a[1]) + l2*(c[1]-a[1]), 2), a[3:7])
