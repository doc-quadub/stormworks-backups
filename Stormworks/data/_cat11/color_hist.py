import sys, os, math, collections
sys.path.insert(0, ".")
from swmesh import read_mesh_full
rom = sys.argv[1]
tot = collections.Counter()
for tile in sys.argv[2:]:
    m = read_mesh_full(os.path.join(rom, "meshes", "mainland", tile + ".mesh"))
    V, I = m.vertices, m.indices
    for i in range(0, len(I) - 2, 3):
        a, b, c = V[I[i]], V[I[i+1]], V[I[i+2]]
        area = abs((b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])) / 2
        tot[a[3:6]] += area
for col, ar in tot.most_common(25): print(col, round(ar))
