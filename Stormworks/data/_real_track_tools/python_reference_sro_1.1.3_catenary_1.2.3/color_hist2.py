"""Area per vertex colour in terrain meshes of a tile family (arid_mainland / mainland)."""
import sys, os, collections, struct
import numpy as np
sys.path.insert(0, ".")
from swmesh import read_vertex_array
rom, folder, prefix = sys.argv[1], sys.argv[2], sys.argv[3]
tot = collections.Counter()
d = os.path.join(rom, "meshes", folder)
for f in sorted(os.listdir(d)):
    if not (f.startswith(prefix) and f.endswith(".mesh")): continue
    data, v = read_vertex_array(os.path.join(d, f))
    n = len(v); off = 14 + 28 * n
    cnt = struct.unpack_from("<I", data, off)[0]
    idx = np.frombuffer(data, dtype="<u2", count=cnt, offset=off + 4).reshape(-1, 3)
    a, b, c = idx[:, 0], idx[:, 1], idx[:, 2]
    area = np.abs((v["x"][b] - v["x"][a]) * (v["z"][c] - v["z"][a]) - (v["x"][c] - v["x"][a]) * (v["z"][b] - v["z"][a])) / 2
    col = np.stack([v["r"][a], v["g"][a], v["b"][a]], axis=1)
    keys, inv = np.unique(col, axis=0, return_inverse=True)
    sums = np.bincount(inv.ravel(), weights=area)
    for k, s_ in zip(keys, sums): tot[tuple(int(x) for x in k)] += s_
for c, a in tot.most_common(30): print(c, round(a))
