"""For the sandy recolour hits of arid tiles: how far the other corners of their triangles reach from
the track (big open-sand triangles smear the ballast colour). Histogram, split by user boxes."""
import sys, os, json, math, struct, collections
import numpy as np
sys.path.insert(0, ".")
import build_world
from swmesh import read_vertex_array
rom = sys.argv[1]; scan = json.load(open(sys.argv[2]))
BOXES = [(1900, 3200, -29700, -26900), (300, 800, -26700, -26500)]     # user's blob stretches (world)
def world(tile, x, z):
    import re
    m = re.fullmatch(r"arid_island_(\d+)_(\d+)", tile)
    return x + 1000 * int(m.group(1)) - 24000, z + 1000 * int(m.group(2)) - 40000
hist = collections.Counter(); inbox = collections.Counter()
for tile in scan["tiles"]:
    if not tile.startswith("arid_island_"): continue
    text = build_world.read_text(build_world.tile_xml_path(rom, tile))
    nodes = build_world.read_tracks(build_world.tile_xml_path(rom, tile))
    grid = build_world.SegmentGrid(nodes)
    for file_name, matrix in build_world.mesh_entries(text):
        hits = {i: (r, g, b) for i, r, g, b in scan["recolor"].get(file_name, [])}
        path = os.path.join(rom, *file_name.split("/"))
        data, verts = read_vertex_array(path)
        n = len(verts)
        sandy = [i for i in hits if (int(verts["r"][i]), int(verts["g"][i]), int(verts["b"][i])) == (188, 157, 125)]
        if not sandy: continue
        off = 14 + 28 * n
        count = struct.unpack_from("<I", data, off)[0]
        idx = np.frombuffer(data, dtype="<u2", count=count, offset=off + 4).reshape(-1, 3)
        sset = set(sandy)
        dist = {}
        def d(i):
            if i not in dist:
                x, y, z = build_world.transform_point((float(verts["x"][i]), float(verts["y"][i]), float(verts["z"][i])), matrix)
                dist[i] = grid.distance(x, z)
            return dist[i]
        reach = {}
        mask = np.isin(idx, np.array(sandy)).any(axis=1)
        for tri in idx[mask]:
            far = max(d(int(j)) for j in tri)
            for j in tri:
                if int(j) in sset: reach[int(j)] = max(reach.get(int(j), 0.0), far)
        for i, r in reach.items():
            b = min(int(r // 2) * 2, 40) if math.isfinite(r) else 99
            x, y, z = build_world.transform_point((float(verts["x"][i]), float(verts["y"][i]), float(verts["z"][i])), matrix)
            wx, wz = world(tile, x, z)
            box = any(x0 <= wx <= x1 and z0 <= wz <= z1 for x0, x1, z0, z1 in BOXES)
            (inbox if box else hist)[b] += 1
print("outside boxes:", sorted(hist.items()))
print("in the user's boxes:", sorted(inbox.items()))
