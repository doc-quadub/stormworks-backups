"""python3 graph_dump.py <rom> <scan.json> <out.json> tile...: processed graph (world coords) of tiles."""
import sys, json, re
sys.path.insert(0, ".")
import build_world
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom, scan_path, out = sys.argv[1:4]
scan = json.load(open(scan_path))
res = {}
for tile in sys.argv[4:]:
    m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
    ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
    nodes, node_y, ins, bt, stray = build_world.prepare_graph(rom, tile, scan)
    res[tile] = {k: [p[0] + ox, p[2] + oz, [o for o in ls if o in nodes and o not in stray and o != k]]
                 for k, (p, ls) in nodes.items() if k not in stray}
json.dump(res, open(out, "w"))
