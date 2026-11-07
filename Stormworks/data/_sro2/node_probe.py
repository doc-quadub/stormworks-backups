"""python3 node_probe.py <rom> <scan.json> <tile> <x> <z> <radius> : vanilla and processed nodes near a
world point (arid / mega families), with links, local and world coordinates."""
import sys, json, math, re
sys.path.insert(0, ".")
import build_world
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
rom, scan_path, tile = sys.argv[1], sys.argv[2], sys.argv[3]
X, Z, R = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
scan = json.load(open(scan_path))
m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
ox, oz = ORIGINS[m.group(1)]; ox += 1000 * int(m.group(2)); oz += 1000 * int(m.group(3))
van = build_world.read_tracks(build_world.tile_xml_path(rom, tile))
nodes, node_y, inserted, bt, stray = build_world.prepare_graph(rom, tile, scan)
for name, g in (("vanilla", van), ("processed", nodes)):
    print("==", name)
    rows = []
    for k, (p, links) in g.items():
        if math.hypot(p[0] + ox - X, p[2] + oz - Z) <= R:
            rows.append((p[0] + ox, p[2] + oz, p[1], k, [o for o in links]))
    for x, z, y, k, links in sorted(rows):
        print("  %-16s (%9.2f, %9.2f) y %6.2f  links %s" % (k, x, z, y, links))
