"""Build the whole-map 'Real Track' mod (v3): every tile that has train tracks.

Steps (each resumable; run repeatedly until it prints 'done'):
  python3 build_world.py <rom_dir> <work_dir> scan      # find tiles, ballast recolour analysis
  python3 build_world.py <rom_dir> <work_dir> tiles     # rails/bearers meshes + sleeper placements
  python3 build_world.py <rom_dir> <work_dir> assemble  # unique track meshes, ballast meshes, tile XMLs

The mod ends up in <work_dir>/mod.
"""

import hashlib
import json
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from swmesh import read_mesh_full, write_mesh, write_phys, read_tracks, parse_transform  # noqa: E402
import rails  # noqa: E402
import smooth  # noqa: E402

MESH_DIR = "meshes/real_track"
LIB_DIR = MESH_DIR + "/lib"
BALLAST_DIR = MESH_DIR + "/ballast"
# dark "rail" strip colour -> ballast colour of the same terrain set
STRIP_TO_BALLAST = {
    (45, 34, 32): (174, 150, 121),     # mainland (mega_island), Sawyer, arctic, track pieces
    (41, 31, 29): (188, 157, 125),     # arid island
}
STRIP_SEARCH_RADIUS = 2.5       # strip-coloured vertices this close to a track line get recoloured
TIME_BUDGET = 140.0             # seconds per invocation (remote shell calls are time limited)
CHUNK_SIZE = 250.0              # spatial grid for splitting baked track geometry into meshes
MIN_COMPONENT_LENGTH = 50.0     # isolated bits of track graph shorter than this are not drawn
BORDER = 499.5

MOD_XML = ('<?xml version="1.0" encoding="UTF-8"?>\n'
           '<mod name="Real Track" author="Real Track" '
           'desc="3D rails and concrete sleepers on every railway tile of the vanilla map. '
           'Track graph raised by {raise_cm} cm so trains run on the rail heads."/>\n')


# ---------------------------------------------------------------- helpers

def tile_xml_path(rom_dir, tile):
    return os.path.join(rom_dir, "data", "tiles", tile + ".xml")


def read_text(path):
    with open(path, encoding="utf-8", newline="") as f:
        return f.read()


def mesh_entries(xml_text):
    """[(file_name, 4x4 matrix)] for every <mesh> element of a tile."""
    out = []
    for m in re.finditer(r'<mesh id="[^"]*"(?: parent_id="[^"]*")? file_name="([^"]+)">\s*'
                         r'<transform([^/]*)/>', xml_text):
        out.append((m.group(1), parse_transform(m.group(2))))
    return out


def transform_point(p, m):
    return (p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0] + m[3][0],
            p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1] + m[3][1],
            p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] + m[3][2])


class SegmentGrid:
    """Fast 'is this point near a track line' queries."""

    def __init__(self, nodes, cell=10.0):
        self.cell = cell
        self.grid = {}
        seen = set()
        for k, (p, links) in nodes.items():
            for o in links:
                if o not in nodes or o == k or frozenset((k, o)) in seen:
                    continue
                seen.add(frozenset((k, o)))
                a, b = p, nodes[o][0]
                steps = int(math.dist(a, b) // (cell / 2)) + 1
                for i in range(steps + 1):
                    f = i / steps
                    x = a[0] + (b[0] - a[0]) * f
                    z = a[2] + (b[2] - a[2]) * f
                    self.grid.setdefault((int(x // cell), int(z // cell)), set()).add((a, b))

    def distance(self, x, z):
        best = math.inf
        cx, cz = int(x // self.cell), int(z // self.cell)
        for ix in (cx - 1, cx, cx + 1):
            for iz in (cz - 1, cz, cz + 1):
                for a, b in self.grid.get((ix, iz), ()):
                    dx, dz = b[0] - a[0], b[2] - a[2]
                    length2 = dx * dx + dz * dz
                    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - a[0]) * dx + (z - a[2]) * dz) / length2))
                    best = min(best, math.hypot(x - a[0] - t * dx, z - a[2] - t * dz))
        return best


def border_normal(p):
    half = rails.TILE_HALF_SIZE - 0.5
    if abs(p[0]) >= half:
        return (math.copysign(1.0, p[0]), 0.0, 0.0)
    if abs(p[2]) >= half:
        return (0.0, 0.0, math.copysign(1.0, p[2]))
    return None


def find_stray_components(rom_dir, tiles):
    """Find pieces of track graph not connected to anything, shorter than MIN_COMPONENT_LENGTH.

    Tiles are linked through border nodes: a node on edge +x at (z, y) of one tile continues a
    node on edge -x at the same (z, y) of a neighbouring tile (world positions are not needed).
    Returns {tile: [node ids]} of nodes that should not get any geometry.
    """
    nodes_by_tile = {t: read_tracks(tile_xml_path(rom_dir, t)) for t in tiles}

    def own_edge(p):
        if p[0] >= BORDER:
            return ("x+", p[2])
        if p[0] <= -BORDER:
            return ("x-", p[2])
        if p[2] >= BORDER:
            return ("z+", p[0])
        if p[2] <= -BORDER:
            return ("z-", p[0])
        return None

    opposite = {"x+": "x-", "x-": "x+", "z+": "z-", "z-": "z+"}
    by_edge = {}
    for t, nodes in nodes_by_tile.items():
        for k, (p, _) in nodes.items():
            e = own_edge(p)
            if e:
                by_edge.setdefault((e[0], round(e[1])), []).append((t, k, e[1], p[1]))

    parent = {}

    def find(a):
        parent.setdefault(a, a)
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        parent[find(a)] = find(b)

    candidates = {}
    for t, nodes in nodes_by_tile.items():
        for k, (p, links) in nodes.items():
            find((t, k))
            for o in links:
                if o in nodes and o != k:
                    union((t, k), (t, o))
            e = own_edge(p)
            if e:
                for c in (round(e[1]) - 1, round(e[1]), round(e[1]) + 1):
                    for tt, kk, along, y in by_edge.get((opposite[e[0]], c), ()):
                        if abs(along - e[1]) < 0.15 and abs(y - p[1]) < 0.3:
                            union((t, k), (tt, kk))
                            candidates.setdefault((t, k), set()).add((tt, kk))

    # Unique border partners: (partner tile, partner node, partner raw y, partner inward direction,
    # partner raw x, partner raw z)
    partners = {}
    for (t, k), found in candidates.items():
        if len(found) != 1:
            continue
        tt, kk = next(iter(found))
        pp, plinks = nodes_by_tile[tt][kk]
        inner = [o for o in plinks if o in nodes_by_tile[tt] and o != kk]
        if len(inner) != 1:
            continue
        q = nodes_by_tile[tt][inner[0]][0]
        length = math.hypot(q[0] - pp[0], q[2] - pp[2])
        if length < 1e-6:
            continue
        partners.setdefault(t, {})[k] = [tt, kk, pp[1], (q[0] - pp[0]) / length, (q[2] - pp[2]) / length,
                                         pp[0], pp[2]]

    members = {}
    for t, nodes in nodes_by_tile.items():
        for k in nodes:
            members.setdefault(find((t, k)), []).append((t, k))
    stray = {}
    for group in members.values():
        group_set = set(group)
        length = 0.0
        for t, k in group:
            p, links = nodes_by_tile[t][k]
            for o in links:
                if (t, o) in group_set and o != k:
                    length += math.dist(p, nodes_by_tile[t][o][0]) / 2
        if length < MIN_COMPONENT_LENGTH:
            for t, k in group:
                stray.setdefault(t, []).append(k)
    return stray, partners


# ---------------------------------------------------------------- step 1: scan

def step_scan(rom_dir, work):
    tiles_dir = os.path.join(rom_dir, "data", "tiles")
    tiles = sorted(f[:-4] for f in os.listdir(tiles_dir)
                   if f.endswith(".xml") and "<track " in read_text(os.path.join(tiles_dir, f)))
    recolor = {}          # mesh file -> {vertex index: ballast colour}
    usage = {}            # mesh file -> tiles using it
    for tile in tiles:
        text = read_text(tile_xml_path(rom_dir, tile))
        nodes = read_tracks(tile_xml_path(rom_dir, tile))
        grid = SegmentGrid(nodes)
        for file_name, matrix in mesh_entries(text):
            path = os.path.join(rom_dir, *file_name.split("/"))
            if not os.path.exists(path):
                continue
            mesh = read_mesh_full(path)
            hits = recolor.setdefault(file_name, {})
            for i, v in enumerate(mesh.vertices):
                new_color = STRIP_TO_BALLAST.get(tuple(v[3:6]))
                if new_color is None:
                    continue
                x, _, z = transform_point(v, matrix)
                if grid.distance(x, z) <= STRIP_SEARCH_RADIUS:
                    hits[i] = new_color
            usage.setdefault(file_name, set()).add(tile)
    recolor = {k: sorted([i, *c] for i, c in v.items()) for k, v in recolor.items() if v}
    stray, partners = find_stray_components(rom_dir, tiles)
    with open(os.path.join(work, "scan.json"), "w") as f:
        json.dump({"tiles": tiles, "recolor": recolor, "stray": stray, "partners": partners,
                   "usage": {k: sorted(v) for k, v in usage.items() if k in recolor}}, f)
    print("tiles with tracks:", len(tiles), " meshes to recolour:", len(recolor),
          " vertices:", sum(len(v) for v in recolor.values()),
          " stray nodes skipped:", sum(len(v) for v in stray.values()),
          " border nodes with a partner:", sum(len(v) for v in partners.values()))
    print("done")


# ---------------------------------------------------------------- step 2: per-tile geometry

def step_tiles(rom_dir, work):
    rails.RAIL_PIECE_LENGTH = 1e9            # one rail piece per graph segment
    rails.CHUNK_SIZE = CHUNK_SIZE
    scan = json.load(open(os.path.join(work, "scan.json")))
    # Written outside the mod folder; assemble moves in only unique files (some prefab tiles,
    # e.g. the rotated track_* variants, produce byte-identical meshes).
    out_mesh_dir = os.path.join(work, "track_meshes")
    os.makedirs(out_mesh_dir, exist_ok=True)
    state_dir = os.path.join(work, "tiles")
    os.makedirs(state_dir, exist_ok=True)
    start = time.time()
    remaining = 0
    for tile in scan["tiles"]:
        state_path = os.path.join(state_dir, tile + ".json")
        if os.path.exists(state_path):
            continue
        if time.time() - start > TIME_BUDGET:
            remaining += 1
            continue
        nodes = read_tracks(tile_xml_path(rom_dir, tile))
        node_y = {k: p[1] + rails.GRAPH_RAISE for k, (p, _) in nodes.items()}
        stray = set(scan.get("stray", {}).get(tile, ()))

        # Border nodes shared with a neighbour: same point, height and direction on both sides.
        # Vanilla pairs can be a few centimetres apart; both tiles move their node to the middle,
        # exactly on the tile edge.
        border_tangents = {}
        for k, (ptile, pnode, py, dx, dz, px, pz) in scan.get("partners", {}).get(tile, {}).items():
            p, links = nodes[k]
            inner = [o for o in links if o in nodes and o != k]
            if len(inner) != 1:
                continue
            if abs(p[0]) >= BORDER:
                p = (math.copysign(500.0, p[0]), p[1], (p[2] + pz) / 2)
            else:
                p = ((p[0] + px) / 2, p[1], math.copysign(500.0, p[2]))
            nodes[k] = (p, links)
            node_y[k] = (p[1] + py) / 2 + rails.GRAPH_RAISE
            q = nodes[inner[0]][0]
            ox, oz = p[0] - q[0], p[2] - q[2]          # own direction, heading out of the tile
            olen = math.hypot(ox, oz)
            tx, tz = ox / olen + dx, oz / olen + dz     # + partner direction, heading on
            tlen = math.hypot(tx, tz)
            if tlen > 1e-6:
                border_tangents[k] = (tx / tlen, tz / tlen)

        nodes, node_y, inserted = smooth.smooth_graph(nodes, node_y, border_tangents, stray)
        drawn = {k: v for k, v in nodes.items() if k not in stray}
        border_cuts = {}
        for k, (p, links) in drawn.items():
            n = border_normal(p)
            if n is not None and len([o for o in links if o in drawn and o != k]) == 1:
                if k in border_tangents:
                    t = border_tangents[k]
                    border_cuts[k] = (n, (t[0], 0.0, t[1]))
                else:
                    border_cuts[k] = n
        builder, sleeper_count, chains = rails.build_tile_v4(drawn, node_y, border_cuts,
                                                             set(border_cuts))
        placements = []
        meshes = rails.polys_to_meshes(builder.polys)
        files = []
        vertices = 0
        for chunk_name, mesh in meshes.items():
            safe = chunk_name.replace("+", "p").replace("-", "m")
            file_name = "%s_%s.mesh" % (tile, safe)
            write_mesh(os.path.join(out_mesh_dir, file_name), mesh)
            files.append((safe, file_name))
            vertices += len(mesh.vertices)
        phys_files = []
        for chunk_name, triangles in rails.collision_to_phys_chunks(builder.collision).items():
            safe = chunk_name.replace("+", "p").replace("-", "m")
            file_name = "%s_%s.phys" % (tile, safe)
            write_phys(os.path.join(out_mesh_dir, file_name), triangles)
            phys_files.append((safe, file_name))
        with open(state_path, "w") as f:
            json.dump({"node_y": node_y, "files": files, "phys_files": phys_files,
                       "placements": placements, "vertices": vertices,
                       "collision_triangles": len(builder.collision),
                       "graph": {k: [p[0], p[1], p[2], links] for k, (p, links) in nodes.items()}}, f)
        print("%-45s chains %3d  added nodes %4d  sleepers %6d  vertices %8d  meshes %3d"
              % (tile, len(chains), len(inserted), sleeper_count, vertices, len(files)))
    print("remaining", remaining if remaining else 0)
    if not remaining:
        print("done")


# ---------------------------------------------------------------- step 3: assemble the mod

def ballast_copy_name(file_name):
    return BALLAST_DIR + "/" + file_name[len("meshes/"):].replace("/", "__")


def format_matrix(m):
    keys = []
    for r in range(4):
        for c in range(3):
            keys.append('%d%d="%.6f"' % (r, c, m[r][c]))
    return " ".join(keys)


def append_to_section(text, section, block, newline):
    """Insert entries at the end of <section>...</section> (or fill an empty <section/>)."""
    if not block:
        return text
    closing = "</%s>" % section
    if closing in text:
        idx = text.index(closing)
        line_start = text.rfind("\n", 0, idx) + 1
        return text[:line_start] + block + text[line_start:]
    empty = "<%s/>" % section
    assert empty in text, "no <%s> section" % section
    return text.replace(empty, "<%s>" % section + newline + block + "\t" + closing, 1)


def step_assemble(rom_dir, work):
    scan = json.load(open(os.path.join(work, "scan.json")))
    mod = os.path.join(work, "mod")

    # track meshes: identical files are stored once and shared by the tiles that need them
    raw_dir = os.path.join(work, "track_meshes")
    mesh_dir = os.path.join(mod, *MESH_DIR.split("/"))
    os.makedirs(mesh_dir, exist_ok=True)
    stored_by_hash = {}
    stored_name = {}
    duplicates = 0
    for tile in scan["tiles"]:
        state = json.load(open(os.path.join(work, "tiles", tile + ".json")))
        for _, file_name in state["files"] + state["phys_files"]:
            source = os.path.join(raw_dir, file_name)
            if not os.path.exists(source):            # already moved by an earlier assemble run
                stored_name[file_name] = file_name
                continue
            with open(source, "rb") as f:
                digest = hashlib.md5(f.read()).hexdigest()
            if digest in stored_by_hash:
                stored_name[file_name] = stored_by_hash[digest]
                duplicates += 1
                continue
            stored_by_hash[digest] = file_name
            stored_name[file_name] = file_name
            os.replace(source, os.path.join(mesh_dir, file_name))

    # ballast copies (dark rail strips painted as ballast, only next to track lines)
    ballast = os.path.join(mod, *BALLAST_DIR.split("/"))
    os.makedirs(ballast, exist_ok=True)
    for file_name, indices in scan["recolor"].items():
        mesh = read_mesh_full(os.path.join(rom_dir, *file_name.split("/")))
        for i, r, g, b in indices:
            v = mesh.vertices[i]
            mesh.vertices[i] = v[:3] + (r, g, b) + v[6:]
        write_mesh(os.path.join(mod, *ballast_copy_name(file_name).split("/")), mesh)

    # tile XMLs
    tiles_out = os.path.join(mod, "data", "tiles")
    os.makedirs(tiles_out, exist_ok=True)
    total_entries = 0
    total_phys = 0
    total_vertices = 0
    for tile in scan["tiles"]:
        state = json.load(open(os.path.join(work, "tiles", tile + ".json")))
        text = read_text(tile_xml_path(rom_dir, tile))
        newline = "\r\n" if "\r\n" in text else "\n"

        graph = {k: ((v[0], v[1], v[2]), v[3]) for k, v in state["graph"].items()}
        original_count = len(re.findall(r'<track id="', text))
        text = smooth.train_tracks_xml(text, graph, state["node_y"], newline)
        assert len(re.findall(r'<track id="', text)) == len(graph) >= original_count, tile

        for file_name in scan["recolor"]:
            text = text.replace('file_name="%s"' % file_name,
                                'file_name="%s"' % ballast_copy_name(file_name))

        entries = []
        for safe, file_name in state["files"]:
            entries.append('\t\t<mesh id="real_track_%s" file_name="%s/%s">%s\t\t\t<transform/>%s\t\t</mesh>%s'
                           % (safe, MESH_DIR, stored_name[file_name], newline, newline, newline))
        text = append_to_section(text, "meshes", "".join(entries), newline)
        phys_entries = []
        for safe, file_name in state["phys_files"]:
            phys_entries.append('\t\t<physics_mesh id="real_track_phys_%s" file_name="%s/%s">%s'
                                '\t\t\t<transform/>%s\t\t</physics_mesh>%s'
                                % (safe, MESH_DIR, stored_name[file_name], newline, newline, newline))
        text = append_to_section(text, "physics_meshes", "".join(phys_entries), newline)
        total_phys += len(phys_entries)
        with open(os.path.join(tiles_out, tile + ".xml"), "w", encoding="utf-8", newline="") as f:
            f.write(text)
        total_entries += len(entries)
        total_vertices += state["vertices"]

    with open(os.path.join(mod, "mod.xml"), "w", encoding="utf-8") as f:
        f.write(MOD_XML.format(raise_cm=int(round(rails.GRAPH_RAISE * 100))))
    print("tiles", len(scan["tiles"]), " mesh entries added", total_entries,
          " track vertices", total_vertices, " physics entries added", total_phys,
          " shared duplicate files", duplicates)
    print("done")


if __name__ == "__main__":
    rom_dir, work, step = sys.argv[1:4]
    os.makedirs(work, exist_ok=True)
    {"scan": step_scan, "tiles": step_tiles, "assemble": step_assemble}[step](rom_dir, work)
