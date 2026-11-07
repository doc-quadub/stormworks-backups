"""Build the 'Real Track' test mod for a group of adjacent mainland tiles.

North Harbour (the start base) is mega_island_9_8, NOT island_33_tile_32.
Mainland tile mega_island_X_Y has its centre at world (X*1000 - 8000, Y*1000 - 12000).

Usage: python3 build_mod.py <rom_dir> <out_mod_dir> [tile_name ...]
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from swmesh import read_mesh_full, write_mesh, read_tracks  # noqa: E402
import rails  # noqa: E402

DEFAULT_TILES = [
    "mega_island_9_8",    # North Harbour
    "mega_island_9_7",    # south neighbour
    "mega_island_8_8",    # west neighbour
    "mega_island_10_8",   # east neighbour
    "mega_island_11_8",   # station east of 10_8
]
MESH_DIR = "meshes/real_track"
VANILLA_TRACK_STRIP_COLOR = (45, 34, 32)
BALLAST_COLOR = (174, 150, 121)

MOD_XML = ('<?xml version="1.0" encoding="UTF-8"?>\n'
           '<mod name="Real Track (test: North Harbour area)" author="Real Track" '
           'desc="3D rails and concrete sleepers on vanilla tiles: {tiles}. '
           'Track graph raised by {raise_cm} cm so trains run on the rail heads."/>\n')


def tile_origin(tile):
    m = re.fullmatch(r"mega_island_(\d+)_(\d+)", tile)
    assert m, "only mainland mega_island_X_Y tiles are supported: %s" % tile
    return (int(m.group(1)) * 1000 - 8000, int(m.group(2)) * 1000 - 12000)


def terrain_mesh_path(tile):
    return "meshes/mainland/%s.mesh" % tile


def set_transform_y(transform_attrs, new_y):
    """Replace or add the 31 (translation y) attribute inside a <transform .../> attribute string."""
    value = "%.6f" % new_y
    if re.search(r'\b31="[^"]*"', transform_attrs):
        return re.sub(r'\b31="[^"]*"', '31="%s"' % value, transform_attrs)
    return transform_attrs.rstrip() + ' 31="%s"' % value


def patch_tile_xml(text, tile, node_y, track_mesh_names, newline):
    # 1) lift the train track graph
    def repl(m):
        node_id, ws, attrs = m.group(1), m.group(2), m.group(3)
        return '<track id="%s">%s<transform%s/>' % (node_id, ws, set_transform_y(attrs, node_y[node_id]))

    text, n = re.subn(r'<track id="([^"]+)">(\s*)<transform([^/]*)/>', repl, text)
    assert n == len(node_y), "lifted %d of %d nodes" % (n, len(node_y))

    # 2) point the terrain mesh at the copy without the dark rail strips
    old = 'file_name="%s"' % terrain_mesh_path(tile)
    assert text.count(old) == 1, "terrain mesh entry not found exactly once in %s" % tile
    text = text.replace(old, 'file_name="%s/%s_ballast.mesh"' % (MESH_DIR, tile))

    # 3) add the generated track meshes at the end of <meshes>
    entries = "".join(
        '\t\t<mesh id="real_track_%s" file_name="%s/%s">%s\t\t\t<transform/>%s\t\t</mesh>%s'
        % (name, MESH_DIR, file_name, newline, newline, newline)
        for name, file_name in track_mesh_names)
    idx = text.index("</meshes>")
    line_start = text.rfind("\n", 0, idx) + 1
    return text[:line_start] + entries + text[line_start:]


def main(rom_dir, out_dir, tiles=None):
    tiles = tiles or DEFAULT_TILES
    xml_paths = {t: os.path.join(rom_dir, "data", "tiles", t + ".xml") for t in tiles}
    all_nodes = {t: read_tracks(xml_paths[t]) for t in tiles}
    node_y, continuing, border_ends = rails.lift_graph_multi(
        all_nodes, {t: tile_origin(t) for t in tiles})

    mesh_out = os.path.join(out_dir, *MESH_DIR.split("/"))
    tiles_out = os.path.join(out_dir, "data", "tiles")
    os.makedirs(mesh_out, exist_ok=True)
    os.makedirs(tiles_out, exist_ok=True)

    for t in tiles:
        builder, _, chains = rails.build_track_geometry(all_nodes[t], node_y[t], continuing[t],
                                                        border_ends[t])
        meshes = rails.polys_to_meshes(builder.polys)
        names = []
        vertices = 0
        for chunk_name, mesh in meshes.items():
            safe = chunk_name.replace("+", "p").replace("-", "m")
            file_name = "%s_track_%s.mesh" % (t, safe)
            write_mesh(os.path.join(mesh_out, file_name), mesh)
            names.append((safe, file_name))
            vertices += len(mesh.vertices)

        terrain = read_mesh_full(os.path.join(rom_dir, *terrain_mesh_path(t).split("/")))
        recolored = 0
        for i, v in enumerate(terrain.vertices):
            if tuple(v[3:6]) == VANILLA_TRACK_STRIP_COLOR:
                terrain.vertices[i] = v[:3] + BALLAST_COLOR + v[6:]
                recolored += 1
        write_mesh(os.path.join(mesh_out, t + "_ballast.mesh"), terrain)

        with open(xml_paths[t], encoding="utf-8", newline="") as f:
            text = f.read()
        newline = "\r\n" if "\r\n" in text else "\n"
        with open(os.path.join(tiles_out, t + ".xml"), "w", encoding="utf-8", newline="") as f:
            f.write(patch_tile_xml(text, t, node_y[t], names, newline))

        lifts = [node_y[t][k] - all_nodes[t][k][0][1] for k in all_nodes[t]]
        print("%-18s chains %3d  meshes %2d  vertices %7d  strip vertices recolored %5d  "
              "lift %.2f..%.2f  joined border nodes %d"
              % (t, len(chains), len(names), vertices, recolored, min(lifts), max(lifts),
                 len(continuing[t])))

    with open(os.path.join(out_dir, "mod.xml"), "w", encoding="utf-8") as f:
        f.write(MOD_XML.format(tiles=", ".join(tiles), raise_cm=int(round(rails.GRAPH_RAISE * 100))))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3:] or None)
