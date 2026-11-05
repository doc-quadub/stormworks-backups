"""Build the whole-map 'Real Track' mod (v3): every tile that has train tracks.

Steps (each resumable; run repeatedly until it prints 'done'):
  python3 build_world.py <rom_dir> <work_dir> scan      # find tiles, ballast recolour analysis
  python3 build_world.py <rom_dir> <work_dir> signs     # kilometre / picket posts (mainland)
  python3 build_world.py <rom_dir> <work_dir> catenary  # Plus edition only: overhead line plan
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

from swmesh import (read_mesh_full, write_mesh, write_phys, read_tracks, parse_transform,  # noqa: E402
                    read_vertex_array, VERTEX_COLOR_OFFSET, FIRST_VERTEX_OFFSET)
import numpy as np  # noqa: E402
import rails  # noqa: E402
import smooth  # noqa: E402
import signs  # noqa: E402
import catenary  # noqa: E402

MESH_DIR = "meshes/real_track"
LIB_DIR = MESH_DIR + "/lib"
BALLAST_DIR = MESH_DIR + "/ballast"
# dark "rail" strip colour -> ballast colour of the same terrain set
# Test: ballast prism (rails.add_ballast_prism) only inside this box, world x0, x1, z0, z1
# (internal tile-grid coordinates); None = nowhere. North Harbour station to ~2.3 km east.
BALLAST_PRISM_TEST_BOX = None
BALLAST_COLOR = (82, 80, 76)       # dark grey crushed stone
# Vanilla track bed colours -> (new colour, search radius): vertices of these colours closer
# than the radius to a track line are repainted. The radius keeps beaches and dirt roads of the
# same sandy colour untouched; the sandy shoulder reaches ~2.2 m, more between double tracks.
TRACK_BED_RECOLOR = {
    (45, 34, 32): (BALLAST_COLOR, 2.5),      # dark rail strips: mainland, Sawyer, arctic, modular
    (174, 150, 121): (BALLAST_COLOR, 4.0),   # sandy ballast and shoulder of the same set
    (41, 31, 29): (BALLAST_COLOR, 2.5),      # dark rail strips: arid island
    (188, 157, 125): (BALLAST_COLOR, 4.0),   # sandy ballast: arid island
}
WORKERS = 2                     # parallel tile builders (the PC's shell has 2 cores)
TIME_BUDGET = 140.0             # seconds per invocation (remote shell calls are time limited)
CHUNK_SIZE = 250.0              # spatial grid for splitting baked track geometry into meshes
MIN_COMPONENT_LENGTH = 50.0     # isolated bits of track graph shorter than this are not drawn
BORDER = 499.5
BORDER_NEAR = 497.0             # border pairs may lie this far inside / outside their tile edge
BORDER_PAIR_GAP = 1.0           # ... and this far apart (vanilla pairs that do not quite meet)
BORDER_SLIDE_MAX = 3.0          # a border end slides at most this far along its track onto the edge
SHALLOW_CROSSING_COS = 0.7     # track meets a tile edge at less than ~45 degrees

MOD_NAME = "Stormworks Railway Overhaul (SRO)"
MOD_VERSION = "1.0.3"
# with an overhead line plan (step "catenary") the build is the Plus edition
PLUS_NAME = "SRO Catenary+ (Sawyer)"
PLUS_VERSION = "1.0"
MOD_AUTHOR = "quadub"
MOD_XML = ('<?xml version="1.0" encoding="UTF-8"?>\n'
           '<mod name="{name}" author="{author}" '
           'desc="v{version}. 3D rails and concrete sleepers on every railway tile of the vanilla map, '
           'smoothed curves, dark grey ballast, kilometre and picket posts on the mainland. '
           'Track graph raised by {raise_cm} cm so trains run on the rail heads."/>\n')
PLUS_XML = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<mod name="{name}" author="{author}" '
            'desc="v{version}. Stormworks Railway Overhaul with a Soviet / Russian style 25 kV overhead line '
            '(visual) on every track of the Sawyer mainland: concrete poles with insulated cantilevers, '
            'zigzagged contact wire on droppers, anchor sections with overlaps, tension weights, portals '
            'with lattice girders and working ladders, wires through truss bridges. Includes the whole base SRO '
            '(3D rails and sleepers on every railway tile, smoothed curves, kilometre posts, track graph '
            'raised by {raise_cm} cm): use it instead of SRO, not together with it."/>\n')


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

    def near_bed(self, x, y, z, radius):
        """True if (x, y, z) lies within `radius` (horizontally) of a track line and roughly at
        track-bed height there - not on a beach or road under a bridge, not down in a dock."""
        cx, cz = int(x // self.cell), int(z // self.cell)
        for ix in (cx - 1, cx, cx + 1):
            for iz in (cz - 1, cz, cz + 1):
                for a, b in self.grid.get((ix, iz), ()):
                    dx, dz = b[0] - a[0], b[2] - a[2]
                    length2 = dx * dx + dz * dz
                    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - a[0]) * dx + (z - a[2]) * dz) / length2))
                    if math.hypot(x - a[0] - t * dx, z - a[2] - t * dz) > radius:
                        continue
                    track_y = a[1] + (b[1] - a[1]) * t
                    if BED_BELOW_TRACK <= y - track_y <= BED_ABOVE_TRACK:
                        return True
        return False


BED_BELOW_TRACK = -1.5     # track bed vertices lie at most this far below the track line ...
BED_ABOVE_TRACK = 0.3      # ... and at most this far above it

MODULAR_HEIGHT = 16.0
MODULAR_TRACK_OFFSET = 2.175     # straight joints: tracks at +-2.17..2.2 from the tile axis
MODULAR_CORNER = (498.5, 501.5)  # diagonal joints: just outside a tile corner


def modular_joint(p):
    """Nominal (position, outward direction) of a modular-line track end, or None."""
    x, y, z = p
    if abs(y - MODULAR_HEIGHT) > 0.08:
        return None
    ax, az = abs(x), abs(z)
    sx, sz = math.copysign(1.0, x), math.copysign(1.0, z)
    if abs(ax - 500.0) < 0.05 and 2.1 < az < 2.25:
        return (sx * 500.0, MODULAR_HEIGHT, sz * MODULAR_TRACK_OFFSET), (sx, 0.0)
    if abs(az - 500.0) < 0.05 and 2.1 < ax < 2.25:
        return (sx * MODULAR_TRACK_OFFSET, MODULAR_HEIGHT, sz * 500.0), (0.0, sz)
    for cx, cz in (MODULAR_CORNER, MODULAR_CORNER[::-1]):
        if abs(ax - cx) < 0.15 and abs(az - cz) < 0.15:
            h = math.sqrt(0.5)
            return (sx * cx, MODULAR_HEIGHT, sz * cz), (sx * h, sz * h)
    return None


def border_normal(p):
    half = rails.TILE_HALF_SIZE - 0.5
    if abs(p[0]) >= half - 2.0 and abs(p[2]) >= half - 2.0:
        # diagonal joint through a tile corner: cut across the 45 degree track
        h = math.sqrt(0.5)
        return (math.copysign(h, p[0]), 0.0, math.copysign(h, p[2]))
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

    def own_edge(p, near=BORDER):
        """(edge, coordinate along the edge, coordinate across it) of a node near a tile edge."""
        if max(abs(p[0]), abs(p[2])) < near:
            return None
        if abs(p[0]) >= abs(p[2]):               # near a corner: the edge it is nearest to
            return ("x+" if p[0] > 0 else "x-", p[2], p[0])
        return ("z+" if p[2] > 0 else "z-", p[0], p[2])

    def grid(t):
        m = re.fullmatch(r"(.+)_(\d+)_(\d+)", t)
        return None if m is None else (m.group(1), int(m.group(2)), int(m.group(3)))

    step = {"x+": (1, 0), "x-": (-1, 0), "z+": (0, 1), "z-": (0, -1)}

    def neighbours(t, edge, tt):
        """Is tile tt the neighbour of t across `edge` (for tiles of one grid)?"""
        a, b = grid(t), grid(tt)
        if a is None or b is None or a[0] != b[0]:
            return None                         # not one grid: unknown
        return (b[1] - a[1], b[2] - a[2]) == step[edge]

    def degree(nodes, k):
        return len([o for o in nodes[k][1] if o in nodes and o != k])

    opposite = {"x+": "x-", "x-": "x+", "z+": "z-", "z-": "z+"}
    by_edge = {}
    for t, nodes in nodes_by_tile.items():
        for k, (p, _) in nodes.items():
            e = own_edge(p, BORDER_NEAR)
            if e:
                by_edge.setdefault((e[0], round(e[1])), []).append((t, k, e[1], p[1], e[2]))

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
            e = own_edge(p, BORDER_NEAR)
            if e:
                for c in (round(e[1]) - 1, round(e[1]), round(e[1]) + 1):
                    for tt, kk, along, y, across in by_edge.get((opposite[e[0]], c), ()):
                        if abs(y - p[1]) >= 0.3:
                            continue
                        adjacent = neighbours(t, e[0], tt)
                        on_edge = abs(e[2]) >= BORDER and abs(across) >= BORDER
                        # the old rule (any tiles): both right on the edge, at the same point
                        strict = on_edge and abs(along - e[1]) < 0.15
                        loose = False
                        if not strict and adjacent:
                            # neighbouring grid tiles, two track ends that do not quite meet on the
                            # edge (a little apart, or both a little off it)
                            gap_across = e[2] - math.copysign(1000.0, e[2]) - across
                            loose = (math.hypot(along - e[1], gap_across) < BORDER_PAIR_GAP
                                     and degree(nodes, k) == 1 and degree(nodes_by_tile[tt], kk) == 1)
                        if strict or loose:
                            union((t, k), (tt, kk))
                            candidates.setdefault((t, k), {"strict": set(), "loose": set()})[
                                "strict" if strict else "loose"].add((tt, kk))

    # Unique border partners: (partner tile, partner node, partner raw y, partner inward direction,
    # partner raw x, partner raw z)
    partners = {}
    chosen = {}
    for key, found in candidates.items():
        best = found["strict"] or found["loose"]      # an exact pair always wins
        if len(best) == 1:
            chosen[key] = (next(iter(best)), bool(found["strict"]))
    for (t, k), (pair, exact) in chosen.items():
        found = {pair}
        if not exact and chosen.get(pair, (None,))[0] != (t, k):
            continue                                  # a loose pair must be mutual
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
    # *_instances.xml (tree placement) are large and never hold tracks: skip reading them
    texts = {}
    for f in sorted(os.listdir(tiles_dir)):
        if f.endswith(".xml") and not f.endswith("_instances.xml"):
            text = read_text(os.path.join(tiles_dir, f))
            if "<track " in text:
                texts[f[:-4]] = text
    tiles = sorted(texts)
    recolor = {}          # mesh file -> {vertex index: ballast colour}
    usage = {}            # mesh file -> tiles using it
    for tile in tiles:
        text = texts[tile]
        nodes = read_tracks(tile_xml_path(rom_dir, tile))
        grid = SegmentGrid(nodes)
        for file_name, matrix in mesh_entries(text):
            path = os.path.join(rom_dir, *file_name.split("/"))
            if not os.path.exists(path):
                continue
            _, verts = read_vertex_array(path)
            hits = recolor.setdefault(file_name, {})
            colors = np.stack([verts["r"], verts["g"], verts["b"]], axis=1)
            for old_color, (new_color, radius) in TRACK_BED_RECOLOR.items():
                for i in np.nonzero((colors == old_color).all(axis=1))[0]:
                    v = (float(verts["x"][i]), float(verts["y"][i]), float(verts["z"][i]))
                    x, y, z = transform_point(v, matrix)
                    if grid.near_bed(x, y, z, radius):
                        hits[int(i)] = new_color
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


# ---------------------------------------------------------------- step 1b: track posts

def step_signs(rom_dir, work):
    scan = json.load(open(os.path.join(work, "scan.json")))
    planned = signs.plan_signs(rom_dir, scan["tiles"], lambda tile: prepare_graph(rom_dir, tile, scan))
    with open(os.path.join(work, "signs.json"), "w") as f:
        json.dump(planned, f)
    print("done")


def step_catenary(rom_dir, work):
    """Plan of the overhead line (Plus edition). Without catenary.json the build is the plain
    Sawyer Railroads Overhaul."""
    scan = json.load(open(os.path.join(work, "scan.json")))
    plan = catenary.plan_catenary(rom_dir, scan["tiles"], lambda tile: prepare_graph(rom_dir, tile, scan))
    with open(os.path.join(work, "catenary.json"), "w") as f:
        json.dump(plan, f)
    print("done")


# ---------------------------------------------------------------- step 2: per-tile geometry

def prepare_graph(rom_dir, tile, scan):
    """Tile graph as it is drawn: border nodes joined with their partners, curves smoothed.
    Used by both the scan (kilometre posts) and the tiles step, so both see the same track.
    Returns (nodes, node_y, inserted ids, border tangents, stray ids)."""
    nodes = read_tracks(tile_xml_path(rom_dir, tile))
    node_y = {k: p[1] + rails.GRAPH_RAISE for k, (p, _) in nodes.items()}
    stray = set(scan.get("stray", {}).get(tile, ()))

    # Border nodes shared with a neighbour: same point, height and direction on both sides.
    # Vanilla pairs can be a few centimetres apart; both tiles move their node to the middle,
    # exactly on the tile edge.
    border_tangents = {}
    # Joints of the modular sea lines (track_* tiles and the tiles they plug into). One modular
    # file is used in many places with different neighbours, so pairs cannot be averaged; instead
    # every such joint is snapped to its exact nominal position, height and direction.
    for k, (p, links) in list(nodes.items()):
        if k in stray:
            continue
        if len([o for o in links if o in nodes and o != k]) != 1:
            continue
        joint = modular_joint(p)
        if joint is not None:
            position, direction = joint
            nodes[k] = (position, links)
            node_y[k] = position[1] + rails.GRAPH_RAISE
            border_tangents[k] = direction

    for k, (ptile, pnode, py, dx, dz, px, pz) in scan.get("partners", {}).get(tile, {}).items():
        if k in border_tangents:
            continue          # snapped modular joint
        p, links = nodes[k]
        inner = [o for o in links if o in nodes and o != k]
        if len(inner) != 1:
            continue
        # Onto the edge the node is nearest to. A pair may lie a little off the edge (or apart):
        # each end slides along its own track onto the edge (so a track meeting the edge at an
        # angle keeps its line), and the pair meets in the middle of the two slid points.
        q0 = nodes[inner[0]][0]
        x_edge = abs(abs(p[0]) - 500.0) <= abs(abs(p[2]) - 500.0)
        own_dir = (p[0] - q0[0], p[2] - q0[2])
        partner_dir = (-dx, -dz)                      # partner heading out of its tile

        def slide(x, z, d):
            across, along, d_across, d_along = (x, z, d[0], d[1]) if x_edge else (z, x, d[1], d[0])
            length = math.hypot(*d)
            if length < 1e-9 or abs(d_across) / length < 0.3:
                return along                          # along the edge: no sliding
            u = (math.copysign(500.0, across) - across) / d_across
            if abs(u) * length > BORDER_SLIDE_MAX:
                return along
            return along + d_along * u

        along = (slide(p[0], p[2], own_dir) + slide(px, pz, partner_dir)) / 2
        if x_edge:
            p = (math.copysign(500.0, p[0]), p[1], along)
        else:
            p = (along, p[1], math.copysign(500.0, p[2]))
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
    return nodes, node_y, inserted, border_tangents, stray


TILE_JOB = {}      # set once per worker process: rom_dir, scan, planned signs, output folders


def _init_tile_worker(rom_dir, work):
    rails.RAIL_PIECE_LENGTH = 1e9            # one rail piece per graph segment
    rails.CHUNK_SIZE = CHUNK_SIZE
    signs_path = os.path.join(work, "signs.json")
    catenary_path = os.path.join(work, "catenary.json")
    TILE_JOB.update(
        catenary_plan=json.load(open(catenary_path)) if os.path.exists(catenary_path) else None,
        rom_dir=rom_dir,
        scan=json.load(open(os.path.join(work, "scan.json"))),
        planned_signs=json.load(open(signs_path)) if os.path.exists(signs_path) else {},
        # Written outside the mod folder; assemble moves in only unique files (some prefab
        # tiles, e.g. the rotated track_* variants, produce byte-identical meshes).
        out_mesh_dir=os.path.join(work, "track_meshes"),
        state_dir=os.path.join(work, "tiles"))


def build_one_tile(tile):
    """Geometry, collision and state file of one tile. Returns a log line."""
    rom_dir, scan = TILE_JOB["rom_dir"], TILE_JOB["scan"]
    planned_signs = TILE_JOB["planned_signs"]
    out_mesh_dir = TILE_JOB["out_mesh_dir"]
    state_path = os.path.join(TILE_JOB["state_dir"], tile + ".json")
    nodes, node_y, inserted, border_tangents, stray = prepare_graph(rom_dir, tile, scan)
    drawn = {k: v for k, v in nodes.items() if k not in stray}
    border_cuts = {}
    for k, (p, links) in drawn.items():
        n = border_normal(p)
        if n is not None and len([o for o in links if o in drawn and o != k]) == 1:
            if k in border_tangents:
                t = border_tangents[k]
                t3 = (t[0], 0.0, t[1])
                if abs(rails.dot(n, t3)) < SHALLOW_CROSSING_COS:
                    # the track crosses the tile edge at a shallow angle: an edge-plane cut
                    # would be a long sliver, so both tiles cut square to the shared direction
                    border_cuts[k] = (t3, t3)
                else:
                    border_cuts[k] = (n, t3)
            else:
                border_cuts[k] = n
    ballast_keep = None
    origin = signs.mainland_origin(tile)
    if BALLAST_PRISM_TEST_BOX is not None and origin is not None:
        x0, x1, z0, z1 = BALLAST_PRISM_TEST_BOX

        def ballast_keep(p, ox=origin[0], oz=origin[1]):
            return x0 <= p[0] + ox <= x1 and z0 <= p[2] + oz <= z1
    builder, sleeper_count, chains = rails.build_tile_v4(drawn, node_y, border_cuts,
                                                         set(border_cuts), ballast_keep)
    tile_signs = planned_signs.get(tile, [])
    for sign in tile_signs:
        signs.add_sign(builder, sign)
    if TILE_JOB["catenary_plan"] and signs.mainland_origin(tile) is not None:
        catenary.add_catenary(builder, TILE_JOB["catenary_plan"], tile, signs.mainland_origin(tile))
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
    # written to a temporary name and renamed: a run stopped half-way never leaves a broken state
    with open(state_path + ".tmp", "w") as f:
        json.dump({"node_y": node_y, "files": files, "phys_files": phys_files,
                   "placements": placements, "vertices": vertices,
                   "collision_triangles": len(builder.collision),
                   "graph": {k: [p[0], p[1], p[2], links] for k, (p, links) in nodes.items()}}, f)
    os.replace(state_path + ".tmp", state_path)
    return ("%-45s chains %3d  added nodes %4d  sleepers %6d  posts %3d  vertices %8d  meshes %3d"
            % (tile, len(chains), len(inserted), sleeper_count, len(tile_signs), vertices, len(files)))


def step_tiles(rom_dir, work):
    """Tiles are independent: built in parallel worker processes (one per CPU core), until the
    time budget of this invocation runs out; run again to continue."""
    import multiprocessing
    os.makedirs(os.path.join(work, "track_meshes"), exist_ok=True)
    os.makedirs(os.path.join(work, "tiles"), exist_ok=True)
    scan = json.load(open(os.path.join(work, "scan.json")))
    pending = [t for t in scan["tiles"]
               if not os.path.exists(os.path.join(work, "tiles", t + ".json"))]
    start = time.time()
    finished = 0
    workers = max(1, min(WORKERS, os.cpu_count() or 1))
    with multiprocessing.Pool(workers, _init_tile_worker, (rom_dir, work)) as pool:
        for line in pool.imap_unordered(build_one_tile, pending):
            print(line)
            finished += 1
            if time.time() - start > TIME_BUDGET:
                break
        pool.terminate()
    remaining = len(pending) - finished
    print("remaining", remaining)
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
        data = bytearray(open(os.path.join(rom_dir, *file_name.split("/")), "rb").read())
        for i, r, g, b in indices:
            at = FIRST_VERTEX_OFFSET + 28 * i + VERTEX_COLOR_OFFSET
            data[at:at + 3] = bytes((r, g, b))
        with open(os.path.join(mod, *ballast_copy_name(file_name).split("/")), "wb") as f:
            f.write(data)

    # game ladder elements for the portal ladders (Plus edition), by the tile they stand in
    ladders = {}
    catenary_path = os.path.join(work, "catenary.json")
    if os.path.exists(catenary_path):
        for foot, x_axis, z_axis, height in catenary.portal_ladders(json.load(open(catenary_path))):
            tile = "mega_island_%d_%d" % (int(math.floor((foot[0] + 8500) / 1000)), int(math.floor((foot[2] + 12500) / 1000)))
            if tile in scan["tiles"]:
                ladders.setdefault(tile, []).append((foot, x_axis, z_axis, height))

    # tile XMLs
    tiles_out = os.path.join(mod, "data", "tiles")
    os.makedirs(tiles_out, exist_ok=True)
    total_ladders = 0
    total_entries = 0
    total_phys = 0
    total_vertices = 0
    for tile in scan["tiles"]:
        state = json.load(open(os.path.join(work, "tiles", tile + ".json")))
        text = read_text(tile_xml_path(rom_dir, tile))
        newline = "\r\n" if "\r\n" in text else "\n"

        graph = {k: ((v[0], v[1], v[2]), v[3]) for k, v in state["graph"].items()}
        original_ids = set(re.findall(r'<track id="([^"]+)"', text))
        text = smooth.train_tracks_xml(text, graph, state["node_y"], newline, rails.GRAPH_RAISE)
        written_ids = re.findall(r'<track id="([^"]+)"', text)
        assert len(written_ids) == len(set(written_ids)) and original_ids <= set(written_ids), tile
        for links in re.findall(r'<links>(.*?)</links>', text, re.S):
            assert set(re.findall(r'<link id="([^"]+)"', links)) <= set(written_ids), tile

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
        ladder_entries = []
        if tile in ladders:
            ox, oz = signs.mainland_origin(tile)
            for i, (foot, x_axis, z_axis, height) in enumerate(ladders[tile]):
                ladder_entries.append(
                    '\t\t<ladder id="real_track_ladder_%d" height="%.3f">%s'
                    '\t\t\t<transform 00="%.6f" 02="%.6f" 20="%.6f" 22="%.6f" 30="%.6f" 31="%.6f" 32="%.6f"/>%s'
                    '\t\t</ladder>%s'
                    % (i, height, newline, x_axis[0], x_axis[2], z_axis[0], z_axis[2],
                       foot[0] - ox, foot[1], foot[2] - oz, newline, newline))
            text = append_to_section(text, "ladders", "".join(ladder_entries), newline)
        total_ladders += len(ladder_entries)
        with open(os.path.join(tiles_out, tile + ".xml"), "w", encoding="utf-8", newline="") as f:
            f.write(text)
        total_entries += len(entries)
        total_vertices += state["vertices"]

    with open(os.path.join(mod, "mod.xml"), "w", encoding="utf-8") as f:
        plus = os.path.exists(os.path.join(work, "catenary.json"))
        f.write((PLUS_XML if plus else MOD_XML).format(name=PLUS_NAME if plus else MOD_NAME, author=MOD_AUTHOR,
                               version=PLUS_VERSION if plus else MOD_VERSION,
                               raise_cm=int(round(rails.GRAPH_RAISE * 100))))
    print("tiles", len(scan["tiles"]), " mesh entries added", total_entries,
          " track vertices", total_vertices, " physics entries added", total_phys,
          " shared duplicate files", duplicates, " ladders", total_ladders)
    print("done")


if __name__ == "__main__":
    rom_dir, work, step = sys.argv[1:4]
    os.makedirs(work, exist_ok=True)
    {"scan": step_scan, "signs": step_signs, "catenary": step_catenary, "tiles": step_tiles,
     "assemble": step_assemble}[step](rom_dir, work)
