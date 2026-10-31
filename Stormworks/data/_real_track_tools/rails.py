"""Generate 3D rail + sleeper track meshes along a Stormworks tile's train_tracks graph.

Coordinates are tile-local: x/z horizontal, y up. All dimensions in metres.
"""

import heapq
import math
from dataclasses import dataclass, field

from swmesh import Mesh, Submesh

# ---------------------------------------------------------------- parameters

RAIL_HALF_GAUGE = 0.85        # lateral offset of each rail axis from the track centreline
GRAPH_RAISE = 0.28            # how much the train_tracks graph is lifted above the vanilla nodes
RAIL_TOP_OFFSET = -0.03       # rail top relative to the (lifted) graph; tuned in game 2026-10-30:
                              # with 0.0 the wheels sank ~3 cm into the rail head
BORDER_RAMP_LENGTH = 40.0     # path length over which the lift fades to 0 at tile borders
TILE_HALF_SIZE = 500.0

SLEEPER_SPACING = 0.60
SLEEPER_HALF_LENGTH = 1.35     # concrete sleeper, 2.7 m
SLEEPER_HALF_WIDTH = 0.14      # half width at the bottom (used for overlap tests)
SLEEPER_TOP_HALF_WIDTH_LOSS = 0.03   # sides taper inwards by this much over the full height
SLEEPER_HEIGHT = 0.19          # height under the rail seat
# Concrete sleeper top profile, one half (u >= 0), heights above the sleeper bottom.
# v4 (whole map): a plain prism with a flat top - cheap enough to bake every sleeper into the
# tile meshes (the profiled v2 shape is kept below for reference).
SLEEPER_PROFILE_HALF = [
    (0.00, 0.190),
    (1.35, 0.190),
]
SLEEPER_PROFILE_HALF_PROFILED_V2 = [
    (0.00, 0.145),
    (0.40, 0.145),
    (0.58, 0.190),
    (1.10, 0.190),
    (1.35, 0.150),
]
SLEEPER_MERGE_MAX_ANGLE = math.radians(12.0)
SLEEPER_EXTENSION_MARGIN = 0.35
SLEEPER_MAX_HALF_SPAN = 4.5

COLOR_RAIL_SIDE = (62, 56, 52, 255)
COLOR_RAIL_TOP = (150, 148, 145, 255)
COLOR_SLEEPER = (138, 136, 130, 255)
COLOR_SLEEPER_TOP = (158, 156, 150, 255)

# Rail cross-section: (lateral u, height h), rail top at h = 0, listed left -> over the top -> right.
# The bottom edge is omitted (it rests on the sleepers and is never visible).
RAIL_PROFILE = [
    (-0.075, -0.172),
    (-0.075, -0.157),
    (-0.009, -0.140),
    (-0.009, -0.060),
    (-0.036, -0.048),
    (-0.036, -0.008),
    (-0.030, 0.000),
    (0.030, 0.000),
    (0.036, -0.008),
    (0.036, -0.048),
    (0.009, -0.060),
    (0.009, -0.140),
    (0.075, -0.157),
    (0.075, -0.172),
]
RAIL_HEIGHT = 0.172
RAIL_TOP_EDGE_INDEX = 6       # profile edge 6->7 is the running surface
RAIL_PIECE_LENGTH = 4.0       # long rail quads are split into pieces of at most this length
                              # (v3 whole-map build sets it to 1e9: one piece per segment)

MAX_VERTICES_PER_MESH = 60000
CHUNK_SIZE = 125.0


# ---------------------------------------------------------------- small vector helpers

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    length = math.sqrt(dot(a, a))
    return (a[0] / length, a[1] / length, a[2] / length) if length > 1e-12 else (0.0, 0.0, 0.0)


def horizontal_dir(a, b):
    return norm((b[0] - a[0], 0.0, b[2] - a[2]))


def left_of(t):
    """Horizontal unit vector to the left of horizontal direction t (y up)."""
    return (-t[2], 0.0, t[0])


# ---------------------------------------------------------------- graph handling

def lift_graph(nodes):
    """Return {id: new_y}: graph lifted by GRAPH_RAISE, faded out towards tile-border nodes."""
    def is_border(p):
        return abs(p[0]) >= TILE_HALF_SIZE - 0.5 or abs(p[2]) >= TILE_HALF_SIZE - 0.5

    dist = {k: math.inf for k in nodes}
    heap = []
    for k, (p, _) in nodes.items():
        if is_border(p):
            dist[k] = 0.0
            heap.append((0.0, k))
    heapq.heapify(heap)
    while heap:
        d, k = heapq.heappop(heap)
        if d > dist[k]:
            continue
        for o in nodes[k][1]:
            if o not in nodes:
                continue
            nd = d + math.dist(nodes[k][0], nodes[o][0])
            if nd < dist[o]:
                dist[o] = nd
                heapq.heappush(heap, (nd, o))

    new_y = {}
    for k, (p, _) in nodes.items():
        factor = 1.0 if math.isinf(dist[k]) else min(1.0, dist[k] / BORDER_RAMP_LENGTH)
        new_y[k] = p[1] + GRAPH_RAISE * factor
    return new_y


def build_chains(nodes):
    """Split the graph into polylines (chains).

    At a node with 2 links the chain continues; at a node with 3 links the two links that form
    the straightest line are paired (the through route) and the third one ends there.
    Returns a list of (node_id_list, is_through_preferred) sorted with long chains first.
    """
    links = {k: [o for o in l if o in nodes and o != k] for k, (_, l) in nodes.items()}
    links = {k: list(dict.fromkeys(v)) for k, v in links.items()}

    pair = {}   # (node, neighbour) -> neighbour on the other side, if the chain continues
    for k, nbrs in links.items():
        p = nodes[k][0]
        if len(nbrs) == 2:
            a, b = nbrs
            pair[(k, a)] = b
            pair[(k, b)] = a
        elif len(nbrs) >= 3:
            best = None
            for i in range(len(nbrs)):
                for j in range(i + 1, len(nbrs)):
                    da = horizontal_dir(p, nodes[nbrs[i]][0])
                    db = horizontal_dir(p, nodes[nbrs[j]][0])
                    straightness = -dot(da, db)          # 1.0 = perfectly straight through
                    if best is None or straightness > best[0]:
                        best = (straightness, nbrs[i], nbrs[j])
            _, a, b = best
            pair[(k, a)] = b
            pair[(k, b)] = a

    used_edges = set()
    chains = []

    def walk(start, first):
        chain = [start, first]
        used_edges.add(frozenset((start, first)))
        prev, cur = start, first
        while (cur, prev) in pair:
            nxt = pair[(cur, prev)]
            edge = frozenset((cur, nxt))
            if edge in used_edges:
                break
            used_edges.add(edge)
            chain.append(nxt)
            prev, cur = cur, nxt
        return chain

    # Start from chain ends (nodes where some link is not paired), then pick up closed loops.
    for k, nbrs in links.items():
        for o in nbrs:
            if (k, o) not in pair and frozenset((k, o)) not in used_edges:
                chains.append(walk(k, o))
    for k, nbrs in links.items():
        for o in nbrs:
            if frozenset((k, o)) not in used_edges:
                chains.append(walk(k, o))

    def length(chain):
        return sum(math.dist(nodes[a][0], nodes[b][0]) for a, b in zip(chain, chain[1:]))

    # A chain is a "through" chain if neither end is the unpaired branch of a junction.
    def is_branch_end(chain_end, neighbour):
        return len(links[chain_end]) >= 3 and (chain_end, neighbour) not in pair

    result = []
    for c in chains:
        branch = is_branch_end(c[0], c[1]) or is_branch_end(c[-1], c[-2])
        result.append((c, not branch))
    result.sort(key=lambda item: (not item[1], -length(item[0])))
    return result


# ---------------------------------------------------------------- geometry builder

def polygon_normal(points):
    """Newell's method: unit normal of a planar polygon, right-hand rule on the point order."""
    nx = ny = nz = 0.0
    for i in range(len(points)):
        a = points[i]
        b = points[(i + 1) % len(points)]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return norm((nx, ny, nz))


@dataclass
class Builder:
    """Collects flat-shaded planar polygons, each tagged with a horizontal position for chunking.

    A polygon is triangulated as a fan from its first vertex, so it must be convex or at least
    fully visible from vertex 0 (true for the shapes built here).
    """
    polys: list = field(default_factory=list)   # (points, normal, colour, anchor_xz)
    collision: list = field(default_factory=list)   # (triangle points, anchor_xz) for .phys

    def polygon(self, points, color, anchor, outward):
        """Add a planar polygon; vertex order is fixed up so the normal points along `outward`."""
        points = list(points)
        n = polygon_normal(points)
        if dot(n, outward) < 0:
            points = [points[0]] + points[:0:-1]     # reverse, keeping the fan origin first
            n = mul(n, -1.0)
        self.polys.append((tuple(points), n, color, anchor))

    def quad(self, p0, p1, p2, p3, color, anchor, outward):
        self.polygon((p0, p1, p2, p3), color, anchor, outward)


def concrete_sleeper(builder, center, axis_u, axis_t, span, seat_y):
    """Concrete sleeper: tapered sides; a profiled top for a standard sleeper, flat for a long
    turnout bearer (span extended by merging). seat_y = height of the rail seat (rail foot)."""
    bottom_y = seat_y - SLEEPER_HEIGHT
    anchor = (center[0], center[2])
    standard = span[0] <= -SLEEPER_HALF_LENGTH + 1e-6 and span[0] >= -SLEEPER_HALF_LENGTH - 1e-6 \
        and abs(span[1] - SLEEPER_HALF_LENGTH) < 1e-6
    if standard:
        half = SLEEPER_PROFILE_HALF
        profile = without_collinear_points([(-u, h) for u, h in reversed(half[1:])] + list(half))
    else:
        profile = [(span[0], SLEEPER_HEIGHT), (span[1], SLEEPER_HEIGHT)]

    def half_width(h):
        return SLEEPER_HALF_WIDTH - SLEEPER_TOP_HALF_WIDTH_LOSS * h / SLEEPER_HEIGHT

    def point(u, h, side):
        p = add(add(center, mul(axis_u, u)), mul(axis_t, side * half_width(h)))
        return (p[0], bottom_y + h, p[2])

    u_min, u_max = profile[0][0], profile[-1][0]
    # top surface, one quad per profile segment
    for (u0, h0), (u1, h1) in zip(profile, profile[1:]):
        up = add(mul(axis_u, -(h1 - h0)), (0.0, u1 - u0, 0.0))
        builder.quad(point(u0, h0, -1), point(u1, h1, -1), point(u1, h1, 1), point(u0, h0, 1),
                     COLOR_SLEEPER_TOP, anchor, up)
    # long sides: polygon = bottom edge + top profile. The fan starts at the bottom middle point:
    # every top profile point is visible from there (a fan from a bottom corner would cut
    # above the low middle part of the sleeper).
    # A flat top needs only a quad per side (20 vertices per sleeper instead of 28).
    u_mid = (u_min + u_max) / 2
    for side in (-1, 1):
        if len(profile) == 2:
            pts = [point(u_min, 0.0, side), point(u_max, 0.0, side),
                   point(u_max, profile[1][1], side), point(u_min, profile[0][1], side)]
        else:
            pts = [point(u_mid, 0.0, side), point(u_max, 0.0, side)]
            pts += [point(u, h, side) for u, h in reversed(profile)]
            pts.append(point(u_min, 0.0, side))
        builder.polygon(pts, COLOR_SLEEPER, anchor, mul(axis_t, side))
    # ends
    for u, h, sign in ((u_min, profile[0][1], -1), (u_max, profile[-1][1], 1)):
        builder.quad(point(u, 0.0, -1), point(u, 0.0, 1), point(u, h, 1), point(u, h, -1),
                     COLOR_SLEEPER, anchor, mul(axis_u, sign))


def without_collinear_points(profile):
    """Drop profile points lying on the straight line between their neighbours."""
    result = [profile[0]]
    for i in range(1, len(profile) - 1):
        (u0, h0), (u1, h1), (u2, h2) = result[-1], profile[i], profile[i + 1]
        if abs((u1 - u0) * (h2 - h0) - (h1 - h0) * (u2 - u0)) > 1e-9:
            result.append(profile[i])
    result.append(profile[-1])
    return result


def oriented_rect_overlap(c1, u1, span1, t1, hw1, c2, u2, span2, t2, hw2):
    """Separating-axis test for two horizontal rectangles given by centre, axes and extents."""
    def corners(c, u, span, t, hw):
        pts = []
        for su in span:
            for st in (-hw, hw):
                p = add(add(c, mul(u, su)), mul(t, st))
                pts.append((p[0], p[2]))
        return pts

    a = corners(c1, u1, span1, t1, hw1)
    b = corners(c2, u2, span2, t2, hw2)
    for axis in ((u1[0], u1[2]), (t1[0], t1[2]), (u2[0], u2[2]), (t2[0], t2[2])):
        pa = [p[0] * axis[0] + p[1] * axis[1] for p in a]
        pb = [p[0] * axis[0] + p[1] * axis[1] for p in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


@dataclass
class Sleeper:
    center: tuple
    axis_u: tuple        # along the sleeper (left of track direction)
    axis_t: tuple        # along the track
    span: list           # [min_u, max_u]
    top_y: float
    chain: int = -1      # index of the chain it was placed on
    segment: int = -1    # index of the polyline segment within that chain
    order: int = -1      # running number along the chain
    direction: tuple = (0.0, 0.0, 1.0)   # 3D unit direction of the segment (includes grade)

    def is_standard(self):
        return abs(self.span[0] + SLEEPER_HALF_LENGTH) < 1e-6 and \
            abs(self.span[1] - SLEEPER_HALF_LENGTH) < 1e-6


def chain_points(chain, nodes, node_y):
    """Polyline of the rail-top line (graph height plus RAIL_TOP_OFFSET)."""
    return [(nodes[k][0][0], node_y[k] + RAIL_TOP_OFFSET, nodes[k][0][2]) for k in chain]


def densify(points, max_step):
    """Insert collinear points so no polyline piece is longer than max_step."""
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.ceil(math.dist(a, b) / max_step)))
        for i in range(1, n + 1):
            f = i / n
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f))
    return out


def miter_for(cut_dir, segment_dir):
    """Lateral vector + scale for a cross-section cut perpendicular to cut_dir on a segment."""
    n = left_of(cut_dir)
    c = dot(n, left_of(segment_dir))
    return (n, 1.0 / c if c > 0.3 else 1.0)


def miter_offsets(points):
    """Per-point (lateral unit vector, scale) so offsets stay parallel through corners."""
    tangents = [horizontal_dir(a, b) for a, b in zip(points, points[1:])]
    result = []
    for i in range(len(points)):
        if i == 0:
            n = left_of(tangents[0])
            result.append((n, 1.0))
        elif i == len(points) - 1:
            n = left_of(tangents[-1])
            result.append((n, 1.0))
        else:
            n1 = left_of(tangents[i - 1])
            n2 = left_of(tangents[i])
            n = norm(add(n1, n2))
            c = dot(n, n2)
            scale = 1.0 / c if c > 0.3 else 1.0
            result.append((n, scale))
    return result


def build_rails(builder, points, capped_start, capped_end, start_dir=None, end_dir=None):
    """Extrude the rail profile along a chain polyline for both rails.

    start_dir / end_dir: optional horizontal track direction to use for the end cross-section
    (so two tiles meeting at a border cut their rails along the same plane).
    """
    offsets = miter_offsets(points)

    def end_offset(cut, segment_dir):
        # cut may be a plain direction or (cut plane normal, reference track direction); with a
        # reference both tiles meeting at a border compute exactly the same cross-section.
        reference = segment_dir
        if isinstance(cut[0], tuple):
            cut, reference = cut
        if dot(cut, segment_dir) < 0:
            cut = mul(cut, -1.0)
        if dot(reference, cut) < 0:
            reference = mul(reference, -1.0)
        return miter_for(cut, reference)

    if start_dir is not None:
        offsets[0] = end_offset(start_dir, horizontal_dir(points[0], points[1]))
    if end_dir is not None:
        offsets[-1] = end_offset(end_dir, horizontal_dir(points[-2], points[-1]))
    for side in (-1.0, 1.0):
        rings = []
        for p, (n, scale) in zip(points, offsets):
            ring = []
            for u, h in RAIL_PROFILE:
                lateral = (side * RAIL_HALF_GAUGE + u) * scale
                q = add(p, mul(n, lateral))
                ring.append((q[0], p[1] + h, q[2]))
            rings.append(ring)
        add_rail_strips(builder, points, rings)
        t_start = horizontal_dir(points[0], points[1])
        t_end = horizontal_dir(points[-2], points[-1])
        if capped_start:
            add_rail_cap(builder, rings[0], mul(t_start, -1.0))
        if capped_end:
            add_rail_cap(builder, rings[-1], t_end)
    add_track_collision(builder, points, offsets)


# Collision (.phys) cross-section across the whole track, relative to the rail-top line:
# a slab at sleeper-top height (sleepers + ballast to walk on) with two low rail ridges.
# The ridges stop RAIL_COLLISION_TOP_DROP below the rail top so train wheels (which ride the
# graph) never touch them - North Sawyer Overhaul uses the same clearance (~11 cm).
RAIL_COLLISION_TOP_DROP = 0.11
RAIL_COLLISION_HALF_WIDTH = 0.04
COLLISION_PIECE_LENGTH = 20.0


def _collision_profile():
    seat = -RAIL_HEIGHT
    ridge = -RAIL_COLLISION_TOP_DROP
    bottom = seat - SLEEPER_HEIGHT
    g, w, half = RAIL_HALF_GAUGE, RAIL_COLLISION_HALF_WIDTH, SLEEPER_HALF_LENGTH
    return [(-half, bottom), (-half, seat),
            (-g - w, seat), (-g - w, ridge), (-g + w, ridge), (-g + w, seat),
            (g - w, seat), (g - w, ridge), (g + w, ridge), (g + w, seat),
            (half, seat), (half, bottom)]


COLLISION_PROFILE = _collision_profile()


def add_track_collision(builder, points, offsets):
    """Extrude the collision cross-section along the chain (same miters and border cuts as
    the rails). Triangles use the game winding: (b-a)x(c-a) points into the solid."""
    rings = []
    for p, (n, scale) in zip(points, offsets):
        rings.append([(p[0] + n[0] * u * scale, p[1] + h, p[2] + n[2] * u * scale)
                      for u, h in COLLISION_PROFILE])
    for i in range(len(rings) - 1):
        n_seg = left_of(horizontal_dir(points[i], points[i + 1]))
        # long straight segments are cut into pieces of at most COLLISION_PIECE_LENGTH
        # (very long thin triangles are a poor fit for the physics engine)
        pieces = max(1, int(math.ceil(math.dist(points[i], points[i + 1]) / COLLISION_PIECE_LENGTH)))
        for k in range(pieces):
            f0, f1 = k / pieces, (k + 1) / pieces
            a = [lerp(p, q, f0) for p, q in zip(rings[i], rings[i + 1])]
            b = [lerp(p, q, f1) for p, q in zip(rings[i], rings[i + 1])]
            mid = lerp(points[i], points[i + 1], (f0 + f1) / 2)
            add_collision_piece(builder, a, b, n_seg, (mid[0], mid[2]))


def lerp(p, q, f):
    return (p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f, p[2] + (q[2] - p[2]) * f)


def add_collision_piece(builder, a, b, n_seg, anchor):
    for e in range(len(COLLISION_PROFILE) - 1):
        (u0, h0), (u1, h1) = COLLISION_PROFILE[e], COLLISION_PROFILE[e + 1]
        # profile goes left -> over the top -> right, i.e. clockwise: outward = (-dh, du)
        eu, eh = -(h1 - h0), (u1 - u0)
        outward = (n_seg[0] * eu, eh, n_seg[2] * eu)
        quad = [a[e], a[e + 1], b[e + 1], b[e]]
        if dot(polygon_normal(quad), outward) < 0:
            quad = [quad[0], quad[3], quad[2], quad[1]]
        builder.collision.append(((quad[0], quad[2], quad[1]), anchor))
        builder.collision.append(((quad[0], quad[3], quad[2]), anchor))


def collision_to_phys_chunks(collision):
    """Group collision triangles into the same spatial chunks as the meshes.
    Returns {chunk_name: [triangle, ...]}."""
    chunks = {}
    for triangle, (ax, az) in collision:
        key = (int(math.floor(ax / CHUNK_SIZE)), int(math.floor(az / CHUNK_SIZE)))
        chunks.setdefault(key, []).append(triangle)
    return {"c%+d_%+d" % key: chunks[key] for key in sorted(chunks)}


RAIL_EDGE_COUNT = len(RAIL_PROFILE) - 1
RAIL_MAX_RINGS_PER_STRIP = 400


def add_rail_strips(builder, points, rings):
    """Rail surface with vertices shared between consecutive pieces.

    Every profile edge is a flat face across the rail, so each ring keeps two vertices per edge
    (hard edges around the profile), but along the rail one ring of vertices serves both
    neighbouring pieces, with the normal averaged between them (the pieces differ by about one
    degree after smoothing). That halves the rail vertices compared with separate quads.
    Strips are cut where the pieces change mesh chunk, so chunking works as for polygons.
    """
    segment_count = len(rings) - 1
    face_normals = []          # [segment][edge]
    flipped = []               # [segment][edge]: quad order must be reversed to face outwards
    for i in range(segment_count):
        a, b = rings[i], rings[i + 1]
        n_seg = left_of(horizontal_dir(points[i], points[i + 1]))
        normals, flips = [], []
        for e in range(RAIL_EDGE_COUNT):
            (u0, h0), (u1, h1) = RAIL_PROFILE[e], RAIL_PROFILE[e + 1]
            # The profile is traversed clockwise in the (lateral, up) plane, so the outward
            # normal of an edge (du, dh) is (-dh, du).
            eu, eh = -(h1 - h0), (u1 - u0)
            outward = (n_seg[0] * eu, eh, n_seg[2] * eu)
            n = polygon_normal((a[e], a[e + 1], b[e + 1], b[e]))
            flip = dot(n, outward) < 0
            normals.append(mul(n, -1.0) if flip else n)
            flips.append(flip)
        face_normals.append(normals)
        flipped.append(flips)

    def ring_normal(j, e):
        around = [face_normals[i][e] for i in (j - 1, j) if 0 <= i < segment_count]
        n = norm(around[0] if len(around) == 1 else add(around[0], around[1]))
        return n if n != (0.0, 0.0, 0.0) else (0.0, 1.0, 0.0)

    def chunk_of(i):
        return (int(math.floor((points[i][0] + points[i + 1][0]) / 2 / CHUNK_SIZE)),
                int(math.floor((points[i][2] + points[i + 1][2]) / 2 / CHUNK_SIZE)))

    start = 0
    while start < segment_count:
        end = start + 1
        while (end < segment_count and end - start < RAIL_MAX_RINGS_PER_STRIP - 1
               and chunk_of(end) == chunk_of(start)):
            end += 1
        vertices, normals, colors, triangles = [], [], [], []
        for j in range(start, end + 1):
            for e in range(RAIL_EDGE_COUNT):
                color = COLOR_RAIL_TOP if e == RAIL_TOP_EDGE_INDEX else COLOR_RAIL_SIDE
                n = ring_normal(j, e)
                for k in (e, e + 1):
                    vertices.append(rings[j][k])
                    normals.append(n)
                    colors.append(color)
        per_ring = 2 * RAIL_EDGE_COUNT
        for i in range(start, end):
            row_a = (i - start) * per_ring
            row_b = row_a + per_ring
            for e in range(RAIL_EDGE_COUNT):
                # quad a[e], a[e+1], b[e+1], b[e], same winding rules as Builder.polygon
                quad = [row_a + 2 * e, row_a + 2 * e + 1, row_b + 2 * e + 1, row_b + 2 * e]
                if flipped[i][e]:
                    quad = [quad[0], quad[3], quad[2], quad[1]]
                triangles += [(quad[0], quad[2], quad[1]), (quad[0], quad[3], quad[2])]
        anchor = ((points[start][0] + points[start + 1][0]) / 2,
                  (points[start][2] + points[start + 1][2]) / 2)
        builder.polys.append((tuple(vertices), tuple(normals), tuple(colors), anchor,
                              tuple(triangles)))
        start = end


def add_rail_cap(builder, ring, outward):
    """Close a rail end with three quads: foot, web, head."""
    anchor = (ring[0][0], ring[0][2])
    pieces = [(0, 1, 12, 13), (2, 3, 10, 11), (4, 5, 8, 9), (5, 6, 7, 8)]   # foot, web, head
    for i0, i1, i2, i3 in pieces:
        builder.quad(ring[i0], ring[i1], ring[i2], ring[i3], COLOR_RAIL_SIDE, anchor, outward)


def sample_chain(points, spacing, phase):
    """Yield (position, horizontal tangent, segment index, 3D segment direction) every `spacing`
    metres (measured horizontally) along the polyline."""
    s_next = phase
    s_base = 0.0
    for seg_index, (a, b) in enumerate(zip(points, points[1:])):
        seg_len = math.dist((a[0], a[2]), (b[0], b[2]))
        if seg_len < 1e-6:
            continue
        t = horizontal_dir(a, b)
        d3 = norm(sub(b, a))
        while s_next <= s_base + seg_len:
            f = (s_next - s_base) / seg_len
            pos = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)
            yield pos, t, seg_index, d3
            s_next += spacing
        s_base += seg_len


def place_sleepers(chains_points):
    """Place sleepers for all chains; overlapping ones are merged into long timbers."""
    placed = []
    grid = {}

    def cells_near(c, radius=6.0):
        cx, cz = int(c[0] // 5), int(c[2] // 5)
        r = int(radius // 5) + 1
        for ix in range(cx - r, cx + r + 1):
            for iz in range(cz - r, cz + r + 1):
                yield from grid.get((ix, iz), [])

    def register(s):
        grid.setdefault((int(s.center[0] // 5), int(s.center[2] // 5)), []).append(s)

    for chain_index, points in enumerate(chains_points):
        order = 0
        for pos, t, seg_index, d3 in sample_chain(points, SLEEPER_SPACING, SLEEPER_SPACING / 2):
            order += 1
            u = left_of(t)
            top_y = pos[1] - RAIL_HEIGHT
            span = [-SLEEPER_HALF_LENGTH, SLEEPER_HALF_LENGTH]
            rails = [add(pos, mul(u, -RAIL_HALF_GAUGE)), add(pos, mul(u, RAIL_HALF_GAUGE))]

            merged = False
            for other in cells_near(pos):
                if not oriented_rect_overlap(pos, u, span, t, SLEEPER_HALF_WIDTH,
                                             other.center, other.axis_u, other.span,
                                             other.axis_t, SLEEPER_HALF_WIDTH):
                    continue
                angle = math.acos(max(-1.0, min(1.0, abs(dot(t, other.axis_t)))))
                if angle < SLEEPER_MERGE_MAX_ANGLE:
                    # Extend the existing timber so it carries this track's rails as well.
                    for r in rails:
                        s = dot(sub(r, other.center), other.axis_u)
                        other.span[0] = max(-SLEEPER_MAX_HALF_SPAN,
                                            min(other.span[0], s - SLEEPER_EXTENSION_MARGIN))
                        other.span[1] = min(SLEEPER_MAX_HALF_SPAN,
                                            max(other.span[1], s + SLEEPER_EXTENSION_MARGIN))
                merged = True
                break
            if not merged:
                s = Sleeper(pos, u, t, span, top_y, chain_index, seg_index, order, d3)
                placed.append(s)
                register(s)
    return placed


def build_track_geometry(nodes, node_y=None, continuing=frozenset(), border_ends=None):
    """Build rails and sleepers for one tile.

    node_y: lifted node heights (default: lift_graph for a single isolated tile).
    continuing: node ids where the track continues into another modded tile (no rail end caps).
    border_ends: {node_id: ((x, y, z) shared rail-top point in tile coordinates, shared direction)}
        for joined border nodes, so both tiles end their rails on exactly the same cross-section.
    """
    border_ends = border_ends or {}
    if node_y is None:
        node_y = lift_graph(nodes)
    chains = build_chains(nodes)
    builder = Builder()
    chains_points = []
    for chain, _through in chains:
        pts = chain_points(chain, nodes, node_y)
        start_dir = end_dir = None
        if chain[0] in border_ends:
            pts[0], start_dir = border_ends[chain[0]]
        if chain[-1] in border_ends:
            pts[-1], end_dir = border_ends[chain[-1]]
        chains_points.append(pts)

        def capped(k):
            return len(nodes[k][1]) == 1 and k not in continuing

        build_rails(builder, densify(pts, RAIL_PIECE_LENGTH),
                    capped_start=capped(chain[0]), capped_end=capped(chain[-1]),
                    start_dir=start_dir, end_dir=end_dir)
    for s in place_sleepers(chains_points):
        concrete_sleeper(builder, s.center, s.axis_u, s.axis_t, s.span, s.top_y)
    return builder, node_y, chains


def lift_graph_multi(tiles, origins):
    """Lift several adjacent tiles as one network.

    tiles: {tile_name: nodes}, origins: {tile_name: (world_x, world_z)} of each tile centre.
    Border nodes that meet a node of another tile in the set are joined (no ramp there); the lift
    only fades out at border nodes that lead to tiles outside the set.
    Returns ({tile_name: {node_id: new_y}}, {tile_name: set(continuing node ids)}).
    """
    def is_border(p):
        return abs(p[0]) >= TILE_HALF_SIZE - 0.5 or abs(p[2]) >= TILE_HALF_SIZE - 0.5

    world = {}
    for t, nodes in tiles.items():
        ox, oz = origins[t]
        for k, (p, _) in nodes.items():
            world[(t, k)] = (p[0] + ox, p[1], p[2] + oz)

    adjacency = {key: [] for key in world}
    for t, nodes in tiles.items():
        for k, (_, links) in nodes.items():
            for o in links:
                if o in nodes and o != k:
                    adjacency[(t, k)].append(((t, o), math.dist(world[(t, k)], world[(t, o)])))

    border = [key for key in world if is_border(tiles[key[0]][key[1]][0])]
    joined = set()
    partner = {}
    for a in border:
        for b in border:
            if a[0] != b[0] and math.hypot(world[a][0] - world[b][0], world[a][2] - world[b][2]) < 0.5:
                adjacency[a].append((b, 0.0))
                joined.add(a)
                partner[a] = b

    dist = {key: math.inf for key in world}
    heap = [(0.0, key) for key in border if key not in joined]
    for _, key in heap:
        dist[key] = 0.0
    heapq.heapify(heap)
    while heap:
        d, key = heapq.heappop(heap)
        if d > dist[key]:
            continue
        for other, w in adjacency[key]:
            if d + w < dist[other]:
                dist[other] = d + w
                heapq.heappush(heap, (d + w, other))

    node_y = {t: {} for t in tiles}
    for (t, k), d in dist.items():
        factor = 1.0 if math.isinf(d) else min(1.0, d / BORDER_RAMP_LENGTH)
        node_y[t][k] = tiles[t][k][0][1] + GRAPH_RAISE * factor
    continuing = {t: {k for (tt, k) in joined if tt == t} for t in tiles}

    # Shared rail end cross-section for every joined border pair: midpoint, mean height and the
    # mean of the two tracks' directions at the border.
    def inner_neighbour(key):
        t, k = key
        return (t, [o for o in tiles[t][k][1] if o in tiles[t]][0])

    border_ends = {t: {} for t in tiles}
    for a, b in partner.items():
        pa, pb = world[a], world[b]
        dir_a = horizontal_dir(world[inner_neighbour(a)], pa)      # heading out of tile a
        dir_b = horizontal_dir(pb, world[inner_neighbour(b)])      # heading into tile b
        shared_dir = norm(add(dir_a, dir_b))
        y = (node_y[a[0]][a[1]] + node_y[b[0]][b[1]]) / 2 + RAIL_TOP_OFFSET
        mx, mz = (pa[0] + pb[0]) / 2, (pa[2] + pb[2]) / 2
        ox, oz = origins[a[0]]
        border_ends[a[0]][a[1]] = ((mx - ox, y, mz - oz), shared_dir)
    return node_y, continuing, border_ends


# ---------------------------------------------------------------- mesh output

def polys_to_meshes(polys):
    """Group polygons into spatial chunks, each below the vertex limit. Returns {chunk_name: Mesh}."""
    chunks = {}
    for poly in polys:
        ax, az = poly[3]
        key = (int(math.floor(ax / CHUNK_SIZE)), int(math.floor(az / CHUNK_SIZE)))
        chunks.setdefault(key, []).append(poly)

    meshes = {}
    for key in sorted(chunks):
        part = 0
        batch = []
        count = 0
        for poly in chunks[key] + [None]:
            if poly is None or count + len(poly[0]) > MAX_VERTICES_PER_MESH:
                if batch:
                    meshes["c%+d_%+d_%d" % (key[0], key[1], part)] = polys_to_mesh(batch)
                    part += 1
                batch, count = [], 0
            if poly is not None:
                batch.append(poly)
                count += len(poly[0])
    return meshes


def polys_to_mesh(polys):
    mesh = Mesh()
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for poly in polys:
        base = len(mesh.vertices)
        points = poly[0]
        for p in points:
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
        if len(poly) == 5:
            # shared-vertex strip: per-vertex normals and colours, triangles in game winding
            _, normals, colors, _, triangles = poly
            for p, n, color in zip(points, normals, colors):
                mesh.vertices.append((p[0], p[1], p[2], color[0], color[1], color[2], color[3],
                                      n[0], n[1], n[2]))
            for t in triangles:
                mesh.indices += [base + t[0], base + t[1], base + t[2]]
            continue
        _, n, color, _ = poly
        for p in points:
            mesh.vertices.append((p[0], p[1], p[2], color[0], color[1], color[2], color[3],
                                  n[0], n[1], n[2]))
        # Fan triangulation. Game convention: (b-a)x(c-a) points against the normal,
        # so each counter-clockwise triangle (0, i, i+1) is written as (0, i+1, i).
        for i in range(1, len(points) - 1):
            mesh.indices += [base, base + i + 1, base + i]
    mesh.submeshes = [Submesh(0, len(mesh.indices), 0, tuple(lo), tuple(hi))]
    return mesh


# ---------------------------------------------------------------- whole-map version (v3):
# rails and turnout bearers are unique per tile, standard sleepers are instanced panels.

PANEL_SIZES = (64, 32, 16, 8, 4, 2, 1)


def build_sleeper_panel(count):
    """Mesh with `count` standard concrete sleepers along local +z (spacing SLEEPER_SPACING).
    Local axes: x = right of the track, y = up, z = forward. Origin = first sleeper centre at
    rail-seat height."""
    builder = Builder()
    for i in range(count):
        concrete_sleeper(builder, (0.0, 0.0, i * SLEEPER_SPACING), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                         [-SLEEPER_HALF_LENGTH, SLEEPER_HALF_LENGTH], 0.0)
    return polys_to_mesh(builder.polys)


def panel_matrix(origin, direction):
    """Row-major 4x4 for a panel placed at origin, facing the 3D unit direction.

    Stormworks transforms are row vectors: world = x*row0 + y*row1 + z*row2 + row3.
    row0 = right (horizontal), row2 = forward (with grade), row1 = row2 x row0 (determinant +1).
    """
    forward = direction
    right = norm((forward[2], 0.0, -forward[0]))
    up = cross(forward, right)
    return [[right[0], right[1], right[2], 0.0],
            [up[0], up[1], up[2], 0.0],
            [forward[0], forward[1], forward[2], 0.0],
            [origin[0], origin[1], origin[2], 1.0]]


def sleeper_runs(sleepers):
    """Split standard sleepers into straight runs (same chain and segment, consecutive), then
    into binary panel sizes. Returns [(panel_size, origin, direction)]."""
    by_key = {}
    for s in sleepers:
        if s.is_standard():
            by_key.setdefault((s.chain, s.segment), []).append(s)
    placements = []
    for run_sleepers in by_key.values():
        run_sleepers.sort(key=lambda s: s.order)
        # split where the running number has gaps (merged sleepers removed in between)
        runs = [[run_sleepers[0]]]
        for s in run_sleepers[1:]:
            if s.order == runs[-1][-1].order + 1:
                runs[-1].append(s)
            else:
                runs.append([s])
        for run in runs:
            i = 0
            while i < len(run):
                left = len(run) - i
                size = next(p for p in PANEL_SIZES if p <= left)
                first = run[i]
                origin = (first.center[0], first.top_y, first.center[2])
                placements.append((size, origin, first.direction))
                i += size
    return placements


def build_tile_v3(nodes, node_y, border_cuts, continuing):
    """One tile: unique geometry (rails + non-standard bearers) and instanced sleeper placements.

    border_cuts: {node_id: horizontal unit normal of the tile edge} -> rails end exactly on the
    tile border plane, so the neighbouring tile's rails meet them without a wedge gap.
    """
    chains = build_chains(nodes)
    builder = Builder()
    chains_points = []
    for chain, _through in chains:
        pts = chain_points(chain, nodes, node_y)
        chains_points.append(pts)
        start_dir = border_cuts.get(chain[0])
        end_dir = border_cuts.get(chain[-1])

        def capped(k):
            return len(nodes[k][1]) == 1 and k not in continuing

        build_rails(builder, densify(pts, RAIL_PIECE_LENGTH),
                    capped_start=capped(chain[0]), capped_end=capped(chain[-1]),
                    start_dir=start_dir, end_dir=end_dir)
    sleepers = place_sleepers(chains_points)
    for s in sleepers:
        if not s.is_standard():
            concrete_sleeper(builder, s.center, s.axis_u, s.axis_t, s.span, s.top_y)
    return builder, sleeper_runs(sleepers), chains


# ---------------------------------------------------------------- whole-map version (v4):
# everything baked into a few large meshes per tile (few draw calls).

def _sleeper_template():
    """Polygons of one standard sleeper in panel-local space (x right, y up, z forward)."""
    builder = Builder()
    concrete_sleeper(builder, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                     [-SLEEPER_HALF_LENGTH, SLEEPER_HALF_LENGTH], 0.0)
    return builder.polys


def add_standard_sleeper(builder, template, sleeper):
    """Append a transformed copy of the template sleeper (fast path, no per-sleeper rebuild)."""
    m = panel_matrix((sleeper.center[0], sleeper.top_y, sleeper.center[2]), sleeper.direction)
    anchor = (sleeper.center[0], sleeper.center[2])
    for points, n, color, _ in template:
        pts = tuple((p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0] + m[3][0],
                     p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1] + m[3][1],
                     p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] + m[3][2]) for p in points)
        nn = (n[0] * m[0][0] + n[1] * m[1][0] + n[2] * m[2][0],
              n[0] * m[0][1] + n[1] * m[1][1] + n[2] * m[2][1],
              n[0] * m[0][2] + n[1] * m[1][2] + n[2] * m[2][2])
        builder.polys.append((pts, nn, color, anchor))


def build_tile_v4(nodes, node_y, border_cuts, continuing):
    """One tile, fully baked: rails, standard sleepers and turnout bearers in the same meshes."""
    chains = build_chains(nodes)
    builder = Builder()
    chains_points = []
    for chain, _through in chains:
        pts = chain_points(chain, nodes, node_y)
        chains_points.append(pts)

        def capped(k):
            return len(nodes[k][1]) == 1 and k not in continuing

        build_rails(builder, densify(pts, RAIL_PIECE_LENGTH),
                    capped_start=capped(chain[0]), capped_end=capped(chain[-1]),
                    start_dir=border_cuts.get(chain[0]), end_dir=border_cuts.get(chain[-1]))
    template = _sleeper_template()
    sleepers = place_sleepers(chains_points)
    for s in sleepers:
        if s.is_standard():
            add_standard_sleeper(builder, template, s)
        else:
            concrete_sleeper(builder, s.center, s.axis_u, s.axis_t, s.span, s.top_y)
    return builder, len(sleepers), chains
