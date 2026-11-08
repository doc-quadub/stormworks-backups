"""Overhead line (catenary) in the style of Soviet / Russian 25 kV AC lines - visual only.

  * plan_catenary() - supports along the electrified tracks (span length from the curve
                      radius), the side for each pole, heights; spans between the supports.
  * add_catenary()  - geometry for one tile: reinforced concrete poles, insulated cantilevers,
                      registration arms, messenger wire with droppers, contact wire, the two
                      feeder ("DPR") wires on the field side of the poles. Poles get a collider,
                      wires do not (pantographs must not catch on anything).

All positions are world coordinates (x east, y up, z north); add_catenary moves them into the
tile. Heights of the wires are measured from the rail top.
"""

import heapq
import math
import os
import re

import rails
import signs
import swmesh

# ---------------------------------------------------------------- dimensions (metres)

CONTACT_WIRE_HEIGHT = 6.25        # above the rail top (allowed 5.75 - 6.80)
SYSTEM_HEIGHT = 1.80              # messenger above the contact wire at the supports
# Zigzag: on straight track and curves of radius >= ZIGZAG_MIN_RADIUS the contact wire goes
# +-STAGGER from the track axis, alternating support by support (away from the pole: reverse
# fixator, towards the pole: direct fixator). On tighter curves there is no zigzag: the wire is
# moved to the outer side of the curve by half the chord sag, at most CURVE_OFFSET_MAX.
STAGGER = 0.175
ZIGZAG_MIN_RADIUS = 3000.0
CURVE_OFFSET_MAX = 0.30
MAX_SPAN = 70.0
MIN_SPAN = 30.0
CURVE_SAG_ALLOWANCE = 0.60        # a span may cut the curve by this much (chord vs track)
MESSENGER_SAG_70M = 1.10          # sag of the messenger in a 70 m span (scales with span^2)
FEEDER_SAG_70M = 1.40
DROPPER_SPACING = 9.0

POLE_OFFSET = 3.30                # pole centre from the track axis (clearance >= 3.1 m)
POLE_HEIGHT = 10.00               # above the pole base level (SS136.6: 13.6 m long)
POLE_BURIED = 0.50                # drawn below the ground point (slopes)
POLE_BASE_BELOW_RAIL = 0.80       # every pole stands on this level (below the rail top), so all
                                  # cantilevers sit at the same height on it
FOUNDATION_SIDE = 0.90            # square concrete block under a pole whose ground is lower
FOUNDATION_BURIED = 1.20
COLOR_FOUNDATION = (128, 126, 120, 255)
POLE_RADIUS_GROUND = 0.217        # 433 mm at the ground line (490 mm at the foot, 290 at top)
POLE_RADIUS_TOP = 0.145
POLE_SIDES = 8

# Cantilever (after a manual drawing): the inclined tube goes from a hinge on the pole 250 mm
# below the contact wire up to the bracket ("skoba") above the messenger; the tie rod holds the
# bracket from higher up the pole; the messenger hangs from the bracket. Fixators hold the
# contact wire: the direct one is a steady arm straight from the tube to the wire (wire pulled
# towards the pole); the reverse one is a main fixator tube from the cantilever out past the wire,
# held up by a string from the messenger, with the steady arm coming back from its end (wire
# pulled away from the pole). Heights above the rail top.
CANTILEVER_HINGE = CONTACT_WIRE_HEIGHT - 0.25
SKOBA_ABOVE_MESSENGER = 0.40
TIE_ABOVE_SKOBA = 0.40
CANTILEVER_OVERHANG = 0.15        # tube runs on past the bracket
FIXATOR_ATTACH_HEIGHT = 6.60      # where the fixators leave the cantilever tube
MAIN_FIXATOR_BEYOND = 1.35        # the reverse fixator's main tube ends this far past the wire
MAIN_FIXATOR_END_HEIGHT = 6.40
STEADY_ARM_HEIGHT = 6.33          # the steady arm runs at this height and bends down to the wire
STEADY_ARM_BEND = 0.30            # bend this far from the wire
FIXATOR_STRING_BEYOND = 0.90      # string messenger -> main fixator, this far past the wire
POLE_FLAT_FACTOR = 0.88           # fittings start this far out (x pole radius): inside the flats
CLAMP_BAND_HEIGHT = 0.10
SECOND_WIRE_RAISE = 0.35          # a second wire on the same cantilever (turnout, overlap) is held
                                  # this much higher, above the first one's fixators
SECOND_ARM_ATTACH_HEIGHT = FIXATOR_ATTACH_HEIGHT + 0.55
FEEDERS = False                   # DPR line (two feeder wires on the field side): off for now
FEEDER_ARM = (0.55, 1.10)         # feeder wires, outwards from the pole axis (field side)
FEEDER_DROP = 0.45                # below the pole top

TUBE_RADIUS = 0.030
ROD_RADIUS = 0.016
INSULATOR_RADIUS = 0.085
INSULATOR_LENGTH = 0.80
SHED_SPACING_FACTOR = 0.5         # insulator discs half as far apart as spread evenly over the
                                  # insulator, sitting together in the middle
WIRE_RADIUS = 0.025              # 5 cm thick: thinner wires vanish at a distance in the game
DROPPER_RADIUS = 0.010

COLOR_CONCRETE = (150, 148, 142, 255)
COLOR_STEEL = (118, 120, 122, 255)
COLOR_INSULATOR = (112, 58, 36, 255)
COLOR_MESSENGER = (70, 70, 72, 255)
COLOR_CONTACT = (96, 74, 58, 255)
COLOR_FEEDER = (64, 64, 66, 255)

# Electrified area (test): the shortest route between ROUTE_START and ROUTE_END (world, internal
# tile-grid coordinates) and every track within ROUTE_CORRIDOR of it - the parallel main track and
# the station tracks too.
ROUTE_START = (1340.0, -3766.0)     # North Harbour station, at the canopies
ROUTE_END = (5390.0, -6710.0)       # km 6 east of North Harbour, before the junction there
ROUTE_CORRIDOR = 25.0
ELECTRIFY_ALL = True                # every track of the Sawyer mainland, not just the route above
# Tracks with anything overhead between these heights above the rail (roofs, footbridges) stay
# without wires; the line just ends before them.
COVER_LOW = 3.0
COVER_HIGH = 10.5
COVER_SAMPLE_STEP = 3.0
COVER_MARGIN = 3.0
MIN_RUN_LENGTH = 50.0              # shorter wire runs are left out
END_GAP = 6.0                      # first / last support this far from the end of a track


# ---------------------------------------------------------------- small vector helpers

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _lerp(a, b, f):
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)


def _unit(a):
    length = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    return (a[0] / length, a[1] / length, a[2] / length) if length > 1e-12 else (0.0, 1.0, 0.0)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# ---------------------------------------------------------------- planning

def _polyline_at(points, cumulative, s):
    """Point and horizontal unit tangent at arc length s along a polyline."""
    s = max(0.0, min(cumulative[-1], s))
    lo, hi = 0, len(cumulative) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cumulative[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cumulative[hi] - cumulative[lo]
    f = 0.0 if seg < 1e-9 else (s - cumulative[lo]) / seg
    a, b = points[lo], points[hi]
    t = (b[0] - a[0], b[2] - a[2])
    tl = math.hypot(*t)
    return _lerp(a, b, f), ((t[0] / tl, t[1] / tl) if tl > 1e-9 else (1.0, 0.0))


def _heading(t):
    return math.atan2(t[1], t[0])


def _turn(t1, t2):
    return (_heading(t2) - _heading(t1) + math.pi) % (2 * math.pi) - math.pi


# One pole serves the tracks of a few supports close together (two cantilevers on one pole, as
# at turnouts and between station tracks): a support moves along its track by up to
# SHARE_SHIFT (SHARE_SHIFT_AT_END for the first / last support of a wire run) onto a pole of
# another track, if that pole is on the same side and its cantilever reach stays within limits.
SHARE_SHIFT = 8.0
SHARE_SHIFT_AT_END = 16.0
SHARE_REACH = (2.6, 6.3)             # pole axis to the track axis
SHARE_KEEP_SPAN = 10.0               # moved support stays this far from its neighbours
SHARE_MIN_TRACK_GAP = 2.0            # the two tracks of a shared pole are at least this far apart
JUNCTION_DROP = 15.0                 # a side track's last support this close to its turnout is
                                     # dropped when its wire runs on over the turnout
ANCHOR_POLE_SHIFTS = (6.0, 9.0, 12.0, 4.0, -6.0, -9.0, -12.0)   # an anchor on a shared pole gets its
                                     # own pole this far beyond (away from its span; negative:
                                     # back towards it), on its side of the track or else the other
POLE_SPACING = 2.5                   # poles at least this far apart

# "DPR" line (two wires - rail, 27.5 kV for non-traction consumers): on the field side of the
# outermost poles of the bundle (portal posts too), straight through, independent of the tracks.
FEEDER_OUTER_CHECK = 15.0            # an outer pole has no electrified track within this outwards
FEEDER_MAX_SPAN = 80.0
FEEDER_SIDE_PENALTY = 4.0            # cost per metre of sideways jump between poles
FEEDER_POLE_BONUS = 6.0              # prefer hanging on every pole of the line

# A side track's wire does not end at the turnout: it runs on over the main track for a span
# (beside the main wire, OVERLAP_OFFSET apart) and is anchored on the next pole of the main track.
OVERLAP_OFFSET = 0.20
OVERLAP_FIRST_SPAN = 20.0            # the span onto the main track is at least this long
OVERLAP_MAX_REACH = 160.0            # anchor pole no further than this past the turnout

# Signals (placed by the railroad signal playlist, not in the tile files): poles, guy wires and
# their anchors keep this far from them.
SIGNAL_PLAYLIST = "data/missions/default_railroad_signals/playlist.xml"
SIGNAL_CLEARANCE = 1.6

# End of a wire run: the last support is an anchor pole - both wires leave the track sideways to
# it, end in insulators, the tension comes from a weight stack on the far side of the pole, and a
# guy wire to a ground anchor behind the pole takes the pull.
# Anchoring (semi-compensated, as drawn by the user from a real one): the messenger is anchored
# straight to the pole top through an insulator; the contact wire is tensioned by weights through
# a two-stage pulley block (1:4): the wire (through an insulator) pulls the outer moving block,
# whose rope (one end on the pole) pulls the axle of the inner moving block, whose rope (one end
# on the pole) runs back over a pulley on a rigid lever at the pole and down to the weights.
# All pulleys are the same size. Two guy ropes (from the messenger and the compensator height)
# loop round the pole, meet above the ground and go on as one anchor rod to the ground block.
# Heights above the rail top, distances out from the pole axis along the anchored wires.
MESSENGER_ANCHOR_HEIGHT = 8.40
COMPENSATOR_HEIGHT = 7.00            # the anchored contact wire rises to here, out of reach
PULLEY_RADIUS = 0.13
INNER_PULLEY_OUT = 2.00
OUTER_PULLEY_OUT = 3.60
CONTACT_INSULATOR = (3.90, 4.60)     # insulator on the rope between the outer block and the wire
CONTACT_END_OUT = 4.80               # the contact wire ends here (clamp)
MESSENGER_INSULATOR = (1.20, 1.90)
MESSENGER_END_OUT = 2.10
LEVER_DROP = 0.70                    # the lever pulley is this much below the compensator
WEIGHTS_OUT = 0.60                   # weights hang this far out (span side), below the lever pulley
WEIGHT_STACK = (0.15, 0.09, 7)       # stack of concrete discs: radius, disc thickness, discs;
                                     # two stacks side by side
WEIGHT_STACK_GAP = 0.04
WEIGHTS_BOTTOM = 1.00                # above the rail
ANCHOR_SHEDS = 4                     # insulator discs on the anchored wires
GUY_SPLIT = 0.90                     # the guy rope splits into two strands this far from the pole
GUY_ROD_LENGTH = 1.60                # steel anchor rod above the ground block
GUY_DISTANCE = 7.0                   # ground anchor this far behind the pole
GUY_CLEARANCE = 2.6                  # ground anchor from any track axis, at least
GUY_RADIUS = 0.014
GUY_ROD_RADIUS = 0.030
GUY_BLOCK = (0.6, 0.35)              # ground anchor block: side, height above the ground
COLOR_WEIGHTS = (88, 88, 86, 255)

POLE_CLEARANCE = 3.10                # from the pole centre to any other track axis, at least
PORTAL_GAP = POLE_OFFSET + POLE_CLEARANCE   # free room beside a track needed for a pole
PORTAL_REACH = 30.0                  # how far across a bundle of tracks a portal may reach
PORTAL_ROUNDS = 6
PORTAL_SEARCH_STEP = 3.0             # step when looking for a spot with room for a pole
PORTAL_REUSE_AHEAD = 15.0
PORTAL_STRETCH = 1.2                 # a span may stretch this much to hang from an existing portal
PORTAL_BEAM_HEIGHT = 9.30            # beam centre above the rail top
PORTAL_BEAM_SIZE = (0.40, 0.55)      # width, height

# bridges, ledges
FOOTING_LOW = 3.0                    # ground at most this far below the rail: an ordinary pole
FOOTING_HIGH = 3.0
FOOTING_LOW_FAR = 6.0                # or this far (tall foundation) / higher, if nothing better
DECK_EDGE_SEARCH = (3.0, 2.7, 2.4, 2.1, 1.8)   # a viaduct deck edge, from the track axis
DECK_LEVEL = (-1.6, 0.3)             # deck surface around the rail top
BRIDGE_GAP = 4.0                     # something this far under the deck: a bridge
BRIDGE_SIDE = 8.0                    # ... or no ground this far to either side (past the deck)
BRACKET_BEAM = (0.50, 0.35)          # steel console under a pole on a viaduct: width, height
BRACKET_DROP = 1.80                  # its strut meets the deck side this far below the beam
TRUSS_COLOR = (45, 34, 32)           # the steel of the through-truss bridges in the tile meshes
TRUSS_TOP = (9.0, 14.0)              # truss top members above the rail
TRUSS_WALL = (2.0, 14.0)             # any truss member (not the deck girders)
TRUSS_REACH = 12.0                   # lateral, from the track axis
TRUSS_STEP = 2.0
TRUSS_BEAM_SIZE = (0.25, 0.30)       # cross girder under the top chords: width, height
COLOR_TRUSS = (45, 34, 32, 255)

# anchor sections: overlaps (3-span non-insulating on the line, 4-span insulating at the
# station limits) and the middle anchor
SECTION_MAX = 1600.0
SECTION_CURVED_CUT = 300.0           # shorter by up to this where the section lies on curves
SECTION_CURVE_RADIUS = 1500.0
SECTION_SEARCH = 300.0               # an overlap may move this far from its ideal place
OVERLAP_GAP = 0.10                   # non-insulating overlap: wires this far apart
INSULATING_GAP = 0.55                # insulating overlap: contact wires this far apart (middle)
STATION_TURNOUT_GAP = 400.0          # turnouts closer than this along a track: one station
STATION_LINE_BEYOND = 500.0          # the line goes on at least this far beyond the station
STATION_OVERLAP_RANGE = (40.0, 350.0)   # insulating overlap centre beyond the outer turnout
STATION_OVERLAP_TARGET = 120.0
MIDDLE_ANCHOR_MIN = 500.0            # sections at least this long get a middle anchor
MIDDLE_ANCHOR_REACH = 0.30           # its rope ends on the messenger this far (x span) from mid
OVERLAP_INSULATOR_AT = (1.6, 2.4)    # insulating overlap: insulators in the anchored branch,
                                     # this far from the transition support


class Trusses:
    """Steel of the through-truss bridges, found by its colour in the tile meshes."""

    CELL = 10.0

    def __init__(self, rom_dir):
        self.rom_dir = rom_dir
        self.tiles = {}

    def _load(self, tile):
        if tile in self.tiles:
            return self.tiles[tile]
        grid = {}
        origin = signs.mainland_origin(tile)
        path = os.path.join(self.rom_dir, "data", "tiles", tile + ".xml")
        if origin is not None and os.path.exists(path):
            text = open(path, encoding="utf-8", errors="replace").read()
            for m in re.finditer(r"<mesh ([^>]*)>\s*<transform([^/]*)/>", text):
                attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
                if "parent_id" in attrs or "file_name" not in attrs:
                    continue
                mesh_path = os.path.join(self.rom_dir, *attrs["file_name"].split("/"))
                if not os.path.exists(mesh_path):
                    continue
                matrix = swmesh.parse_transform(m.group(2))
                for v in swmesh.read_mesh_full(mesh_path).vertices:
                    if (int(v[3]), int(v[4]), int(v[5])) != TRUSS_COLOR:
                        continue
                    x, y, z = signs._transform((v[0], v[1], v[2]), matrix)
                    x, z = x + origin[0], z + origin[1]
                    grid.setdefault((int(math.floor(x / self.CELL)), int(math.floor(z / self.CELL))),
                                    []).append((x, y, z))
        self.tiles[tile] = grid
        return grid

    def near(self, x, z, radius):
        found = []
        for cx in range(int(math.floor((x - radius) / self.CELL)), int(math.floor((x + radius) / self.CELL)) + 1):
            for cz in range(int(math.floor((z - radius) / self.CELL)), int(math.floor((z + radius) / self.CELL)) + 1):
                tile = "mega_island_%d_%d" % (int(math.floor((cx * self.CELL + 5 + 8500) / 1000)),
                                              int(math.floor((cz * self.CELL + 5 + 12500) / 1000)))
                found.extend(self._load(tile).get((cx, cz), ()))
        return found

    def at(self, p, t, r, along, heights):
        """Truss points near the track point p as (along, lateral, height above the rail)."""
        out = []
        for x, y, z in self.near(p[0], p[2], TRUSS_REACH + along):
            dx, dz = x - p[0], z - p[2]
            a = dx * t[0] + dz * t[1]
            b = dx * r[0] + dz * r[1]
            h = y - p[1]
            if abs(a) <= along and abs(b) <= TRUSS_REACH and heights[0] <= h <= heights[1]:
                out.append((a, b, h))
        return out


def _truss_zones(trusses, points, cumulative, first, last):
    """[near_from, near_to, top_from, top_to] along a chain: where truss members stand beside
    the track (no pole), and where its top members run over it (wires hang from them)."""
    zones = []
    current = None
    s = max(0.0, first - 20.0)
    end = min(cumulative[-1], last + 20.0)
    while s <= end:
        p, t = _polyline_at(points, cumulative, s)
        r = signs.right_of(t)
        # the truss members have vertices only at the panel points: look a few metres along
        if trusses.at(p, t, r, 5.0, TRUSS_WALL):
            if current is None or s - current[1] > 20.0:
                current = [s, s, None, None]
                zones.append(current)
            current[1] = s
            if trusses.at(p, t, r, 5.0, TRUSS_TOP):
                if current[2] is None:
                    current[2] = s
                current[3] = s
        s += TRUSS_STEP
    return zones


def _make_truss_portal(trusses, p, t, r):
    """Cross girder under the truss top chords at track point p (a portal with no poles)."""
    top = trusses.at(p, t, r, 8.0, TRUSS_TOP)
    if len(top) < 2:
        return None
    height = min(h for _, _, h in top)
    lo = min(b for _, b, _ in top)
    hi = max(b for _, b, _ in top)
    if hi < 1.0 or lo > -1.0:
        return None
    poles = [[p[0] + r[0] * hi, p[1], p[2] + r[1] * hi], [p[0] + r[0] * lo, p[1], p[2] + r[1] * lo]]
    centre = [(poles[0][0] + poles[1][0]) / 2, p[1], (poles[0][2] + poles[1][2]) / 2]
    return {"poles": poles, "rail_y": p[1], "centre": centre, "kind": "truss",
            "beam_y": p[1] + height - TRUSS_BEAM_SIZE[1] / 2}


def _on_bridge(ground, point, t=None):
    """Is the track at `point` on a bridge: something far under the deck, or (with the track
    direction t) no ground near the rail level on either side?"""
    heights = ground.heights(point[0], point[2])
    deck = [h for h in heights if point[1] - 1.5 <= h <= point[1] + 0.5]
    if not deck:
        return False
    top = max(deck)
    if any(h < top - BRIDGE_GAP for h in heights):
        return True
    if t is None:
        return False
    r = signs.right_of(tuple(t))
    for side in (1, -1):
        beside = ground.heights(point[0] + r[0] * side * BRIDGE_SIDE, point[2] + r[1] * side * BRIDGE_SIDE)
        if any(point[1] - FOOTING_LOW_FAR <= h <= point[1] + FOOTING_HIGH for h in beside):
            return False
    return True


def load_signals(rom_dir):
    """World (x, z) of the railroad signals on the mainland."""
    path = os.path.join(rom_dir, *SIGNAL_PLAYLIST.split("/"))
    if not os.path.exists(path):
        return []
    text = open(path, encoding="utf-8", errors="replace").read()
    found = []
    for location in re.finditer(r'<l id="\d+" tile="data/tiles/([^"]+)\.xml".*?</l>', text, re.S):
        origin = signs.mainland_origin(location.group(1))
        if origin is None:
            continue
        for zone in re.finditer(r'cargo_zone_import_cargo_type="railroad_signal[^"]*".*?<spawn_transform([^/]*)/>',
                                location.group(0), re.S):
            values = dict(re.findall(r'(\d\d)="([^"]*)"', zone.group(1)))
            found.append((float(values.get("30", 0)) + origin[0], float(values.get("32", 0)) + origin[1]))
    return found


def _near_signal(net, a, b=None):
    """Is the point a (or the segment a-b), as (x, z), closer than SIGNAL_CLEARANCE to a signal?"""
    for signal in net.signals:
        d = (math.dist(signal, a) if b is None else signs._point_segment_distance(signal, a, b))
        if d < SIGNAL_CLEARANCE:
            return True
    return False


def _nearest_node(net, point):
    return min(net.nodes_world, key=lambda k: math.hypot(net.nodes_world[k][0] - point[0],
                                                         net.nodes_world[k][2] - point[1]))


def _route(net, start, end):
    """Points of the shortest track path between the nodes nearest to `start` and `end`."""
    source, target = _nearest_node(net, start), _nearest_node(net, end)
    best = {source: 0.0}
    previous = {}
    heap = [(0.0, source)]
    done = set()
    while heap:
        d, key = heapq.heappop(heap)
        if key in done:
            continue
        done.add(key)
        if key == target:
            break
        for o in net.links[key]:
            nd = d + math.dist(net.nodes_world[key], net.nodes_world[o])
            if nd < best.get(o, 1e18):
                best[o] = nd
                previous[o] = key
                heapq.heappush(heap, (nd, o))
    path = [target]
    while path[-1] != source:
        path.append(previous[path[-1]])
    return [net.nodes_world[k] for k in reversed(path)]


def plan_catenary(rom_dir, tiles, prepare_graph, start=ROUTE_START, end=ROUTE_END,
                  corridor=ROUTE_CORRIDOR, log=print):
    """Return {"supports": [...], "spans": [...], "portals": [...]} (world coordinates). Each
    item carries the tile whose meshes get its geometry."""
    net = signs.build_network(tiles, prepare_graph)
    net.signals = load_signals(rom_dir)
    if ELECTRIFY_ALL:
        route = []
    else:
        route = _route(net, start, end)
        # the route runs exactly from `start` to `end`, not just between the nearest nodes
        route = [(start[0], 0.0, start[1])] + route + [(end[0], 0.0, end[1])]
    route_index = signs.SegmentIndex()
    for a, b in zip(route, route[1:]):
        route_index.add((a[0], a[2]), (b[0], b[2]), None)

    def near_route(p):
        if ELECTRIFY_ALL:
            return True
        return any(signs._point_segment_distance((p[0], p[2]), sa, sb) <= corridor
                   for sa, sb, _ in route_index.near(p[0], p[2], corridor))

    def edge_near_route(a, b):
        pa, pb = net.nodes_world[a], net.nodes_world[b]
        pieces = max(1, int(math.dist(pa, pb) // COVER_SAMPLE_STEP))
        return any(near_route(_lerp(pa, pb, i / pieces)) for i in range(pieces + 1))

    ground = signs.Ground(rom_dir)
    trusses = Trusses(rom_dir)

    def covered(q):
        """Something overhead (a station roof, a footbridge) where the wires would run."""
        return any(q[1] + COVER_LOW < h < q[1] + COVER_HIGH for h in ground.heights(q[0], q[2]))

    electrified = {(a, b) for a in net.links for b in net.links[a]
                   if a < b and edge_near_route(a, b)}
    sub_links = {}
    for a, b in sorted(electrified):            # sorted: the same plan on every run
        sub_links.setdefault(a, []).append(b)
        sub_links.setdefault(b, []).append(a)
    sub_nodes = {k: (net.nodes_world[k], v) for k, v in sub_links.items()}
    chains = rails.build_chains(sub_nodes)

    # wire runs: the stretches of every chain near the route and not covered (the line ends a
    # little before a roof)
    chain_data = []
    for chain, _ in chains:
        points = [net.nodes_world[k] for k in chain]
        cumulative = [0.0]
        for a, b in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + math.dist((a[0], a[2]), (b[0], b[2])))
        total = cumulative[-1]
        pieces = max(1, int(total // COVER_SAMPLE_STEP))
        open_from = 0.0
        runs = []
        for i in range(pieces + 1):
            s = total * i / pieces
            q = _polyline_at(points, cumulative, s)[0]
            if covered(q) or not near_route(q):
                if open_from is not None:
                    runs.append((open_from, s - COVER_MARGIN))
                open_from = None
            elif open_from is None:
                open_from = s + COVER_MARGIN
        if open_from is not None:
            runs.append((open_from, total))
        for run_start, run_end in runs:
            if run_end - run_start >= MIN_RUN_LENGTH:
                chain_data.append((chain, points, cumulative, run_start + END_GAP, run_end - END_GAP))

    # Supports go along each track as far apart as the curve allows. Where there is no room for a
    # pole on either side, a portal spans the whole bundle of tracks. Every electrified track that
    # passes under a portal hangs from it, so the planning repeats until no new portal appears.
    truss_zones = [_truss_zones(trusses, points, cumulative, first, last)
                   for chain, points, cumulative, first, last in chain_data]
    portals = []
    for round_index in range(PORTAL_ROUNDS):
        count = len(portals)
        planned = [_space_supports(ground, net, points, cumulative, first, last, portals,
                                   trusses, zones)
                   for (chain, points, cumulative, first, last), zones in zip(chain_data, truss_zones)]
        if len(portals) == count:
            break
    used = {item["portal"] for items in planned for item in items if item["portal"] is not None}
    renumber = {}
    for old_id in sorted(used):
        renumber[old_id] = len(renumber)
    portals = [dict(portals[i], id=renumber[i]) for i in sorted(used)]
    for items in planned:
        for item in items:
            if item["portal"] is not None:
                item["portal"] = renumber[item["portal"]]

    supports, spans = [], []
    runs = []                     # per wire run: support indices in order
    blocked = 0
    for run_index, ((chain, points, cumulative, _, _), items) in enumerate(zip(chain_data, planned)):
        indices = []
        for item in items:
            if item["portal"] is None and item["side"] is None:
                blocked += 1          # no pole, no portal: the wires run on to the next support
                continue
            support = _place_support(ground, item, len(indices))
            support["seq"] = len(indices)
            support["tile"] = _chain_tile(net, chain, cumulative, item["s"])
            support["s"] = item["s"]
            support["run"] = run_index
            indices.append(len(supports))
            supports.append(support)
        runs.append(indices)

    edge_ends = set()
    for run, (chain, points, cumulative, first, last) in enumerate(chain_data):
        if _leaves_mainland(net, chain[0]):
            edge_ends.add((run, False))
        if _leaves_mainland(net, chain[-1]):
            edge_ends.add((run, True))
    trimmed = _trim_bridge_ends(ground, runs, supports, portals, edge_ends)
    shared = _share_poles(net, chain_data, runs, supports)
    overlaps = _extend_over_turnouts(net, chain_data, runs, supports)
    sections = _split_sections(ground, net, chain_data, runs, supports)
    own_anchor_poles = _anchor_poles(ground, net, runs, supports, portals)
    for run, indices in enumerate(runs):
        for a, b in zip(indices, indices[1:]):
            span = {"tile": supports[a]["tile"], "from": a, "to": b, "run": run}
            if supports[b].get("insulate") and supports[a].get("section_anchor"):
                span["insulators"] = "to"
            elif supports[a].get("insulate") and supports[b].get("section_anchor"):
                span["insulators"] = "from"
            spans.append(span)
    anchors = _mark_anchors(ground, net, runs, supports)
    _assign_stagger(runs, supports)
    middle_anchors = _middle_anchors(runs, supports, spans)
    live = {sp["portal"] for sp in supports if sp["portal"] is not None and not sp.get("dropped")}
    for portal in portals:
        portal["tile"] = _tile_of_point(net, portal["centre"])
        if portal["id"] not in live:
            portal["unused"] = True           # its supports were trimmed off
    if FEEDERS:
        feeder_poles, feeder_spans = _plan_feeders(net, route, chain_data, supports, portals)
    else:
        feeder_poles, feeder_spans = [], []
    log("catenary: route nodes %d, electrified edges %d, wire runs %d, sections %d, supports %d "
        "(on portals %d, on truss girders %d, on viaduct brackets %d, on shared poles %d, anchors %d "
        "(moved to own poles %d), left out %d, trimmed off bridge ends %d), turnout overlaps %d, "
        "section overlaps %d plain + %d insulating, middle anchors %d, spans %d, portals %d "
        "(truss girders %d), DPR spans %d"
        % (len(route), len(electrified), len(chain_data), len([r for r in runs if len(r) >= 2]), len(supports),
           sum(1 for sp in supports if sp["portal"] is not None and portals[sp["portal"]].get("kind") != "truss"),
           sum(1 for sp in supports if sp["portal"] is not None and portals[sp["portal"]].get("kind") == "truss"),
           sum(1 for sp in supports if sp.get("bracket") is not None and not sp.get("dropped")),
           shared, anchors, own_anchor_poles, blocked, trimmed,
           overlaps, sections["plain"], sections["insulating"], middle_anchors, len(spans), len(portals),
           sum(1 for portal in portals if portal.get("kind") == "truss"), len(feeder_spans)))
    return {"supports": supports, "spans": spans, "portals": portals,
            "feeder_poles": feeder_poles, "feeder_spans": feeder_spans}


def _chain_tile(net, chain, cumulative, s):
    for i in range(len(cumulative) - 1):
        if cumulative[i + 1] >= s:
            return net.edge_tile[(chain[i], chain[i + 1])]
    return net.edge_tile[(chain[-2], chain[-1])]


def _closest_on_chain(points, cumulative, x, z):
    """(arc length, distance) of the point of a chain closest to (x, z)."""
    best = None
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        dx, dz = b[0] - a[0], b[2] - a[2]
        length2 = dx * dx + dz * dz
        f = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - a[0]) * dx + (z - a[2]) * dz) / length2))
        d = math.hypot(a[0] + dx * f - x, a[2] + dz * f - z)
        if best is None or d < best[1]:
            best = (cumulative[i] + f * (cumulative[i + 1] - cumulative[i]), d)
    return best


def _share_poles(net, chain_data, runs, supports):
    """Supports of different tracks close together use one pole (see SHARE_SHIFT). Returns the
    number of supports moved onto another pole."""
    owners = []                   # indices of supports that own their pole
    grid = {}
    users = {}                    # owner -> number of other supports on its pole
    moved = 0
    for indices in runs:
        for position, index in enumerate(indices):
            sp = supports[index]
            if sp["portal"] is not None:
                continue
            chain, points, cumulative, first, last = chain_data[sp["run"]]
            at_end = position == 0 or position == len(indices) - 1
            limit = SHARE_SHIFT_AT_END if at_end else SHARE_SHIFT
            low = supports[indices[position - 1]]["s"] + SHARE_KEEP_SPAN if position > 0 else first - END_GAP + 1.0
            high = (supports[indices[position + 1]]["s"] - SHARE_KEEP_SPAN
                    if position + 1 < len(indices) else last + END_GAP - 1.0)
            best = None
            cx, cz = int(sp["pole"][0] // 20), int(sp["pole"][2] // 20)
            for gx in (cx - 1, cx, cx + 1):
                for gz in (cz - 1, cz, cz + 1):
                    for owner in grid.get((gx, gz), ()):
                        other = supports[owner]
                        if other["run"] == sp["run"] or users.get(owner, 0) >= 1:
                            continue
                        px, pz = other["pole"][0], other["pole"][2]
                        if math.hypot(px - sp["pole"][0], pz - sp["pole"][2]) > limit + 6.0:
                            continue
                        s_new, reach = _closest_on_chain(points, cumulative, px, pz)
                        if not (SHARE_REACH[0] <= reach <= SHARE_REACH[1]):
                            continue
                        if abs(s_new - sp["s"]) > limit or not (low <= s_new <= high):
                            continue
                        p, t = _polyline_at(points, cumulative, s_new)
                        r = signs.right_of(t)
                        side = (px - p[0]) * r[0] + (pz - p[2]) * r[1]
                        if side * sp["side"] <= 0:
                            continue             # the pole is on the other side of this track
                        if _crossings(net, p, _unit2((px - p[0], pz - p[2])), reach - 0.5):
                            continue             # another track runs between them
                        # tracks this close (at turnouts) share one cantilever: the second wire
                        # hangs from the first one's bracket and fixator ("rider")
                        rider = math.hypot(other["track"][0] - p[0], other["track"][2] - p[2]) < SHARE_MIN_TRACK_GAP
                        if best is None or abs(s_new - sp["s"]) < best[0]:
                            best = (abs(s_new - sp["s"]), owner, s_new, p, t, rider)
            if best is None:
                owners.append(index)
                grid.setdefault((cx, cz), []).append(index)
                continue
            _, owner, s_new, p, t, rider = best
            users[owner] = users.get(owner, 0) + 1
            sp["rider"] = rider
            sp.update(s=s_new, track=[p[0], p[1], p[2]], t=[t[0], t[1]],
                      pole=list(supports[owner]["pole"]), pole_owner=owner,
                      pole_base=supports[owner]["pole_base"])
            moved += 1
    return moved


def _extend_over_turnouts(net, chain_data, runs, supports):
    """A wire run that ends at a turnout of another electrified track goes on over that track:
    one more support on the main track's pole (second cantilever), then the anchor on the next
    pole. Returns the number of runs extended."""
    passing = {}                  # node -> (run, arc length) of runs going through it
    for run, (chain, points, cumulative, first, last) in enumerate(chain_data):
        for i in range(1, len(chain) - 1):
            if first - END_GAP <= cumulative[i] <= last + END_GAP:
                passing.setdefault(chain[i], []).append((run, cumulative[i]))
    extended = 0
    used_middles = set()          # a main-track support carries at most one overlap wire
    for run, (chain, points, cumulative, first, last) in enumerate(chain_data):
        for at_start in (True, False):
            indices = runs[run]
            if len(indices) < 2:
                continue
            if at_start:
                if first - END_GAP > 0.5:
                    continue               # the run starts before a roof, not at a turnout
                node, before = chain[0], points[1]
                end_support = supports[indices[0]]
            else:
                if last + END_GAP < cumulative[-1] - 0.5:
                    continue
                node, before = chain[-1], points[-2]
                end_support = supports[indices[-1]]
            through = [(r, s) for r, s in passing.get(node, ()) if r != run]
            if not through:
                continue
            main_run, s_node = through[0]
            main_chain, main_points, main_cumulative, _, _ = chain_data[main_run]
            node_p, t = _polyline_at(main_points, main_cumulative, s_node)
            into = _unit2((node_p[0] - before[0], node_p[2] - before[2]))   # side track -> turnout
            ahead = 1.0 if into[0] * t[0] + into[1] * t[1] > 0 else -1.0
            r = signs.right_of(t)
            offset_side = 1.0 if (before[0] - node_p[0]) * r[0] + (before[2] - node_p[2]) * r[1] > 0 else -1.0
            main = runs[main_run]
            beyond = sorted((i for i in main if (supports[i]["s"] - s_node) * ahead > 0),
                            key=lambda i: (supports[i]["s"] - s_node) * ahead)
            beyond = [i for i in beyond
                      if math.dist((supports[i]["track"][0], supports[i]["track"][2]),
                                   (end_support["track"][0], end_support["track"][2])) >= OVERLAP_FIRST_SPAN]
            beyond = [i for i in beyond if i not in used_middles
                      and not any(o.get("pole_owner") == i for o in supports)]   # nor one sharing its pole
            if len(beyond) < 2:
                continue
            middle, anchor = beyond[0], beyond[1]
            if (supports[anchor]["portal"] is not None or anchor in (main[0], main[-1])
                    or abs(supports[anchor]["s"] - s_node) > OVERLAP_MAX_REACH):
                continue
            used_middles.add(middle)
            added = []
            for source in (middle, anchor):
                copy = dict(supports[source])
                copy["run"] = run
                copy["overlap_of"] = source
                copy["overlap_side"] = offset_side
                if copy["portal"] is None:
                    owner = supports[source].get("pole_owner")
                    copy["pole_owner"] = source if owner is None else owner
                copy.pop("anchor", None)
                added.append(len(supports))
                supports.append(copy)
            # the side track's support right at the turnout is no longer needed: its wire runs on
            # to the main track's support
            end_index = indices[0] if at_start else indices[-1]
            from_turnout = end_support["s"] if at_start else cumulative[-1] - end_support["s"]
            neighbour = supports[indices[1] if at_start else indices[-2]]
            reach = math.dist((neighbour["track"][0], neighbour["track"][2]),
                              (supports[middle]["track"][0], supports[middle]["track"][2]))
            if (len(indices) > 2 and from_turnout <= JUNCTION_DROP and reach <= MAX_SPAN * 1.25
                    and end_support["portal"] is None
                    and not any(o.get("pole_owner") == end_index for o in supports)):
                end_support["dropped"] = True
                indices = indices[1:] if at_start else indices[:-1]
            if at_start:
                runs[run] = added[::-1] + indices
            else:
                runs[run] = indices + added
            extended += 1
    return extended


def _leaves_mainland(net, key):
    """Does the track end at this node because it runs on past the mainland tiles?"""
    if len(net.links.get(key, ())) != 1:
        return False
    x, _, z = net.nodes_world[key]
    ox, oz = signs.mainland_origin(key[0])
    if max(abs(x - ox), abs(z - oz)) < signs.TILE_HALF - 0.6:
        return False
    beyond = _tile_of_point(net, (x + (x - ox) * 0.01, 0.0, z + (z - oz) * 0.01))
    return beyond not in {k[0] for k in net.nodes_world}


def _trim_bridge_ends(ground, runs, supports, portals, edge_ends=None):
    """A wire run does not end on a bridge (where the line leaves the mainland over a trestle,
    the wires stop on land before it): its end supports are taken off while they hang from a
    truss, stand on a viaduct console or the track there is on a bridge. Returns the number of
    supports taken off."""
    trimmed = 0

    def off_land(sp):
        if sp["portal"] is not None:
            return portals[sp["portal"]].get("kind") == "truss"
        return sp.get("bracket") is not None or _on_bridge(ground, sp["track"], sp["t"])

    for k, indices in enumerate(runs):
        if indices and all(off_land(supports[i]) for i in indices):
            for i in indices:                # a short run out on a trestle
                supports[i]["dropped"] = True
            trimmed += len(indices)
            runs[k] = []
            continue
        for at_end in (True, False):
            if edge_ends is not None and (k, at_end) not in edge_ends:
                continue                     # only where the line runs off the mainland
            while indices:
                i = indices[-1] if at_end else indices[0]
                if not off_land(supports[i]):
                    break
                supports[i]["dropped"] = True
                indices = indices[:-1] if at_end else indices[1:]
                trimmed += 1
        if len(indices) < 2:
            for i in indices:
                supports[i]["dropped"] = True
            indices = []
        runs[k] = indices
    return trimmed


def _split_sections(ground, net, chain_data, runs, supports):
    """Cut long wire runs into anchor sections (SECTION_MAX, shorter on curves). Where two
    sections meet, a 3-span non-insulating overlap: supports 1-4, section A works at 1 and 2,
    is raised at 3 and anchored at 4; section B is anchored at 1, raised at 2, works from 3; in
    the transition span 2-3 the wires run side by side OVERLAP_GAP apart. Beyond each end of a
    station (a run of turnouts) a 4-span insulating overlap instead: supports 1-5, both wires work
    at the middle support 3, INSULATING_GAP apart, and the anchored branches carry insulators."""
    counts = {"plain": 0, "insulating": 0}
    sharers = {o["pole_owner"] for o in supports if o.get("pole_owner") is not None and not o.get("dropped")}
    sources = {o["overlap_of"] for o in supports if o.get("overlap_of") is not None}

    def eligible(i):
        sp = supports[i]
        return (sp["portal"] is None and not sp.get("dropped") and sp.get("bracket") is None
                and sp.get("pole_owner") is None and i not in sharers
                and sp.get("overlap_of") is None and i not in sources
                and not _on_bridge(ground, sp["track"], sp["t"]))

    def copy_of(src, **fields):
        sp = dict(supports[src])
        sp.pop("anchor", None)
        sp.pop("dropped", None)
        pole = sp["pole"]
        r = signs.right_of(tuple(sp["t"]))
        pole_side = 1.0 if ((pole[0] - sp["track"][0]) * r[0] + (pole[2] - sp["track"][2]) * r[1]) > 0 else -1.0
        sp.update(overlap_of=src, overlap_side=-pole_side, pole_owner=src, **fields)
        supports.append(sp)
        return len(supports) - 1

    for run in range(len(runs)):
        R = runs[run]
        if len(R) < 10:
            continue
        chain, points, cumulative, first, last = chain_data[run]
        core = [q for q in range(len(R)) if supports[R[q]].get("overlap_of") is None]
        if len(core) < 10:
            continue
        s_lo, s_hi = supports[R[core[0]]]["s"], supports[R[core[-1]]]["s"]

        def s_at(q):
            return supports[R[q]]["s"]

        taken = set()
        cuts = []

        def place(target, n, lo, hi):
            best = None
            for q in range(2, len(R) - n - 1):
                if any(p in taken for p in range(q - 2, q + n + 2)):
                    continue
                if not all(eligible(R[p]) for p in range(q, q + n)):
                    continue
                mid = (s_at(q) + s_at(q + n - 1)) / 2
                if lo <= mid <= hi and (best is None or abs(mid - target) < best[0]):
                    best = (abs(mid - target), q)
            if best is None:
                return False
            q = best[1]
            cuts.append((q, n))
            taken.update(range(q, q + n))
            return True

        # stations: turnouts on this track, grouped
        turnouts = [cumulative[i] for i in range(1, len(chain) - 1) if len(net.links.get(chain[i], ())) >= 3]
        stations = []
        for s in turnouts:
            if stations and s - stations[-1][1] < STATION_TURNOUT_GAP:
                stations[-1][1] = s
                stations[-1][2] += 1
            else:
                stations.append([s, s, 1])
        for a, b, count in stations:
            if count < 2:
                continue
            if a - s_lo >= STATION_LINE_BEYOND:
                if place(a - STATION_OVERLAP_TARGET, 5, a - STATION_OVERLAP_RANGE[1], a - STATION_OVERLAP_RANGE[0]):
                    counts["insulating"] += 1
            if s_hi - b >= STATION_LINE_BEYOND:
                if place(b + STATION_OVERLAP_TARGET, 5, b + STATION_OVERLAP_RANGE[0], b + STATION_OVERLAP_RANGE[1]):
                    counts["insulating"] += 1
        # the stretches between: sections no longer than allowed
        bounds = sorted([s_lo] + [(s_at(q) + s_at(q + n - 1)) / 2 for q, n in cuts] + [s_hi])
        for x0, x1 in zip(bounds, bounds[1:]):
            inside = [supports[i] for i in R if x0 <= supports[i]["s"] <= x1 and supports[i].get("overlap_of") is None]
            curved = sum(1 for sp in inside if sp.get("radius", 1e9) < SECTION_CURVE_RADIUS) / max(1, len(inside))
            limit = SECTION_MAX - SECTION_CURVED_CUT * curved
            if x1 - x0 <= limit:
                continue
            pieces = int(math.ceil((x1 - x0) / limit))
            for k in range(1, pieces):
                target = x0 + (x1 - x0) * k / pieces
                if place(target, 4, target - SECTION_SEARCH, target + SECTION_SEARCH):
                    counts["plain"] += 1
        if not cuts:
            continue
        cuts.sort()
        sections = []
        prefix = []
        start = 0
        for q, n in cuts:
            ids = [R[q + k] for k in range(n)]
            keep = n - 2                       # supports of this window section A works at
            insulating = n == 5
            a_part = prefix + R[start:q + keep]
            a_part.append(copy_of(ids[n - 2], insulate=insulating))
            a_part.append(copy_of(ids[n - 1], section_anchor=True))
            sections.append(a_part)
            prefix = [copy_of(ids[0], section_anchor=True), copy_of(ids[1], insulate=insulating)]
            if insulating:
                prefix.append(copy_of(ids[2], insulating_mid=True, overlap_gap=INSULATING_GAP, overlap_raise=0.0))
            start = q + keep
        sections.append(prefix + R[start:])
        runs[run] = sections[0]
        for section in sections[1:]:
            new_run = len(runs)
            runs.append(section)
            for i in section:
                supports[i]["run"] = new_run
    return counts


def _middle_anchors(runs, supports, spans):
    """Middle anchor of the contact wire in the middle span of every long section."""
    by_pair = {(span["from"], span["to"]): span for span in spans}
    count = 0
    for indices in runs:
        plain = [i for i in indices if supports[i].get("overlap_of") is None and supports[i].get("anchor") is None]
        if len(plain) < 6:
            continue
        length = supports[plain[-1]]["s"] - supports[plain[0]]["s"]
        if abs(length) < MIDDLE_ANCHOR_MIN:
            continue
        middle = (supports[plain[0]]["s"] + supports[plain[-1]]["s"]) / 2
        best = None
        for a, b in zip(indices, indices[1:]):
            if a not in plain or b not in plain or (a, b) not in by_pair:
                continue
            d = abs((supports[a]["s"] + supports[b]["s"]) / 2 - middle)
            if best is None or d < best[0]:
                best = (d, (a, b))
        if best is not None:
            by_pair[best[1]]["middle_anchor"] = True
            count += 1
    return count


def _pole_spots(supports, portals):
    spots = [(sp["pole"][0], sp["pole"][2]) for sp in supports
             if sp["portal"] is None and not sp.get("dropped") and sp.get("pole_owner") is None]
    spots += [(p[0], p[2]) for portal in portals for p in portal["poles"]]
    return spots


def _anchor_poles(ground, net, runs, supports, portals):
    """An anchor support on a pole shared with another support gets a pole of its own, a few
    metres beyond on the same side (otherwise the pulley block, weights and guys end up among the
    other cantilevers). Returns the number of anchors moved."""
    moved = 0
    for indices in runs:
        if len(indices) < 2:
            continue
        for end, neighbour in ((indices[0], indices[1]), (indices[-1], indices[-2])):
            sp = supports[end]
            if sp["portal"] is not None:
                continue
            sharers = [i for i, o in enumerate(supports)
                       if o.get("pole_owner") == end and not o.get("dropped")]
            if sp.get("pole_owner") is None and not sharers:
                continue
            track = sp["track"]
            other = supports[neighbour]["track"]
            beyond = _unit2((track[0] - other[0], track[2] - other[2]))
            lateral = (sp["pole"][0] - track[0], sp["pole"][2] - track[2])
            spots = _pole_spots(supports, portals)
            options = [(shift, 1.0) for shift in ANCHOR_POLE_SHIFTS] + [(shift, -1.0) for shift in ANCHOR_POLE_SHIFTS]
            for shift, mirror in options:
                x = track[0] + lateral[0] * mirror + beyond[0] * shift
                z = track[2] + lateral[1] * mirror + beyond[1] * shift
                if _track_distance(net, x, z) < POLE_CLEARANCE or _near_signal(net, (x, z)):
                    continue
                if any(math.hypot(x - a, z - b) < POLE_SPACING for a, b in spots):
                    continue
                rail_y = track[1]
                candidates = [h for h in ground.heights(x, z) if rail_y - FOOTING_LOW <= h <= rail_y + FOOTING_HIGH]
                if not candidates:
                    continue                 # not over a viaduct or a ledge
                if sharers:
                    # the pole stays where it is, owned by the first of the others
                    new_owner = sharers[0]
                    supports[new_owner]["pole_owner"] = None
                    supports[new_owner]["rider"] = False
                    for i in sharers[1:]:
                        supports[i]["pole_owner"] = new_owner
                sp["pole"] = [x, max(candidates), z]
                sp["pole_base"] = rail_y - POLE_BASE_BELOW_RAIL
                sp["pole_owner"] = None
                sp.pop("bracket", None)
                moved += 1
                break
    return moved


def _assign_stagger(runs, supports):
    """Zigzag and fixator type of every working support (see STAGGER)."""
    for indices in runs:
        working = [i for i in indices if supports[i].get("anchor") is None]
        for k, index in enumerate(working):
            sp = supports[index]
            if sp.get("overlap_of") is not None:
                continue
            r = signs.right_of(tuple(sp["t"]))
            pole_side = 0.0
            if sp["portal"] is None:
                pole_side = 1.0 if ((sp["pole"][0] - sp["track"][0]) * r[0] +
                                    (sp["pole"][2] - sp["track"][2]) * r[1]) > 0 else -1.0
            radius = sp.get("radius", 1e9)
            if radius >= ZIGZAG_MIN_RADIUS:
                away = sp.get("seq", k + 1) % 2 == 1
                if pole_side:
                    sp["stagger"] = (-pole_side if away else pole_side) * STAGGER
                else:
                    sp["stagger"] = STAGGER if away else -STAGGER
            else:
                lengths = []
                position = indices.index(index)
                for j in (position - 1, position + 1):
                    if 0 <= j < len(indices):
                        o = supports[indices[j]]["track"]
                        lengths.append(math.hypot(o[0] - sp["track"][0], o[2] - sp["track"][2]))
                span = sum(lengths) / len(lengths) if lengths else MAX_SPAN
                outer = -1.0 if sp.get("turn", 0.0) > 0 else 1.0      # in units of r (right)
                sp["stagger"] = outer * min(CURVE_OFFSET_MAX, span * span / (16.0 * radius))
            sp["fixator"] = "direct" if sp["stagger"] * pole_side > 0 else "reverse"
            if sp.get("rider"):
                sp["fixator"] = "overlap"
                sp["raise"] = SECOND_WIRE_RAISE
        for index in indices:
            sp = supports[index]
            if sp.get("overlap_of") is not None and sp.get("anchor") is None:
                source = supports[sp["overlap_of"]]
                if sp.get("insulating_mid"):
                    # both wires work here, 550 mm apart around the track axis
                    pole_side = -sp["overlap_side"]
                    source["stagger"] = pole_side * INSULATING_GAP / 2
                    source["fixator"] = "direct"
                sp["stagger"] = (source.get("stagger", 0.0)
                                 + sp.get("overlap_gap", OVERLAP_OFFSET) * sp["overlap_side"])
                sp["fixator"] = "overlap"
                sp["raise"] = sp.get("overlap_raise", SECOND_WIRE_RAISE)


def _unit2(v):
    length = math.hypot(v[0], v[1])
    return (v[0] / length, v[1] / length) if length > 1e-9 else (1.0, 0.0)


def _crosses_track(index, a, b):
    """Does the segment a-b (x, z) cross any segment of the index?"""
    length = math.dist(a, b)
    if length < 1e-6:
        return False
    d = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    for sa, sb, _ in index.near(mid[0], mid[1], length / 2 + 1.0):
        if signs._ray_hits_segment(a, d, length, sa, sb) is not None:
            return True
    return False


def _track_distance(net, x, z, radius=10.0):
    return min([signs._point_segment_distance((x, z), sa, sb)
                for sa, sb, _ in net.all_tracks.near(x, z, radius)] + [radius])


def _mark_anchors(ground, net, runs, supports):
    """The first and last support of every wire run with a pole become anchor supports."""
    count = 0
    for indices in runs:
        if len(indices) < 2:
            continue
        for end, neighbour in ((indices[0], indices[1]), (indices[-1], indices[-2])):
            sp = supports[end]
            if sp["portal"] is not None:
                continue
            pole = sp["pole"]
            towards = _unit2((supports[neighbour]["track"][0] - pole[0],
                              supports[neighbour]["track"][2] - pole[2]))
            # the guy goes straight back from the span, or turned away from the track until its
            # ground anchor keeps clear of every track
            # (and of buildings: nothing overhead)
            out = _unit2((pole[0] - sp["track"][0], pole[2] - sp["track"][2]))
            rail_y = sp["track"][1]
            for turn in (0.0, 0.35, 0.7, 1.05, 1.4, -0.35, -0.7):
                back = (-towards[0] * math.cos(turn) + out[0] * math.sin(turn),
                        -towards[1] * math.cos(turn) + out[1] * math.sin(turn))
                gx = pole[0] + back[0] * GUY_DISTANCE
                gz = pole[2] + back[1] * GUY_DISTANCE
                heights = ground.heights(gx, gz)
                candidates = [h for h in heights if rail_y - 8.0 <= h <= rail_y + 4.0]
                floor = max(candidates) if candidates else rail_y - 0.8
                roofed = any(floor + 1.0 < h < floor + 15.0 for h in heights)
                if (_track_distance(net, gx, gz) >= GUY_CLEARANCE and not roofed
                        and not _near_signal(net, (pole[0], pole[2]), (gx, gz))):
                    break
            sp["anchor"] = {"towards": list(towards),
                            "guy_ground": [gx, max(candidates) if candidates else rail_y - 0.8, gz]}
            count += 1
    return count


def _plan_feeders(net, route, chain_data, supports, portals):
    """DPR line on each side of the route: through the outermost poles (normal poles and portal
    posts), the straightest way, as a shortest path over the poles ordered along the route."""
    electrified = signs.SegmentIndex()
    for chain, points, cumulative, first, last in chain_data:
        for i in range(len(points) - 1):
            if cumulative[i + 1] >= first - END_GAP and cumulative[i] <= last + END_GAP:
                electrified.add((points[i][0], points[i][2]), (points[i + 1][0], points[i + 1][2]), None)

    route_cumulative = [0.0]
    for a, b in zip(route, route[1:]):
        route_cumulative.append(route_cumulative[-1] + math.hypot(b[0] - a[0], b[2] - a[2]))

    def along_route(x, z):
        """(arc length along the route, signed distance: + to the right of it)."""
        best = None
        for i in range(len(route) - 1):
            a, b = route[i], route[i + 1]
            dx, dz = b[0] - a[0], b[2] - a[2]
            length2 = dx * dx + dz * dz
            if length2 < 1e-9:
                continue
            f = max(0.0, min(1.0, ((x - a[0]) * dx + (z - a[2]) * dz) / length2))
            qx, qz = a[0] + dx * f, a[2] + dz * f
            d = math.hypot(x - qx, z - qz)
            if best is None or d < best[0]:
                r = signs.right_of(_unit2((dx, dz)))
                side = (x - qx) * r[0] + (z - qz) * r[1]
                best = (d, route_cumulative[i] + f * math.sqrt(length2), side)
        return best[1], best[2]

    poles = []
    for index, sp in enumerate(supports):
        if sp["portal"] is None and sp.get("pole_owner") is None:
            out = _unit2((sp["pole"][0] - sp["track"][0], sp["pole"][2] - sp["track"][2]))
            poles.append({"pos": [sp["pole"][0], sp["pole"][2]], "base_y": sp["pole_base"],
                          "out": list(out), "tile": sp["tile"]})
    for portal in portals:
        for post in portal["poles"]:
            out = _unit2((post[0] - portal["centre"][0], post[2] - portal["centre"][2]))
            poles.append({"pos": [post[0], post[2]], "base_y": portal["rail_y"] - POLE_BASE_BELOW_RAIL,
                          "out": list(out), "tile": portal["tile"]})

    outer = []
    for pole in poles:
        x, z = pole["pos"]
        hits = []
        for sa, sb, _ in electrified.near(x, z, FEEDER_OUTER_CHECK):
            hit = signs._ray_hits_segment((x, z), pole["out"], FEEDER_OUTER_CHECK, sa, sb)
            if hit is not None:
                hits.append(hit)
        if not hits:
            u, v = along_route(x, z)
            pole["u"], pole["v"] = u, v
            outer.append(pole)

    feeder_poles, feeder_spans = [], []
    for side in (1, -1):
        line = sorted((p for p in outer if p["v"] * side > 0), key=lambda p: p["u"])
        # shortest path over the poles in route order (a DAG): span length, plus sideways jumps,
        # minus a bonus for every pole on the way
        cost = []
        previous = []
        for j, pj in enumerate(line):
            best = (0.0, None)
            for i in range(j - 1, -1, -1):
                pi = line[i]
                du = pj["u"] - pi["u"]
                if du > FEEDER_MAX_SPAN:
                    break
                if du < 3.0:
                    continue
                length = math.dist(pi["pos"], pj["pos"])
                if length > FEEDER_MAX_SPAN + 5.0 or _crosses_track(electrified, pi["pos"], pj["pos"]):
                    continue
                c = cost[i] + length + FEEDER_SIDE_PENALTY * abs(pj["v"] - pi["v"]) - FEEDER_POLE_BONUS
                if best[1] is None or c < best[0]:
                    best = (c, i)
            cost.append(best[0])
            previous.append(best[1])
        # walk back from the last pole; where a line starts afresh, go on from the last pole
        # before that start
        j = len(line) - 1
        while j >= 0:
            path = [j]
            while previous[path[-1]] is not None:
                path.append(previous[path[-1]])
            path.reverse()
            for a, b in zip(path, path[1:]):
                for k in (a, b):
                    if "feeder_index" not in line[k]:
                        line[k]["feeder_index"] = len(feeder_poles)
                        feeder_poles.append({key: line[k][key] for key in ("pos", "base_y", "out", "tile")})
                feeder_spans.append({"tile": line[a]["tile"], "from": line[a]["feeder_index"],
                                     "to": line[b]["feeder_index"]})
            start_u = line[path[0]]["u"]
            j = path[0] - 1
            while j >= 0 and line[j]["u"] >= start_u - 3.0:
                j -= 1
    return feeder_poles, feeder_spans


def _span_at(points, cumulative, s):
    """Span length allowed around arc length s (from the curve radius), and the turn there."""
    _, t_ahead = _polyline_at(points, cumulative, s + 30.0)
    _, t_back = _polyline_at(points, cumulative, s - 30.0)
    turn = _turn(t_back, t_ahead)
    radius = 60.0 / abs(turn) if abs(turn) > 1e-4 else 1e9
    span = min(MAX_SPAN, max(MIN_SPAN, math.sqrt(8.0 * radius * CURVE_SAG_ALLOWANCE)))
    return span, radius, turn


def _longest_span(points, cumulative, s):
    """Longest span allowed from s on: the tightest curve within the next MAX_SPAN decides."""
    return min(_span_at(points, cumulative, s + MAX_SPAN * f / 7.0)[0] for f in range(8))


def _space_supports(ground, net, points, cumulative, first, last, portals, trusses=None, zones=()):
    """Supports along one chain. Each next support goes as far as the curve allows (spans evened
    out towards the chain end); it moves back to the nearest spot with room for a pole, or hangs
    from a portal the track passes under, or a new portal is put up. Adds new portals to
    `portals`. The supports go from arc length `first` to `last`."""

    # portals are only ever appended while planning, so each one is checked against this chain
    # once; a grid of the chain's segments finds the few segments near a portal
    cells = {}
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        for cx in range(int(math.floor(min(a[0], b[0]) / CROSS_CELL)), int(math.floor(max(a[0], b[0]) / CROSS_CELL)) + 1):
            for cz in range(int(math.floor(min(a[2], b[2]) / CROSS_CELL)), int(math.floor(max(a[2], b[2]) / CROSS_CELL)) + 1):
                cells.setdefault((cx, cz), []).append(i)
    state = {"seen": 0, "found": []}

    def crossings():
        for portal in portals[state["seen"]:]:
            c = _chain_crossing(points, cumulative, portal, cells)
            if c is not None and first - 2.0 <= c <= last + 2.0:
                state["found"].append((c, portal["id"]))
        state["seen"] = len(portals)
        return sorted(state["found"])

    def in_truss(s, margin=1.0):
        return any(z[0] - margin <= s <= z[1] + margin for z in zones)

    def under_truss_top(s):
        return any(z[2] is not None and z[2] <= s <= z[3] for z in zones)

    def item_at(s, portal):
        p, t = _polyline_at(points, cumulative, s)
        _, radius, turn = _span_at(points, cumulative, s)
        r = signs.right_of(t)
        side, footing = None, None
        if portal is None and not in_truss(s):
            side, footing = _pole_side(net, p, r, ground)
        return {"s": s, "p": p, "t": t, "r": r, "radius": radius, "turn": turn,
                "portal": portal, "side": side, "footing": footing}

    def truss_item(s):
        p, t = _polyline_at(points, cumulative, s)
        portal = _make_truss_portal(trusses, p, t, signs.right_of(t))
        if portal is None:
            return None
        portal["id"] = len(portals)
        portals.append(portal)
        return item_at(s, portal["id"])

    def settle(target, lowest):
        """Support at `target` or, moving back no further than `lowest`, the first spot with
        room for a pole (or under a truss top: a girder to hang from); else a new portal at the
        first spot where one fits; else None."""
        s = target
        while s >= lowest - 1e-6:
            if trusses is not None and under_truss_top(s):
                item = truss_item(s)
                if item is not None:
                    return item
            item = item_at(s, None)
            if item["side"] is not None:
                return item
            s -= PORTAL_SEARCH_STEP
        s = target
        while s >= lowest - 1e-6:
            if in_truss(s, 4.0):
                s -= PORTAL_SEARCH_STEP
                continue
            item = item_at(s, None)
            portal = _make_portal(ground, net, item["p"], item["r"])
            if portal is not None:
                portal["id"] = len(portals)
                portals.append(portal)
                item["portal"] = portal["id"]
                return item
            s -= PORTAL_SEARCH_STEP
        return None

    items = []
    s = first
    under = crossings()
    start_cross = [c for c in under if c[0] <= first + PORTAL_SEARCH_STEP]
    if start_cross:
        item = item_at(max(first, min(last, start_cross[0][0])), start_cross[0][1])
    else:
        item = settle(first + PORTAL_SEARCH_STEP * 2, first) or item_at(first, None)
    items.append(item)
    s = item["s"]
    while s < last - 1.0:
        longest = _longest_span(points, cumulative, s)
        pieces = max(1, int(math.ceil((last - s) / longest - 0.02)))
        target = s + (last - s) / pieces
        # a portal a little further on is taken rather than putting up another one next to it
        reach = min(s + longest * PORTAL_STRETCH, target + PORTAL_REUSE_AHEAD)
        ahead = [c for c in crossings() if s + 3.0 < c[0] <= reach + 1e-6]
        if ahead:
            c, portal_id = ahead[0]
            item = item_at(min(c, last), portal_id)
        else:
            lowest = s + (MIN_SPAN * 0.5 if target - s >= MIN_SPAN * 0.5 else (target - s))
            item = settle(target, lowest)
            if item is None:
                item = item_at(target, None)       # no pole and no portal fits: wires run on
        items.append(item)
        s = item["s"]
    return items


def _tile_of_point(net, p):
    tx = int(math.floor((p[0] + 8000 + 500) / 1000))
    tz = int(math.floor((p[2] + 12000 + 500) / 1000))
    return "mega_island_%d_%d" % (tx, tz)


def _crossings(net, p, d, reach):
    """Distances along the horizontal ray p + s*d (0.5 < s < reach) where it crosses tracks."""
    hits = []
    for sa, sb, _ in net.all_tracks.near(p[0], p[2], reach):
        hit = signs._ray_hits_segment((p[0], p[2]), d, reach, sa, sb)
        if hit is not None and hit > 0.5:
            hits.append(hit)
    return sorted(hits)


def _footing(ground, p, r, side):
    """What a pole on this side would stand on: (rank, deck edge). 3 - ground near the rail
    level; 2 - ground much lower (tall foundation) or higher (a cutting); 1 - nothing there, but a
    viaduct deck whose edge (lateral distance returned) can carry a steel console; 0 - nothing."""
    rail_y = p[1]
    heights = ground.heights(p[0] + r[0] * side * POLE_OFFSET, p[2] + r[1] * side * POLE_OFFSET)
    if any(rail_y - FOOTING_LOW <= h <= rail_y + FOOTING_HIGH for h in heights):
        return 3, None
    if any(rail_y - FOOTING_LOW_FAR <= h <= rail_y + 15.0 for h in heights):
        return 2, None                       # a slope: tall foundation, or the pole stands in a cutting
    # nothing near at the pole spot: a viaduct, if there is a deck edge nearer the track
    for edge in DECK_EDGE_SEARCH:
        deck = ground.heights(p[0] + r[0] * side * edge, p[2] + r[1] * side * edge)
        if any(rail_y + DECK_LEVEL[0] <= h <= rail_y + DECK_LEVEL[1] for h in deck):
            return 1, edge
    return 0, None


def _pole_side(net, p, r, ground=None):
    """(side, footing): +1 / -1, the side (right / left of the track) with room for a pole and
    something to stand on (see _footing), preferring better footing, then more room; (None, None)
    if neither side has PORTAL_GAP free and a footing."""
    best = None
    for side in (1, -1):
        hits = _crossings(net, p, (r[0] * side, r[1] * side), PORTAL_GAP + 1.0)
        room = hits[0] if hits else 99.0
        spot = (p[0] + r[0] * side * POLE_OFFSET, p[2] + r[1] * side * POLE_OFFSET)
        if _near_signal(net, spot):
            room = 0.0
        if room < PORTAL_GAP:
            continue
        rank, edge = _footing(ground, p, r, side) if ground is not None else (3, None)
        if rank == 0:
            continue
        if best is None or (rank, room) > best[0]:
            best = ((rank, room), side, {"rank": rank, "edge": edge})
    if best is None:
        return None, None
    return best[1], best[2]


def _make_portal(ground, net, p, r):
    """Portal across the bundle of tracks around p: a pole beyond the outermost track on each
    side (where there is PORTAL_GAP free), a beam between them."""
    ends = []
    for side in (1, -1):
        d = (r[0] * side, r[1] * side)
        hits = [0.0] + _crossings(net, p, d, PORTAL_REACH)
        edge = None
        for a, b in zip(hits, hits[1:] + [PORTAL_REACH + PORTAL_GAP]):
            if b - a >= PORTAL_GAP:
                edge = a
                break
        if edge is None or edge > PORTAL_REACH - PORTAL_GAP:
            return None
        ends.append(edge + POLE_OFFSET)
    rail_y = p[1]
    poles = []
    for side, out in zip((1, -1), ends):
        x, z = p[0] + r[0] * side * out, p[2] + r[1] * side * out
        candidates = [h for h in ground.heights(x, z) if rail_y - 6.0 <= h <= rail_y + 3.0]
        if not candidates:
            return None                      # a portal post needs ground (not over a viaduct)
        poles.append([x, max(candidates), z])
    centre = [(poles[0][0] + poles[1][0]) / 2, rail_y, (poles[0][2] + poles[1][2]) / 2]
    return {"poles": poles, "rail_y": rail_y, "centre": centre}


CROSS_CELL = 50.0


def _chain_crossing(points, cumulative, portal, cells=None):
    """Arc length where a chain crosses the portal line, or None. `cells`: grid of the chain's
    segment indices (CROSS_CELL), to look only at the segments near the portal."""
    a, b = portal["poles"]
    if cells is None:
        candidates = range(len(points) - 1)
    else:
        found = set()
        for cx in range(int(math.floor(min(a[0], b[0]) / CROSS_CELL)), int(math.floor(max(a[0], b[0]) / CROSS_CELL)) + 1):
            for cz in range(int(math.floor(min(a[2], b[2]) / CROSS_CELL)), int(math.floor(max(a[2], b[2]) / CROSS_CELL)) + 1):
                found.update(cells.get((cx, cz), ()))
        candidates = sorted(found)
    for i in candidates:
        p, q = points[i], points[i + 1]
        d = (q[0] - p[0], q[2] - p[2])
        seg = math.hypot(*d)
        if seg < 1e-9:
            continue
        hit = signs._ray_hits_segment((p[0], p[2]), (d[0] / seg, d[1] / seg), seg + 1e-6,
                                      (a[0], a[2]), (b[0], b[2]))
        if hit is not None:
            return cumulative[i] + hit
    return None


def _place_support(ground, item, index):
    p, t, r = item["p"], item["t"], item["r"]
    rail_y = p[1]
    stagger = 0.0                       # set by _assign_stagger once the runs are final
    support = {"track": [p[0], p[1], p[2]], "t": [t[0], t[1]], "stagger": stagger,
               "side": item["side"], "portal": item.get("portal"),
               "radius": item["radius"], "turn": item["turn"]}
    if item.get("portal") is None:
        side = item["side"]
        pole_x = p[0] + r[0] * side * POLE_OFFSET
        pole_z = p[2] + r[1] * side * POLE_OFFSET
        candidates = [h for h in ground.heights(pole_x, pole_z) if rail_y - 6.0 <= h <= rail_y + 3.0]
        support["pole"] = [pole_x, max(candidates) if candidates else rail_y - 0.6, pole_z]
        support["pole_base"] = rail_y - POLE_BASE_BELOW_RAIL
        support["pole_owner"] = None
        footing = item.get("footing")
        if footing is not None and footing["edge"] is not None:
            # over a viaduct: the pole stands on a steel console fixed to the deck side
            support["bracket"] = footing["edge"]
            support["pole"][1] = support["pole_base"]
    return support


# ---------------------------------------------------------------- geometry

def _tube(builder, a, b, radius, sides, color, anchor, caps=False):
    """Prism along a-b (flat shaded quads); caps=True closes both ends."""
    axis = _unit(_sub(b, a))
    helper = (0.0, 1.0, 0.0) if abs(axis[1]) < 0.9 else (1.0, 0.0, 0.0)
    u = _unit(_cross(axis, helper))
    v = _cross(axis, u)
    ring = []
    for i in range(sides):
        ang = 2 * math.pi * (i + 0.5) / sides
        ring.append(_add(_mul(u, math.cos(ang) * radius), _mul(v, math.sin(ang) * radius)))
    for i in range(sides):
        o0, o1 = ring[i], ring[(i + 1) % sides]
        outward = _unit(_add(o0, o1))
        builder.polygon([_add(a, o0), _add(a, o1), _add(b, o1), _add(b, o0)], color, anchor, outward)
    if caps:
        builder.polygon([_add(a, o) for o in ring], color, anchor, _mul(axis, -1.0))
        builder.polygon([_add(b, o) for o in ring], color, anchor, axis)


def _cone(builder, base, height, r0, r1, sides, color, anchor):
    top = (base[0], base[1] + height, base[2])
    ring0, ring1 = [], []
    for i in range(sides):
        ang = 2 * math.pi * (i + 0.5) / sides
        c, s = math.cos(ang), math.sin(ang)
        ring0.append((base[0] + c * r0, base[1], base[2] + s * r0))
        ring1.append((top[0] + c * r1, top[1], top[2] + s * r1))
    for i in range(sides):
        j = (i + 1) % sides
        ang = 2 * math.pi * (i + 1) / sides
        outward = (math.cos(ang), (r0 - r1) / height, math.sin(ang))
        builder.polygon([ring0[i], ring0[j], ring1[j], ring1[i]], color, anchor, outward)
    builder.polygon(ring1, color, anchor, (0.0, 1.0, 0.0))


def _wire(builder, a, b, sag, pieces, radius, color, anchor):
    """Wire from a to b hanging `sag` metres at mid span (parabola), as tube pieces."""
    prev = a
    for i in range(1, pieces + 1):
        f = i / pieces
        q = _lerp(a, b, f)
        q = (q[0], q[1] - 4.0 * sag * f * (1.0 - f), q[2])
        _tube(builder, prev, q, radius, 3, color, anchor)
        prev = q


def _support_points(sp):
    """Key points of one support (world). Vertical catenary: the messenger hangs right above the
    contact wire (same zigzag), so the droppers between them are vertical."""
    tx, tz = sp["t"]
    r = signs.right_of((tx, tz))
    track = sp["track"]
    rail_y = track[1]
    contact = (track[0] + r[0] * sp["stagger"], rail_y + CONTACT_WIRE_HEIGHT + sp.get("raise", 0.0),
               track[2] + r[1] * sp["stagger"])
    messenger = (contact[0], rail_y + CONTACT_WIRE_HEIGHT + SYSTEM_HEIGHT, contact[2])
    if sp["portal"] is not None:
        return {"r": r, "messenger": messenger, "contact": contact}
    pole = sp["pole"]
    # unit vector from the pole towards the wires, horizontal (a shared pole stands a little
    # further away or askew)
    towards = _unit2((messenger[0] - pole[0], messenger[2] - pole[2]))
    to_track = (towards[0], 0.0, towards[1])
    axis_x, axis_z = pole[0], pole[2]

    def from_pole(dist_out, height):
        return (axis_x + to_track[0] * dist_out, rail_y + height, axis_z + to_track[2] * dist_out)

    base_y = pole_base_y(sp)
    skoba_height = CONTACT_WIRE_HEIGHT + SYSTEM_HEIGHT + SKOBA_ABOVE_MESSENGER
    # hinge and tie start a little inside the pole surface (the pole is an octagon: its flats are
    # nearer the axis than its corners), held by a clamp band round the pole
    hinge = from_pole(pole_radius_at(base_y, rail_y + CANTILEVER_HINGE) * POLE_FLAT_FACTOR, CANTILEVER_HINGE)
    tie = from_pole(pole_radius_at(base_y, rail_y + skoba_height + TIE_ABOVE_SKOBA) * POLE_FLAT_FACTOR,
                    skoba_height + TIE_ABOVE_SKOBA)
    skoba = (messenger[0], rail_y + skoba_height, messenger[2])
    f = (FIXATOR_ATTACH_HEIGHT - CANTILEVER_HINGE) / (skoba_height - CANTILEVER_HINGE)
    attach = _lerp(hinge, skoba, f)
    contact_out = math.hypot(contact[0] - axis_x, contact[2] - axis_z)
    top_y = base_y + POLE_HEIGHT
    return {"r": r, "to_track": to_track, "messenger": messenger, "contact": contact,
            "hinge": hinge, "tie": tie, "skoba": skoba, "attach": attach, "contact_out": contact_out,
            "top_y": top_y, "from_pole": from_pole}


def pole_base_y(sp):
    return sp.get("pole_base", sp["track"][1] - POLE_BASE_BELOW_RAIL)


def wire_ends(sp):
    """(messenger, contact) end points of the wires at a support: over the track, or for an
    anchor support at the pole, where they are anchored."""
    if sp.get("anchor") is None:
        k = _support_points(sp)
        return k["messenger"], k["contact"]
    pole, rail_y = sp["pole"], sp["track"][1]
    tx, tz = sp["anchor"]["towards"]
    return ((pole[0] + tx * MESSENGER_END_OUT, rail_y + MESSENGER_ANCHOR_HEIGHT, pole[2] + tz * MESSENGER_END_OUT),
            (pole[0] + tx * CONTACT_END_OUT, rail_y + COMPENSATOR_HEIGHT, pole[2] + tz * CONTACT_END_OUT))


def _insulator_chain(builder, a, b, anchor, sheds=3, shed_radius=0.11, core_radius=0.03):
    """Insulator from a to b: the sheds (discs) sit close together in the middle on the insulator
    body, steel fittings run on to both ends."""
    axis = _unit(_sub(b, a))
    length = math.dist(a, b)
    pitch = length / sheds * SHED_SPACING_FACTOR
    body = min(length, sheds * pitch + 0.04)
    body_start = _add(a, _mul(axis, (length - body) / 2))
    body_end = _add(body_start, _mul(axis, body))
    _tube(builder, a, body_start, core_radius * 0.6, 4, COLOR_STEEL, anchor, caps=True)
    _tube(builder, body_end, b, core_radius * 0.6, 4, COLOR_STEEL, anchor, caps=True)
    _tube(builder, body_start, body_end, core_radius, 6, COLOR_INSULATOR, anchor, caps=True)
    for i in range(sheds):
        centre = _add(body_start, _mul(axis, 0.02 + pitch * (i + 0.5)))
        _tube(builder, _add(centre, _mul(axis, -0.022)), _add(centre, _mul(axis, 0.022)), shed_radius, 6,
              COLOR_INSULATOR, anchor, caps=True)


def pole_radius_at(sp_base_y, y):
    """Radius of the concrete pole at height y (the cone of _concrete_pole)."""
    bottom = sp_base_y - POLE_BURIED
    f = max(0.0, min(1.0, (y - bottom) / (POLE_HEIGHT + POLE_BURIED)))
    return (POLE_RADIUS_GROUND + 0.007) * (1 - f) + POLE_RADIUS_TOP * f


def _concrete_pole(builder, pole, base_y, top_y, t, origin, anchor, buried=POLE_BURIED):
    """Concrete pole from base_y (below the rail) up to top_y, on a square foundation block
    where the ground is lower than the base, with a collider."""
    ox, oz = origin
    ground_y = pole[1]
    frame = signs.Frame((pole[0] - ox, base_y, pole[2] - oz), tuple(t))
    if ground_y < base_y - 0.05:
        half = FOUNDATION_SIDE / 2
        drop = base_y - ground_y + FOUNDATION_BURIED
        signs._box(builder, frame, half, half, -drop, 0.0, [(-drop, 0.0, COLOR_FOUNDATION)], anchor)
        signs.add_box_collision(builder, frame, half, half, -drop, 0.0, anchor)
    height = top_y - base_y
    _cone(builder, (pole[0] - ox, base_y - buried, pole[2] - oz), height + buried,
          POLE_RADIUS_GROUND + 0.007, POLE_RADIUS_TOP, POLE_SIDES, COLOR_CONCRETE, anchor)
    signs.add_box_collision(builder, frame, POLE_RADIUS_GROUND, POLE_RADIUS_GROUND,
                            -buried, height, anchor)


def add_bracket(builder, sp, origin, anchor):
    """Steel console carrying a pole beside a viaduct deck: a beam from the deck edge out under
    the pole, a strut down to the deck side, a base plate."""
    ox, oz = origin
    pole, track = sp["pole"], sp["track"]
    base_y = pole_base_y(sp)
    out = _unit2((pole[0] - track[0], pole[2] - track[2]))
    reach = math.hypot(pole[0] - track[0], pole[2] - track[2])
    edge = sp["bracket"]
    inner, outer = edge - 0.30, reach + 0.40
    width, height = BRACKET_BEAM
    centre = (track[0] + out[0] * (inner + outer) / 2, track[2] + out[1] * (inner + outer) / 2)
    frame = signs.Frame((centre[0] - ox, base_y - height, centre[1] - oz), out)
    signs._box(builder, frame, (outer - inner) / 2, width / 2, 0.0, height, [(0.0, height, COLOR_STEEL)], anchor)
    builder.polygon([frame.at(-(outer - inner) / 2, -width / 2, 0.0), frame.at((outer - inner) / 2, -width / 2, 0.0),
                     frame.at((outer - inner) / 2, width / 2, 0.0), frame.at(-(outer - inner) / 2, width / 2, 0.0)],
                    COLOR_STEEL, anchor, (0.0, -1.0, 0.0))
    strut_top = (track[0] + out[0] * (outer - 0.15) - ox, base_y - height, track[2] + out[1] * (outer - 0.15) - oz)
    strut_foot = (track[0] + out[0] * edge - ox, base_y - BRACKET_DROP, track[2] + out[1] * edge - oz)
    _tube(builder, strut_top, strut_foot, 0.06, 4, COLOR_STEEL, anchor, caps=True)
    # plate on the deck side where the beam and the strut are bolted on
    plate = signs.Frame((track[0] + out[0] * edge - ox, base_y - BRACKET_DROP - 0.1, track[2] + out[1] * edge - oz), out)
    signs._box(builder, plate, 0.03, width / 2 + 0.05, 0.0, BRACKET_DROP + 0.1 - height * 0.5,
               [(0.0, BRACKET_DROP + 0.1 - height * 0.5, COLOR_STEEL)], anchor)
    base = signs.Frame((pole[0] - ox, base_y, pole[2] - oz), out)
    signs._box(builder, base, 0.32, 0.32, 0.0, 0.05, [(0.0, 0.05, COLOR_STEEL)], anchor)


def portal_beam_y(portal):
    return portal.get("beam_y", portal["rail_y"] + PORTAL_BEAM_HEIGHT)


def portal_beam_size(portal):
    return TRUSS_BEAM_SIZE if portal.get("kind") == "truss" else PORTAL_BEAM_SIZE


def add_portal(builder, portal, origin):
    """Rigid cross-span: two concrete poles beyond the outer tracks, a steel box beam on top."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    a, b = portal["poles"]
    across = _unit((b[0] - a[0], 0.0, b[2] - a[2]))
    along = (across[2], -across[0])                 # horizontal, along the tracks
    beam_y = portal_beam_y(portal)
    base_y = portal["rail_y"] - POLE_BASE_BELOW_RAIL
    width, height = portal_beam_size(portal)
    truss = portal.get("kind") == "truss"
    if not truss:
        for pole in (a, b):
            anchor = (pole[0] - ox, pole[2] - oz)
            _concrete_pole(builder, pole, base_y, beam_y + height / 2 + 0.25, along, origin, anchor)
    # beam: a box from pole to pole (past the pole axes by a little); in a truss a cross girder
    # from top chord to top chord
    over = 0.0 if truss else 0.35
    start = _add((a[0], beam_y, a[2]), _mul(across, -over))
    end = _add((b[0], beam_y, b[2]), _mul(across, over))
    centre = _lerp(start, end, 0.5)
    frame_t = (across[0], across[2])
    frame = signs.Frame(local((centre[0], beam_y - height / 2, centre[2])), frame_t)
    half_length = math.dist(start, end) / 2
    anchor = (centre[0] - ox, centre[2] - oz)
    color = COLOR_TRUSS if truss else COLOR_STEEL
    signs._box(builder, frame, half_length, width / 2, 0.0, height, [(0.0, height, color)], anchor)
    builder.polygon([frame.at(-half_length, -width / 2, 0.0), frame.at(half_length, -width / 2, 0.0),
                     frame.at(half_length, width / 2, 0.0), frame.at(-half_length, width / 2, 0.0)],
                    color, anchor, (0.0, -1.0, 0.0))


def add_portal_hanger(builder, sp, portal, origin):
    """Wires of one track hung from a portal beam: insulator under the beam, rod down to the
    messenger, a rigid rod from the messenger to the contact wire."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    k = _support_points(sp)
    messenger, contact = k["messenger"], k["contact"]
    anchor = (messenger[0] - ox, messenger[2] - oz)
    beam_bottom = portal_beam_y(portal) - portal_beam_size(portal)[1] / 2
    top = (messenger[0], beam_bottom, messenger[2])
    insulator_end = (messenger[0], beam_bottom - INSULATOR_LENGTH, messenger[2])
    _insulator_chain(builder, local(top), local(insulator_end), anchor)
    if insulator_end[1] > messenger[1] + 0.05:
        _tube(builder, local(insulator_end), local(messenger), ROD_RADIUS, 4, COLOR_STEEL, anchor,
              caps=True)
    _tube(builder, local(messenger), local((contact[0], contact[1] + 0.02, contact[2])),
          TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)


def add_support(builder, sp, origin, supports=None):
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    k = _support_points(sp)
    pole = sp["pole"]
    anchor = (pole[0] - ox, pole[2] - oz)
    # The pole always starts at the same level below the rail (higher ground just hides its foot);
    # where the ground is lower, it stands on a square concrete foundation block. A shared pole
    # is drawn by the support that owns it.
    base_y = pole_base_y(sp)
    if sp.get("pole_owner") is None:
        if sp.get("bracket") is not None:
            add_bracket(builder, sp, origin, anchor)
            _concrete_pole(builder, pole, base_y + 0.05, base_y + POLE_HEIGHT, sp["t"], origin, anchor, buried=0.0)
        else:
            _concrete_pole(builder, pole, base_y, base_y + POLE_HEIGHT, sp["t"], origin, anchor)
    if sp.get("anchor") is not None:
        add_anchor(builder, sp, origin)
        return

    contact, messenger = k["contact"], k["messenger"]
    skoba, attach, from_pole, out = k["skoba"], k["attach"], k["from_pole"], k["contact_out"]
    clamp = (contact[0], contact[1] + 0.02, contact[2])
    if sp.get("fixator") != "overlap":
        # cantilever tube with its insulator at the pole, up to and a little past the bracket
        hinge = k["hinge"]
        for point in (hinge, k["tie"]):
            band_radius = pole_radius_at(base_y, point[1]) + 0.015
            _tube(builder, local((sp["pole"][0], point[1] - CLAMP_BAND_HEIGHT / 2, sp["pole"][2])),
                  local((sp["pole"][0], point[1] + CLAMP_BAND_HEIGHT / 2, sp["pole"][2])), band_radius,
                  POLE_SIDES, COLOR_STEEL, anchor, caps=True)
        direction = _unit(_sub(skoba, hinge))
        insulator_end = _add(hinge, _mul(direction, INSULATOR_LENGTH))
        _insulator_chain(builder, local(hinge), local(insulator_end), anchor)
        _tube(builder, local(insulator_end), local(_add(skoba, _mul(direction, CANTILEVER_OVERHANG))),
              TUBE_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
        # tie rod from higher up the pole to the bracket, insulator at the pole
        tie = k["tie"]
        rod_dir = _unit(_sub(skoba, tie))
        rod_insulator_end = _add(tie, _mul(rod_dir, INSULATOR_LENGTH))
        _insulator_chain(builder, local(tie), local(rod_insulator_end), anchor)
        _tube(builder, local(rod_insulator_end), local(skoba), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
        _tube(builder, local(_add(skoba, (0.0, -0.06, 0.0))), local(_add(skoba, (0.0, 0.06, 0.0))),
              0.05, 6, COLOR_STEEL, anchor, caps=True)
    if sp.get("fixator") == "overlap" and supports is not None:
        # second wire on another support's cantilever: hook and steady arm from that one
        host_index = sp.get("overlap_of")
        if host_index is None:
            host_index = sp.get("pole_owner")
        if (host_index is not None and supports[host_index].get("anchor") is None
                and supports[host_index]["portal"] is None):
            host = _support_points(supports[host_index])
            skoba = host["skoba"]
            # the second wire's steady arm leaves the host's tube higher up, above the host's
            # own fixator, so the two never run through each other
            f = (SECOND_ARM_ATTACH_HEIGHT - CANTILEVER_HINGE) / (host["skoba"][1] - sp["track"][1] - CANTILEVER_HINGE)
            attach = _lerp(host["hinge"], host["skoba"], f)
    # the messenger hangs from the bracket
    _tube(builder, local(skoba), local(messenger), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    if sp.get("fixator") == "reverse":
        end = from_pole(out + MAIN_FIXATOR_BEYOND, MAIN_FIXATOR_END_HEIGHT)
        _tube(builder, local(attach), local(end), TUBE_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
        _tube(builder, local(_add(end, (0.0, -0.12, 0.0))), local(_add(end, (0.0, 0.10, 0.0))),
              TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)
        arm_start = from_pole(out + MAIN_FIXATOR_BEYOND, STEADY_ARM_HEIGHT)
        bend = from_pole(out + STEADY_ARM_BEND, STEADY_ARM_HEIGHT)
        _tube(builder, local(arm_start), local(bend), 0.022, 4, COLOR_STEEL, anchor, caps=True)
        _tube(builder, local(bend), local(clamp), 0.022, 4, COLOR_STEEL, anchor, caps=True)
        string_low = _lerp(attach, end, (FIXATOR_STRING_BEYOND + out - math.hypot(attach[0] - sp["pole"][0],
                           attach[2] - sp["pole"][2])) / max(0.1, math.dist((attach[0], attach[2]), (end[0], end[2]))))
        _tube(builder, local(messenger), local(string_low), DROPPER_RADIUS, 3, COLOR_STEEL, anchor)
    else:
        # direct fixator (and the second wire at a turnout overlap): steady arm from the tube
        attach_out = math.hypot(attach[0] - sp["pole"][0], attach[2] - sp["pole"][2])
        if out - STEADY_ARM_BEND > attach_out + 0.2:
            bend = from_pole(out - STEADY_ARM_BEND, STEADY_ARM_HEIGHT + sp.get("raise", 0.0))
            _tube(builder, local(attach), local(bend), 0.022, 4, COLOR_STEEL, anchor, caps=True)
            _tube(builder, local(bend), local(clamp), 0.022, 4, COLOR_STEEL, anchor, caps=True)
        else:
            _tube(builder, local(attach), local(clamp), 0.022, 4, COLOR_STEEL, anchor, caps=True)


def add_anchor(builder, sp, origin):
    """Anchoring at a pole (see MESSENGER_ANCHOR_HEIGHT): messenger to the pole top, contact wire
    through the two-stage pulley block and the lever pulley to the weights, two guy ropes."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    pole, rail_y = sp["pole"], sp["track"][1]
    anchor = (pole[0] - ox, pole[2] - oz)
    tx, tz = sp["anchor"]["towards"]
    base_y = pole_base_y(sp)

    def out(distance, height):
        return local((pole[0] + tx * distance, rail_y + height, pole[2] + tz * distance))

    def rope(a, b, radius=DROPPER_RADIUS * 1.4):
        _tube(builder, a, b, radius, 4, COLOR_STEEL, anchor, caps=True)

    def clamp(height):
        """Band round the pole where something is fixed to it."""
        radius = pole_radius_at(base_y, rail_y + height) + 0.015
        _tube(builder, out(0.0, height - 0.04), out(0.0, height + 0.04), radius, 8, COLOR_STEEL, anchor,
              caps=True)

    def surface(height):
        return pole_radius_at(base_y, rail_y + height)

    across = (-tz, 0.0, tx)                    # pulley axles: horizontal, square to the wires

    def pulley(centre):
        _tube(builder, _add(centre, _mul(across, -0.035)), _add(centre, _mul(across, 0.035)),
              PULLEY_RADIUS, 10, COLOR_STEEL, anchor, caps=True)

    r = PULLEY_RADIUS
    h = COMPENSATOR_HEIGHT
    # messenger: insulator, then a rope to the clamp at the pole top
    clamp(MESSENGER_ANCHOR_HEIGHT)
    _insulator_chain(builder, out(MESSENGER_INSULATOR[1], MESSENGER_ANCHOR_HEIGHT),
                     out(MESSENGER_INSULATOR[0], MESSENGER_ANCHOR_HEIGHT), anchor, sheds=ANCHOR_SHEDS)
    rope(out(MESSENGER_END_OUT, MESSENGER_ANCHOR_HEIGHT), out(MESSENGER_INSULATOR[1], MESSENGER_ANCHOR_HEIGHT),
         ROD_RADIUS)
    rope(out(MESSENGER_INSULATOR[0], MESSENGER_ANCHOR_HEIGHT),
         out(surface(MESSENGER_ANCHOR_HEIGHT), MESSENGER_ANCHOR_HEIGHT), ROD_RADIUS)
    # contact wire: clamp, rope, insulator, rope to the outer moving pulley
    rope(out(CONTACT_END_OUT, h), out(CONTACT_INSULATOR[1], h), ROD_RADIUS)
    _insulator_chain(builder, out(CONTACT_INSULATOR[1], h), out(CONTACT_INSULATOR[0], h), anchor,
                     sheds=ANCHOR_SHEDS)
    rope(out(CONTACT_INSULATOR[0], h), out(OUTER_PULLEY_OUT + r, h), ROD_RADIUS)
    # outer pulley: its rope runs from the pole (upper strand) round it to the inner pulley's axle
    clamp(h + r)
    outer = out(OUTER_PULLEY_OUT, h)
    inner = out(INNER_PULLEY_OUT, h - r)
    pulley(outer)
    pulley(inner)
    rope(out(surface(h + r), h + r), out(OUTER_PULLEY_OUT, h + r))
    rope(out(OUTER_PULLEY_OUT, h - r), inner)
    # inner pulley: its rope runs from the pole round it, back to the lever pulley, down to weights
    clamp(h)
    rope(out(surface(h), h), out(INNER_PULLEY_OUT, h))
    lever_pulley = out(WEIGHTS_OUT + r, h - LEVER_DROP)
    rope(out(INNER_PULLEY_OUT, h - 2 * r), out(WEIGHTS_OUT + r, h - LEVER_DROP + r))
    pulley(lever_pulley)
    _tube(builder, out(surface(h - 0.08), h - 0.08), lever_pulley, 0.035, 4, COLOR_STEEL, anchor, caps=True)
    # weights: two stacks of thick concrete discs side by side (across the wires), each on its own
    # rod with a bottom plate; a bar over both rods carries them on the rope
    radius, disc_height, discs = WEIGHT_STACK
    stack_top = WEIGHTS_BOTTOM + discs * disc_height
    bar_y = stack_top + 0.08
    weights_centre = local((pole[0] + tx * WEIGHTS_OUT, rail_y, pole[2] + tz * WEIGHTS_OUT))
    frame_r = signs.right_of((tx, tz))
    sideways = (frame_r[0], 0.0, frame_r[1])
    stack_offset = radius + WEIGHT_STACK_GAP / 2
    rope(out(WEIGHTS_OUT, h - LEVER_DROP), out(WEIGHTS_OUT, bar_y))
    _tube(builder, _add(out(WEIGHTS_OUT, bar_y), _mul(sideways, -stack_offset - 0.02)),
          _add(out(WEIGHTS_OUT, bar_y), _mul(sideways, stack_offset + 0.02)), 0.025, 4, COLOR_STEEL,
          anchor, caps=True)
    for sign in (-1.0, 1.0):
        centre = _add(weights_centre, _mul(sideways, sign * stack_offset))
        for i in range(discs):
            y0 = rail_y + WEIGHTS_BOTTOM + i * disc_height
            # each disc slightly rounded off at its edges: a thin waist between neighbours
            _tube(builder, (centre[0], y0 + 0.008, centre[2]), (centre[0], y0 + disc_height - 0.008, centre[2]),
                  radius, 10, COLOR_FOUNDATION, anchor, caps=True)
            _tube(builder, (centre[0], y0, centre[2]), (centre[0], y0 + disc_height, centre[2]),
                  radius - 0.012, 10, COLOR_FOUNDATION, anchor, caps=False)
        rod_bottom = (centre[0], rail_y + WEIGHTS_BOTTOM - 0.04, centre[2])
        _tube(builder, rod_bottom, (centre[0], rail_y + bar_y, centre[2]), 0.015, 4, COLOR_STEEL,
              anchor, caps=True)
        _tube(builder, rod_bottom, (centre[0], rail_y + WEIGHTS_BOTTOM, centre[2]), radius * 0.8, 8,
              COLOR_STEEL, anchor, caps=True)
    # guy ropes: from the messenger and the compensator height, looping round the pole on the span
    # side, meeting above the ground, then one anchor rod into the ground block
    ground = sp["anchor"]["guy_ground"]
    block_side, block_height_ground = GUY_BLOCK
    foot = local((ground[0], ground[1] + block_height_ground, ground[2]))
    mean_top = out(0.0, (MESSENGER_ANCHOR_HEIGHT + h) / 2)
    rod_dir = _unit(_sub(mean_top, foot))
    joint = _add(foot, _mul(rod_dir, GUY_ROD_LENGTH))
    _tube(builder, foot, joint, GUY_ROD_RADIUS, 6, COLOR_STEEL, anchor, caps=True)
    back = _unit2((ground[0] - pole[0], ground[2] - pole[2]))       # pole -> ground anchor
    side = (-back[1], 0.0, back[0])
    for height in (MESSENGER_ANCHOR_HEIGHT, h):
        y = rail_y + height
        radius = pole_radius_at(base_y, y) + 0.012
        centre = local((pole[0], y, pole[2]))
        horizontal = math.hypot(joint[0] - centre[0], joint[2] - centre[2])
        split = _lerp(centre, joint, min(0.5, GUY_SPLIT / max(horizontal, 1e-6)))
        rope(joint, split, GUY_RADIUS)
        # the loop round the pole: from one side, round the span side, to the other
        loop = []
        for i in range(7):
            angle = math.pi * i / 6
            direction = _add(_mul(side, math.cos(angle)), (-back[0] * math.sin(angle), 0.0, -back[1] * math.sin(angle)))
            loop.append(_add(centre, _mul(direction, radius)))
        rope(split, loop[0], GUY_RADIUS)
        rope(split, loop[-1], GUY_RADIUS)
        for a, b in zip(loop, loop[1:]):
            rope(a, b, GUY_RADIUS)
    frame = signs.Frame(local((ground[0], ground[1], ground[2])), back)
    signs._box(builder, frame, block_side / 2, block_side / 2, -0.5, block_height_ground,
               [(-0.5, block_height_ground, COLOR_FOUNDATION)], anchor)


def add_feeder_pole(builder, fp, origin):
    """DPR bracket with two suspension insulators on the field side, near the pole top."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    x, z = fp["pos"]
    anchor = (x - ox, z - oz)
    away = (fp["out"][0], 0.0, fp["out"][1])
    y = fp["base_y"] + POLE_HEIGHT - FEEDER_DROP
    arm_a = (x, y, z)
    arm_b = (x + away[0] * (FEEDER_ARM[1] + 0.1), y, z + away[2] * (FEEDER_ARM[1] + 0.1))
    _tube(builder, local(arm_a), local(arm_b), ROD_RADIUS * 1.6, 4, COLOR_STEEL, anchor, caps=True)
    for out in FEEDER_ARM:
        top = (x + away[0] * out, y, z + away[2] * out)
        _insulator_chain(builder, local(top), local((top[0], top[1] - 0.35, top[2])), anchor,
                         shed_radius=0.075, core_radius=0.025)


def feeder_points(fp):
    x, z = fp["pos"]
    y = fp["base_y"] + POLE_HEIGHT - FEEDER_DROP - 0.35
    return [(x + fp["out"][0] * out, y, z + fp["out"][1] * out) for out in FEEDER_ARM]


def add_feeder_span(builder, a, b, origin):
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    anchor = ((a["pos"][0] + b["pos"][0]) / 2 - ox, (a["pos"][1] + b["pos"][1]) / 2 - oz)
    length = math.dist(a["pos"], b["pos"])
    sag = FEEDER_SAG_70M * (length / 70.0) ** 2
    pieces = max(4, int(round(length / 8.0)))
    pa, pb = feeder_points(a), feeder_points(b)
    # wires keep their order across: the one nearer the track stays nearer
    if math.dist(pa[0], pb[0]) + math.dist(pa[1], pb[1]) > math.dist(pa[0], pb[1]) + math.dist(pa[1], pb[0]):
        pb = pb[::-1]
    for p, q in zip(pa, pb):
        _wire(builder, local(p), local(q), sag, pieces, WIRE_RADIUS, COLOR_FEEDER, anchor)


def add_span(builder, a, b, origin, span=None):
    ox, oz = origin
    span = span or {}

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    anchor = ((a["track"][0] + b["track"][0]) / 2 - ox, (a["track"][2] + b["track"][2]) / 2 - oz)
    length = math.dist((a["track"][0], a["track"][2]), (b["track"][0], b["track"][2]))
    scale = (length / 70.0) ** 2
    sag = MESSENGER_SAG_70M * scale
    pieces = max(4, int(round(length / 8.0)))
    m0, c0 = wire_ends(a)
    m1, c1 = wire_ends(b)
    _wire(builder, local(m0), local(m1), sag, pieces, WIRE_RADIUS, COLOR_MESSENGER, anchor)
    _tube(builder, local(c0), local(c1), WIRE_RADIUS, 3, COLOR_CONTACT, anchor)
    # droppers
    count = int(length // DROPPER_SPACING)
    for i in range(1, count + 1):
        f = i / (count + 1)
        c = _lerp(c0, c1, f)
        m = _lerp(m0, m1, f)
        m = (m[0], m[1] - 4.0 * sag * f * (1.0 - f), m[2])
        if m[1] - c[1] < 0.25:
            continue
        _tube(builder, local(c), local(m), DROPPER_RADIUS, 3, COLOR_MESSENGER, anchor)

    def messenger_at(f):
        m = _lerp(m0, m1, f)
        return (m[0], m[1] - 4.0 * sag * f * (1.0 - f), m[2])

    if span.get("middle_anchor"):
        # middle anchor: a rope clamped to the contact wire at mid span, its ends on the messenger
        mid = _lerp(c0, c1, 0.5)
        for f in (0.5 - MIDDLE_ANCHOR_REACH, 0.5 + MIDDLE_ANCHOR_REACH):
            _tube(builder, local(messenger_at(f)), local((mid[0], mid[1] + 0.03, mid[2])), DROPPER_RADIUS * 1.4,
                  4, COLOR_STEEL, anchor, caps=True)
        _tube(builder, local((mid[0], mid[1] - 0.02, mid[2])), local((mid[0], mid[1] + 0.06, mid[2])), 0.03, 6,
              COLOR_STEEL, anchor, caps=True)
    if span.get("insulators") in ("from", "to") and length > 6.0:
        # insulating overlap: insulators in both wires of the anchored branch, near the
        # transition support
        near, far = OVERLAP_INSULATOR_AT
        for d0, d1 in ((near, far),):
            if span["insulators"] == "from":
                f0, f1 = d0 / length, d1 / length
            else:
                f0, f1 = 1.0 - d0 / length, 1.0 - d1 / length
            _insulator_chain(builder, local(messenger_at(f0)), local(messenger_at(f1)), anchor, sheds=ANCHOR_SHEDS)
            _insulator_chain(builder, local(_lerp(c0, c1, f0)), local(_lerp(c0, c1, f1)), anchor, sheds=ANCHOR_SHEDS)


def add_catenary(builder, plan, tile, origin):
    """Geometry of all supports and spans assigned to `tile`."""
    supports = plan["supports"]
    count = 0
    portals = plan.get("portals", [])
    for portal in portals:
        if portal["tile"] == tile and not portal.get("unused"):
            add_portal(builder, portal, origin)
            count += 1
    for sp in supports:
        if sp["tile"] == tile and not sp.get("dropped"):
            if sp["portal"] is None:
                add_support(builder, sp, origin, supports)
                count += 1
            else:
                add_portal_hanger(builder, sp, portals[sp["portal"]], origin)
    for span in plan["spans"]:
        if span["tile"] == tile:
            add_span(builder, supports[span["from"]], supports[span["to"]], origin, span)
    feeder_poles = plan.get("feeder_poles", [])
    for fp in feeder_poles:
        if fp["tile"] == tile:
            add_feeder_pole(builder, fp, origin)
    for span in plan.get("feeder_spans", []):
        if span["tile"] == tile:
            add_feeder_span(builder, feeder_poles[span["from"]], feeder_poles[span["to"]], origin)
    return count
