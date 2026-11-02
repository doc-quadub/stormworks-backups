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

# ---------------------------------------------------------------- dimensions (metres)

CONTACT_WIRE_HEIGHT = 6.25        # above the rail top (allowed 5.75 - 6.80)
SYSTEM_HEIGHT = 1.80              # messenger above the contact wire at the supports
STAGGER = 0.30                    # contact wire zigzag at the supports, straight track
STAGGER_ON_CURVE = 0.40           # on curves: pulled to the outer side
MAX_SPAN = 70.0
MIN_SPAN = 30.0
CURVE_SAG_ALLOWANCE = 0.60        # a span may cut the curve by this much (chord vs track)
MESSENGER_SAG_70M = 1.10          # sag of the messenger in a 70 m span (scales with span^2)
FEEDER_SAG_70M = 1.40
DROPPER_SPACING = 9.0

POLE_OFFSET = 3.30                # pole centre from the track axis (clearance >= 3.1 m)
POLE_HEIGHT = 9.60                # above the ground (SS136.6: 13.6 m, 4 m in the ground)
POLE_BURIED = 0.50                # drawn below the ground point (slopes)
POLE_BASE_BELOW_RAIL = 0.80       # every pole stands on this level (below the rail top), so all
                                  # cantilevers sit at the same height on it
FOUNDATION_SIDE = 0.90            # square concrete block under a pole whose ground is lower
FOUNDATION_BURIED = 1.20
COLOR_FOUNDATION = (128, 126, 120, 255)
POLE_RADIUS_GROUND = 0.217        # 433 mm at the ground line (490 mm at the foot, 290 at top)
POLE_RADIUS_TOP = 0.145
POLE_SIDES = 8

CANTILEVER_HINGE = 6.70           # lower end of the inclined tube on the pole, above rail top
TIE_ROD_HINGE = 8.60              # tie rod on the pole, above rail top
CANTILEVER_OVERHANG = 0.35        # tube runs on past the messenger point
REGISTRATION_ARM_START = 1.10     # from the pole: where the registration arm leaves the tube
FEEDER_ARM = (0.55, 1.10)         # feeder wires, outwards from the pole axis (field side)
FEEDER_DROP = 0.45                # below the pole top

TUBE_RADIUS = 0.030
ROD_RADIUS = 0.016
INSULATOR_RADIUS = 0.085
INSULATOR_LENGTH = 0.80
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
# Compensated anchoring (both wires on one compensator, as on Russian main lines): the anchored
# branch ends in an insulator chain on each wire, both chains meet at a yoke, the yoke pulls on
# the moving block of a pulley block whose fixed block sits at the pole; the rope runs on over
# the fixed block down along the pole to a stack of weights. Heights above the rail, distances
# out from the pole axis towards the span.
COMPENSATOR_HEIGHT = 7.25            # 1.0 m above the working contact wire
ANCHOR_CONTACT_HEIGHT = 7.00         # anchored branch rises out of the pantograph's reach
ANCHOR_MESSENGER_HEIGHT = 7.50
FIXED_BLOCK_OUT = 0.42
MOVING_BLOCK_OUT = 1.25
YOKE_OUT = 1.55
ANCHOR_INSULATOR_LENGTH = 1.00       # insulator chain of each wire, yoke -> wire end
WIRE_END_OUT = YOKE_OUT + ANCHOR_INSULATOR_LENGTH
FIXED_BLOCK_RADIUS = 0.14
MOVING_BLOCK_RADIUS = 0.20
WEIGHTS_RADIUS = 0.17
WEIGHTS_HEIGHT = 1.60
WEIGHTS_BOTTOM = 1.90                # above the rail
GUY_TOP = COMPENSATOR_HEIGHT         # guy wire leaves the pole at the compensator, opposite side
GUY_INSULATOR_FROM_FOOT = 1.5        # insulator in the guy wire, this far up from the ground anchor
GUY_DISTANCE = 7.0                   # ground anchor this far behind the pole
GUY_CLEARANCE = 2.6                  # ground anchor from any track axis, at least
GUY_RADIUS = 0.014
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
    route = _route(net, start, end)
    # the route runs exactly from `start` to `end`, not just between the nearest nodes
    route = [(start[0], 0.0, start[1])] + route + [(end[0], 0.0, end[1])]
    route_index = signs.SegmentIndex()
    for a, b in zip(route, route[1:]):
        route_index.add((a[0], a[2]), (b[0], b[2]), None)

    def near_route(p):
        return any(signs._point_segment_distance((p[0], p[2]), sa, sb) <= corridor
                   for sa, sb, _ in route_index.near(p[0], p[2], corridor))

    def edge_near_route(a, b):
        pa, pb = net.nodes_world[a], net.nodes_world[b]
        pieces = max(1, int(math.dist(pa, pb) // COVER_SAMPLE_STEP))
        return any(near_route(_lerp(pa, pb, i / pieces)) for i in range(pieces + 1))

    ground = signs.Ground(rom_dir)

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
    portals = []
    for round_index in range(PORTAL_ROUNDS):
        count = len(portals)
        planned = [_space_supports(ground, net, points, cumulative, first, last, portals)
                   for chain, points, cumulative, first, last in chain_data]
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
            support["tile"] = _chain_tile(net, chain, cumulative, item["s"])
            support["s"] = item["s"]
            support["run"] = run_index
            indices.append(len(supports))
            supports.append(support)
        runs.append(indices)

    shared = _share_poles(net, chain_data, runs, supports)
    overlaps = _extend_over_turnouts(net, chain_data, runs, supports)
    for indices in runs:
        for a, b in zip(indices, indices[1:]):
            spans.append({"tile": supports[a]["tile"], "from": a, "to": b})
    anchors = _mark_anchors(ground, net, runs, supports)
    for portal in portals:
        portal["tile"] = _tile_of_point(net, portal["centre"])
    feeder_poles, feeder_spans = _plan_feeders(net, route, chain_data, supports, portals)
    log("catenary: route nodes %d, electrified edges %d, wire runs %d, supports %d (on portals %d, "
        "on shared poles %d, anchors %d, left out %d), turnout overlaps %d, spans %d, portals %d, DPR spans %d"
        % (len(route), len(electrified), len(chain_data), len(supports),
           sum(1 for sp in supports if sp["portal"] is not None), shared, anchors, blocked,
           overlaps, len(spans), len(portals), len(feeder_spans)))
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
                        if other["run"] == sp["run"]:
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
                        if best is None or abs(s_new - sp["s"]) < best[0]:
                            best = (abs(s_new - sp["s"]), owner, s_new, p, t)
            if best is None:
                owners.append(index)
                grid.setdefault((cx, cz), []).append(index)
                continue
            _, owner, s_new, p, t = best
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
            if len(beyond) < 2:
                continue
            middle, anchor = beyond[0], beyond[1]
            if (supports[anchor]["portal"] is not None or anchor in (main[0], main[-1])
                    or abs(supports[anchor]["s"] - s_node) > OVERLAP_MAX_REACH):
                continue
            added = []
            for source in (middle, anchor):
                copy = dict(supports[source])
                copy["run"] = run
                copy["stagger"] = supports[source]["stagger"] + OVERLAP_OFFSET * offset_side
                if copy["portal"] is None:
                    owner = supports[source].get("pole_owner")
                    copy["pole_owner"] = source if owner is None else owner
                copy.pop("anchor", None)
                added.append(len(supports))
                supports.append(copy)
            if at_start:
                runs[run] = added[::-1] + indices
            else:
                runs[run] = indices + added
            extended += 1
    return extended


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


def _space_supports(ground, net, points, cumulative, first, last, portals):
    """Supports along one chain. Each next support goes as far as the curve allows (spans evened
    out towards the chain end); it moves back to the nearest spot with room for a pole, or hangs
    from a portal the track passes under, or a new portal is put up. Adds new portals to
    `portals`. The supports go from arc length `first` to `last`."""

    def crossings():
        found = []
        for portal in portals:
            c = _chain_crossing(points, cumulative, portal)
            if c is not None and first - 2.0 <= c <= last + 2.0:
                found.append((c, portal["id"]))
        return sorted(found)

    def item_at(s, portal):
        p, t = _polyline_at(points, cumulative, s)
        _, radius, turn = _span_at(points, cumulative, s)
        r = signs.right_of(t)
        side = None if portal is not None else _pole_side(net, p, r)
        return {"s": s, "p": p, "t": t, "r": r, "radius": radius, "turn": turn,
                "portal": portal, "side": side}

    def settle(target, lowest):
        """Support at `target` or, moving back no further than `lowest`, the first spot with
        room for a pole; else a new portal at the first spot where one fits; else None."""
        s = target
        while s >= lowest - 1e-6:
            item = item_at(s, None)
            if item["side"] is not None:
                return item
            s -= PORTAL_SEARCH_STEP
        s = target
        while s >= lowest - 1e-6:
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


def _pole_side(net, p, r):
    """+1 / -1: side (right / left of the track) with room for a pole, preferring the side with
    more room; None if neither side has PORTAL_GAP free."""
    room = {}
    for side in (1, -1):
        hits = _crossings(net, p, (r[0] * side, r[1] * side), PORTAL_GAP + 1.0)
        room[side] = hits[0] if hits else 99.0
        spot = (p[0] + r[0] * side * POLE_OFFSET, p[2] + r[1] * side * POLE_OFFSET)
        if _near_signal(net, spot):
            room[side] = 0.0
    best = 1 if room[1] >= room[-1] else -1
    return best if room[best] >= PORTAL_GAP else None


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
        poles.append([x, max(candidates) if candidates else rail_y - 0.6, z])
    centre = [(poles[0][0] + poles[1][0]) / 2, rail_y, (poles[0][2] + poles[1][2]) / 2]
    return {"poles": poles, "rail_y": rail_y, "centre": centre}


def _chain_crossing(points, cumulative, portal):
    """Arc length where a chain crosses the portal line, or None."""
    a, b = portal["poles"]
    for i in range(len(points) - 1):
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
    if abs(item["radius"]) < 1500.0:
        # pulled to the outer side of the curve; turn > 0 = turning left (counter-clockwise)
        outer = -1.0 if item["turn"] > 0 else 1.0       # in units of r (right)
        stagger = STAGGER_ON_CURVE * outer
    else:
        stagger = STAGGER * (1 if index % 2 == 0 else -1)
    support = {"track": [p[0], p[1], p[2]], "t": [t[0], t[1]], "stagger": stagger,
               "side": item["side"], "portal": item.get("portal")}
    if item.get("portal") is None:
        side = item["side"]
        pole_x = p[0] + r[0] * side * POLE_OFFSET
        pole_z = p[2] + r[1] * side * POLE_OFFSET
        candidates = [h for h in ground.heights(pole_x, pole_z) if rail_y - 6.0 <= h <= rail_y + 3.0]
        support["pole"] = [pole_x, max(candidates) if candidates else rail_y - 0.6, pole_z]
        support["pole_base"] = rail_y - POLE_BASE_BELOW_RAIL
        support["pole_owner"] = None
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
    contact = (track[0] + r[0] * sp["stagger"], rail_y + CONTACT_WIRE_HEIGHT,
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

    hinge = from_pole(POLE_RADIUS_TOP + 0.05, CANTILEVER_HINGE)
    tie = from_pole(POLE_RADIUS_TOP + 0.05, TIE_ROD_HINGE)
    top_y = pole_base_y(sp) + POLE_HEIGHT
    return {"r": r, "to_track": to_track, "messenger": messenger, "contact": contact,
            "hinge": hinge, "tie": tie, "top_y": top_y, "from_pole": from_pole}


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
    return ((pole[0] + tx * WIRE_END_OUT, rail_y + ANCHOR_MESSENGER_HEIGHT, pole[2] + tz * WIRE_END_OUT),
            (pole[0] + tx * WIRE_END_OUT, rail_y + ANCHOR_CONTACT_HEIGHT, pole[2] + tz * WIRE_END_OUT))


def _insulator_chain(builder, a, b, anchor, sheds=6):
    """Suspension insulator chain from a to b: a rod with sheds (discs) along it."""
    _tube(builder, a, b, 0.03, 6, COLOR_INSULATOR, anchor, caps=True)
    axis = _unit(_sub(b, a))
    length = math.dist(a, b)
    for i in range(sheds):
        centre = _add(a, _mul(axis, length * (i + 0.5) / sheds))
        _tube(builder, _add(centre, _mul(axis, -0.025)), _add(centre, _mul(axis, 0.025)), 0.12, 8,
              COLOR_INSULATOR, anchor, caps=True)


def _concrete_pole(builder, pole, base_y, top_y, t, origin, anchor):
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
    _cone(builder, (pole[0] - ox, base_y - POLE_BURIED, pole[2] - oz), height + POLE_BURIED,
          POLE_RADIUS_GROUND + 0.007, POLE_RADIUS_TOP, POLE_SIDES, COLOR_CONCRETE, anchor)
    signs.add_box_collision(builder, frame, POLE_RADIUS_GROUND, POLE_RADIUS_GROUND,
                            -POLE_BURIED, height, anchor)


def portal_beam_y(portal):
    return portal["rail_y"] + PORTAL_BEAM_HEIGHT


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
    width, height = PORTAL_BEAM_SIZE
    for pole in (a, b):
        anchor = (pole[0] - ox, pole[2] - oz)
        _concrete_pole(builder, pole, base_y, beam_y + height / 2 + 0.25, along, origin, anchor)
    # beam: a box from pole to pole (past the pole axes by a little)
    start = _add((a[0], beam_y, a[2]), _mul(across, -0.35))
    end = _add((b[0], beam_y, b[2]), _mul(across, 0.35))
    centre = _lerp(start, end, 0.5)
    frame_t = (across[0], across[2])
    frame = signs.Frame(local((centre[0], beam_y - height / 2, centre[2])), frame_t)
    half_length = math.dist(start, end) / 2
    anchor = (centre[0] - ox, centre[2] - oz)
    signs._box(builder, frame, half_length, width / 2, 0.0, height, [(0.0, height, COLOR_STEEL)], anchor)
    builder.polygon([frame.at(-half_length, -width / 2, 0.0), frame.at(half_length, -width / 2, 0.0),
                     frame.at(half_length, width / 2, 0.0), frame.at(-half_length, width / 2, 0.0)],
                    COLOR_STEEL, anchor, (0.0, -1.0, 0.0))


def add_portal_hanger(builder, sp, portal, origin):
    """Wires of one track hung from a portal beam: insulator under the beam, rod down to the
    messenger, a rigid rod from the messenger to the contact wire."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    k = _support_points(sp)
    messenger, contact = k["messenger"], k["contact"]
    anchor = (messenger[0] - ox, messenger[2] - oz)
    beam_bottom = portal_beam_y(portal) - PORTAL_BEAM_SIZE[1] / 2
    top = (messenger[0], beam_bottom, messenger[2])
    insulator_end = (messenger[0], beam_bottom - INSULATOR_LENGTH, messenger[2])
    _tube(builder, local(top), local(insulator_end), INSULATOR_RADIUS, 6, COLOR_INSULATOR, anchor,
          caps=True)
    if insulator_end[1] > messenger[1] + 0.05:
        _tube(builder, local(insulator_end), local(messenger), ROD_RADIUS, 4, COLOR_STEEL, anchor,
              caps=True)
    _tube(builder, local(messenger), local((contact[0], contact[1] + 0.02, contact[2])),
          TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)


def add_support(builder, sp, origin):
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
        _concrete_pole(builder, pole, base_y, base_y + POLE_HEIGHT, sp["t"], origin, anchor)
    if sp.get("anchor") is not None:
        add_anchor(builder, sp, origin)
        return

    # inclined cantilever tube: hinge on the pole -> past the messenger point
    hinge, messenger = k["hinge"], k["messenger"]
    direction = _unit(_sub(messenger, hinge))
    tube_end = _add(messenger, _mul(direction, CANTILEVER_OVERHANG))
    insulator_end = _add(hinge, _mul(direction, INSULATOR_LENGTH))
    _tube(builder, local(hinge), local(insulator_end), INSULATOR_RADIUS, 6, COLOR_INSULATOR, anchor,
          caps=True)
    _tube(builder, local(insulator_end), local(tube_end), TUBE_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    # tie rod: pole -> onto the tube, right past the messenger saddle
    tie, saddle = k["tie"], _add(messenger, _mul(direction, 0.10))
    rod_dir = _unit(_sub(saddle, tie))
    rod_insulator_end = _add(tie, _mul(rod_dir, INSULATOR_LENGTH))
    _tube(builder, local(tie), local(rod_insulator_end), INSULATOR_RADIUS, 6, COLOR_INSULATOR, anchor,
          caps=True)
    _tube(builder, local(rod_insulator_end), local(saddle), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    # registration arm: from the tube down to the contact wire
    span_len = math.dist((hinge[0], hinge[2]), (messenger[0], messenger[2]))
    f = min(0.9, REGISTRATION_ARM_START / max(span_len, 1e-6))
    arm_start = _lerp(hinge, messenger, f)
    arm_start = (arm_start[0], max(arm_start[1], k["contact"][1] + 0.35), arm_start[2])
    contact = k["contact"]
    _tube(builder, local(arm_start), local((contact[0], contact[1] + 0.02, contact[2])),
          TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)


def add_anchor(builder, sp, origin):
    """Compensated anchoring at a pole: insulator chains, yoke, pulley block, rope and weight
    stack along the pole, guy wire (with an insulator) to a ground anchor block behind it."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    pole, rail_y = sp["pole"], sp["track"][1]
    anchor = (pole[0] - ox, pole[2] - oz)
    tx, tz = sp["anchor"]["towards"]

    def out(distance, height):
        return local((pole[0] + tx * distance, rail_y + height, pole[2] + tz * distance))

    across = (-tz, 0.0, tx)                    # horizontal, square to the wires: block axles
    messenger_end, contact_end = wire_ends(sp)
    # insulator chains and the yoke joining them
    yoke_top, yoke_bottom = out(YOKE_OUT, ANCHOR_MESSENGER_HEIGHT), out(YOKE_OUT, ANCHOR_CONTACT_HEIGHT)
    _insulator_chain(builder, yoke_top, local(messenger_end), anchor)
    _insulator_chain(builder, yoke_bottom, local(contact_end), anchor)
    _tube(builder, yoke_top, yoke_bottom, TUBE_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    yoke_middle = out(YOKE_OUT, COMPENSATOR_HEIGHT)
    moving = out(MOVING_BLOCK_OUT, COMPENSATOR_HEIGHT)
    fixed = out(FIXED_BLOCK_OUT, COMPENSATOR_HEIGHT)
    _tube(builder, moving, yoke_middle, ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    # blocks (pulleys) and the rope strands between them
    for centre, radius in ((fixed, FIXED_BLOCK_RADIUS), (moving, MOVING_BLOCK_RADIUS)):
        _tube(builder, _add(centre, _mul(across, -0.03)), _add(centre, _mul(across, 0.03)), radius, 10,
              COLOR_STEEL, anchor, caps=True)
    for side in (1.0, -1.0):
        _tube(builder, _add(fixed, (0.0, side * FIXED_BLOCK_RADIUS, 0.0)),
              _add(moving, (0.0, side * MOVING_BLOCK_RADIUS, 0.0)), DROPPER_RADIUS * 1.4, 4,
              COLOR_STEEL, anchor, caps=True)
    # bracket holding the fixed block on the pole
    _tube(builder, out(0.0, COMPENSATOR_HEIGHT + FIXED_BLOCK_RADIUS + 0.05),
          _add(fixed, (0.0, FIXED_BLOCK_RADIUS + 0.05, 0.0)), ROD_RADIUS * 1.8, 4, COLOR_STEEL,
          anchor, caps=True)
    _tube(builder, _add(fixed, (0.0, FIXED_BLOCK_RADIUS + 0.05, 0.0)), fixed, ROD_RADIUS, 4,
          COLOR_STEEL, anchor, caps=True)
    # rope down along the pole to the weight stack
    weights_bottom = out(FIXED_BLOCK_OUT, WEIGHTS_BOTTOM)
    weights_top = out(FIXED_BLOCK_OUT, WEIGHTS_BOTTOM + WEIGHTS_HEIGHT)
    _tube(builder, _add(fixed, (0.0, -FIXED_BLOCK_RADIUS, 0.0)), weights_top, DROPPER_RADIUS * 1.4, 4,
          COLOR_STEEL, anchor, caps=True)
    _tube(builder, weights_bottom, weights_top, WEIGHTS_RADIUS, 8, COLOR_WEIGHTS, anchor, caps=True)
    for i in range(1, 8):                 # disc joints
        y = WEIGHTS_BOTTOM + WEIGHTS_HEIGHT * i / 8
        _tube(builder, out(FIXED_BLOCK_OUT, y - 0.01), out(FIXED_BLOCK_OUT, y + 0.01),
              WEIGHTS_RADIUS + 0.012, 8, COLOR_STEEL, anchor, caps=False)
    # guy wire with an insulator, and its ground anchor block
    ground = sp["anchor"]["guy_ground"]
    block_side, block_height = GUY_BLOCK
    guy_dir = _unit2((ground[0] - pole[0], ground[2] - pole[2]))
    top = local((pole[0] + guy_dir[0] * 0.15, rail_y + GUY_TOP, pole[2] + guy_dir[1] * 0.15))
    foot = local((ground[0], ground[1] + block_height, ground[2]))
    down = _unit(_sub(top, foot))
    insulator_low = _add(foot, _mul(down, GUY_INSULATOR_FROM_FOOT))
    insulator_high = _add(insulator_low, _mul(down, 0.45))
    _tube(builder, top, insulator_high, GUY_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    _insulator_chain(builder, insulator_high, insulator_low, anchor, sheds=3)
    _tube(builder, insulator_low, foot, GUY_RADIUS * 1.6, 4, COLOR_STEEL, anchor, caps=True)
    frame = signs.Frame(local((ground[0], ground[1], ground[2])), guy_dir)
    signs._box(builder, frame, block_side / 2, block_side / 2, -0.5, block_height,
               [(-0.5, block_height, COLOR_FOUNDATION)], anchor)


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
        _tube(builder, local(top), local((top[0], top[1] - 0.35, top[2])), 0.06, 6,
              COLOR_INSULATOR, anchor, caps=True)


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


def add_span(builder, a, b, origin):
    ox, oz = origin

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


def add_catenary(builder, plan, tile, origin):
    """Geometry of all supports and spans assigned to `tile`."""
    supports = plan["supports"]
    count = 0
    portals = plan.get("portals", [])
    for portal in portals:
        if portal["tile"] == tile:
            add_portal(builder, portal, origin)
            count += 1
    for sp in supports:
        if sp["tile"] == tile:
            if sp["portal"] is None:
                add_support(builder, sp, origin)
                count += 1
            else:
                add_portal_hanger(builder, sp, portals[sp["portal"]], origin)
    for span in plan["spans"]:
        if span["tile"] == tile:
            add_span(builder, supports[span["from"]], supports[span["to"]], origin)
    feeder_poles = plan.get("feeder_poles", [])
    for fp in feeder_poles:
        if fp["tile"] == tile:
            add_feeder_pole(builder, fp, origin)
    for span in plan.get("feeder_spans", []):
        if span["tile"] == tile:
            add_feeder_span(builder, feeder_poles[span["from"]], feeder_poles[span["to"]], origin)
    return count
