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
import json
import math
import os
import re

import rails
import signs
import swmesh

# ---------------------------------------------------------------- dimensions (metres)

CONTACT_WIRE_HEIGHT = 6.50        # above the rail top (series 7.501-1 issue 9: 6500 in the span)
SYSTEM_HEIGHT = 1.80              # messenger above the contact wire at the supports
# Zigzag: on straight track and curves of radius >= ZIGZAG_MIN_RADIUS the contact wire goes
# +-STAGGER from the track axis, alternating support by support (away from the pole: reverse
# fixator, towards the pole: direct fixator). On tighter curves there is no zigzag: the wire is
# moved to the outer side of the curve by half the chord sag, at most CURVE_OFFSET_MAX.
STAGGER = 0.30                    # GOST 32679: 300 +- 100 mm on straight track
ZIGZAG_MIN_RADIUS = 3000.0
CURVE_OFFSET_MAX = 0.35            # on curves without zigzag: wire offset at the supports and mid span
MAX_SPAN = 70.0
MIN_SPAN = 30.0
CURVE_SAG_ALLOWANCE = 0.60        # a span may cut the curve by this much (chord vs track)
CURVE_MIN_SPAN = 12.0             # on tight curves spans get this short, so the wire stays over the
                                  # pantograph (+-CURVE_OFFSET_MAX: half the chord sag either way)
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
POLE_RADIUS_GROUND = 0.225        # 450 mm from the foot up to half the height ...
POLE_RADIUS_TOP = 0.150           # ... then tapering to 300 mm at the top
POLE_TAPER_FROM = 0.5             # fraction of the height where the taper starts
POLE_SIDES = 8

# Cantilever: Soviet uninsulated straight inclined console (series 7.501-1 issue 9). The steel
# tube goes from its heel ("pyata") on the pole up to the clevis ("bugel") above the messenger and
# a little past it; the tie rod holds the tube end from higher up the pole (base 2585 mm between
# heel and tie). The console is earthed: the messenger hangs from the clevis on a string of three
# glass cap-and-pin insulators (PS-70) and a saddle; the fixators leave the tube through their own
# ribbed porcelain rod insulator. Direct fixator FP-25: insulator and main rod 350-450 mm above the
# contact wire, a stand at its end, the steady arm on from the stand to the wire (wire pulled
# towards the pole). Reverse fixator FO-25: insulator 600-800 mm above the wire, the main rod runs
# out past the wire, the steady arm comes back from the stand at its end (wire pulled away). The
# main rod hangs on two support strings from the messenger, 1.5 m either side of the support.
# Heights above the rail top.
CANTILEVER_HINGE = CONTACT_WIRE_HEIGHT - 0.25
CONSOLE_BASE = 2.585              # heel to tie on the pole
TIE_HEIGHT = CANTILEVER_HINGE + CONSOLE_BASE
GARLAND_DISCS = 3                 # PS-70 glass discs in a suspension string (25 kV; 3 kV would be 2)
ANCHOR_DISCS = 4                  # ... in a strain (anchor) string
DISC_RADIUS = 0.127               # PS-70: 255 mm across ...
DISC_PITCH = 0.146                # ... 146 mm a disc along the string
GARLAND_FITTINGS = 0.13           # clevis, links and saddle on a suspension string
GARLAND_LENGTH = GARLAND_DISCS * DISC_PITCH + GARLAND_FITTINGS
CANTILEVER_OVERHANG = 0.18        # the tube runs on past the clevis
FIX_INSULATOR_LENGTH = 0.55       # ribbed porcelain rod insulator at the start of a fixator
FIX_INSULATOR_SHEDS = 5
FIX_SHED_SPACING = 0.85           # ... spread over most of the insulator (bunched up they read as one lump)
DIRECT_ATTACH = 0.40              # FP-25: where it leaves the tube, above the contact wire
DIRECT_MAIN = 0.35                # its main rod at the stand, above the contact wire
REVERSE_ATTACH = 0.70             # FO-25
REVERSE_MAIN_AT_WIRE = 0.45       # its main rod over the wire
MAIN_FIXATOR_BEYOND = 1.10        # the reverse fixator's main rod ends this far past the wire
STAND_DROP = 0.14                 # the fixator stand hangs this far below the main rod
STEADY_ARM_REACH = 0.95           # the direct fixator's stand this far before the wire
STEADY_ARM_BEND = 0.30            # the steady arm bends down this far from the wire
STRING_ALONG = 1.50               # support strings: from the messenger this far either side
POLE_FLAT_FACTOR = 0.88           # fittings start this far out (x pole radius): inside the flats
CLAMP_BAND_HEIGHT = 0.10
SECOND_WIRE_RAISE = 0.20          # a second wire on the same cantilever (turnout) is held this
                                  # much higher (issue 9: anchored branch Ho + 200)
SECOND_ARM_ATTACH = CONTACT_WIRE_HEIGHT + 0.95   # ... from the host's tube, above its fixator
OVERLAP_RAISE_PLAIN = 0.20        # anchored branch at a transition support (no sectioning)
OVERLAP_RAISE_INSULATING = 0.50   # ... with sectioning (issue 9: Ho + 500, wires 550 apart)
TRAVERSE_HALF = 0.60              # two consoles on one pole: on two traverses, 1.2 m apart
TRAVERSE_SIZE = (0.12, 0.10)      # channel 120 x 100 (depth from the pole, height)
GARLAND_FIX_RADIUS = 600.0        # inner side of curves up to this radius: the messenger string is
                                  # held by a fixator (Sh-1) from the tube, so it cannot swing in
SPRING_ALONG = 6.0                # spring stay ("ressornaya struna") on the messenger +- 6 m ...
SPRING_DROPPER = 2.0              # ... with its droppers +- 2 m from the support
SPRING_BELOW = 0.45               # ... hanging this far below the messenger at the support
SPRING_MIN_SPAN = 40.0
SPRING_MIN_RADIUS = 800.0
MIDDLE_ANCHOR_ROPE = 18.0         # middle anchor: rope from the messenger at the middle support
                                  # down to the contact wire 18 m either side (issue 9)
FEEDERS = True                    # DPR line (two wires - rail) on the field side of the poles
FEEDER_ARM = (1.65, 3.30)         # wires out from the pole axis (bracket KFD, issue 9 sheet 39)
FEEDER_BRACKET_DROP = 0.10        # bracket this far below the pole top ...
FEEDER_STRUT_DROP = 0.825         # ... its strut from this far below the bracket
FEEDER_STAND = 0.80               # on a portal: a stand this high on the girder end
TUBE_RADIUS = 0.030
ROD_RADIUS = 0.016
INSULATOR_RADIUS = 0.085
INSULATOR_LENGTH = 0.80
COLOR_GLASS = (74, 112, 100, 255)  # glass cap-and-pin insulators
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
ANCHOR_POLE_SHIFTS = (-6.0, -9.0, -12.0, -4.0, 6.0, 9.0, 12.0)   # an anchor on a shared pole gets its
                                     # own pole this far beyond (away from its span; negative:
                                     # back towards it), on its side of the track or else the other
POLE_SPACING = 2.5                   # poles at least this far apart
ANCHOR_POLE_OUTSET = 0.8             # an own anchor pole before a shared pole: this much further out
GUY_POLE_CLEARANCE = 1.5             # a guy rope keeps this far from other poles

# "DPR" line (two wires - rail, 27.5 kV for non-traction consumers): on the field side of the
# outermost poles of the bundle (portal posts too), straight through, independent of the tracks.
FEEDER_OUTER_CHECK = 15.0            # an outer pole has no electrified track within this outwards
FEEDER_MAX_SPAN = 80.0
FEEDER_SIDE_PENALTY = 4.0            # cost per metre of sideways jump between poles
FEEDER_POLE_BONUS = 6.0              # prefer hanging on every pole of the line
FEEDER_TRACK_CLEARANCE = 2.5         # a DPR span keeps this far from every track axis
FEEDER_GROUND_CLEARANCE = 1.5        # ... and its wires this far above the ground (no wires through hills)
FEEDER_STRUT_IN = 0.35               # the bracket strut meets the beam this far inside the inner wire

# A side track's wire does not end at the turnout: it runs on over the main track for a span
# (beside the main wire, OVERLAP_OFFSET apart) and is anchored on the next pole of the main track.
OVERLAP_OFFSET = 0.20
OVERLAP_FIRST_SPAN = 20.0            # the span onto the main track is at least this long
OVERLAP_MAX_REACH = 160.0            # anchor pole no further than this past the turnout
FALLBACK_ANCHOR_REACH = (12.0, 90.0) # no free pole to hang on beyond: anchored this far beyond

# Signals (placed by the railroad signal playlist, not in the tile files): poles, guy wires and
# their anchors keep this far from them.
SIGNAL_PLAYLIST = "data/missions/default_railroad_signals/playlist.xml"
SIGNAL_CLEARANCE = 1.6
SIGNAL_SHADOW_ALONG = 25.0           # no pole this far in front of a signal (towards the trains) ...
SIGNAL_SHADOW_SIDE = 1.2             # ... nearer than this to the signal's line

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
MESSENGER_ANCHOR_HEIGHT = CONTACT_WIRE_HEIGHT + 2.15
COMPENSATOR_HEIGHT = CONTACT_WIRE_HEIGHT + 0.75   # the anchored contact wire rises to here
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
GUY_DISTANCES = (7.0, 6.0, 5.0, 4.0)  # ... or nearer where there is no room
GUY_TURNS = (0.0, 0.35, 0.7, 1.05, 1.4, -0.35, -0.7, -1.05, 1.75)
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
PORTAL_BEAM_SIZE = (0.40, 0.55)      # (old box beam, not used)
PORTAL_BEAM_BOTTOM = 9.60            # underside of the lattice girder (KS-160.11: Hp = 9600)
PORTAL_TRUSS_HEIGHT = 0.70           # one post per side: girder 0.45 x 0.70 (RTs-22.5)
PORTAL_TWIN_TRUSS_HEIGHT = 1.20      # two posts per side (series 7.501-1 issue 16: 740 x 1200)
PORTAL_PANEL = 0.80                  # panel length of the lattice
PORTAL_CHORD_RADIUS = 0.045
PORTAL_WEB_RADIUS = 0.022
PORTAL_TWIN_SPAN = 15.0              # wider than this: two posts in a row on each side
PORTAL_TWIN_OFFSET = (2 * POLE_RADIUS_GROUND + POLE_RADIUS_GROUND) / 2   # post axes this far from the
                                     # pole spot (along): a gap of half the post diameter between them
PORTAL_HEAD_HEIGHT = 0.90            # head: four steel angles round the post top(s) ...
PORTAL_HEAD_HALF = 0.195             # ... at this distance from the post axis (plate 0.45 m)
PORTAL_ANGLE_LEG = 0.075
PORTAL_ANGLE_THICK = 0.010
PORTAL_HEAD_RINGS = ((0.03, 0.09), (0.17, 0.23))   # two clamp rings at the foot of the angles
PORTAL_HEAD_PLATE = 0.06             # bearing plate on the angles, under the girder
PORTAL_PLATE_MARGIN = 0.03
PORTAL_DECK = 0.04
PORTAL_RAILING = 1.10
PORTAL_RAILING_STEP = 1.50
PORTAL_LADDER_FROM = 0.30            # the lowest rung this far above the ground (a working ladder)
PORTAL_LADDER_ELEMENT_BACK = 0.30    # the game's ladder element this far behind the rungs (towards
                                     # the post): the climbing character then hangs close to them


def PORTAL_LADDER_BRACKETS(foot_y, head_y):
    """Heights of the ladder's stand-offs to the post: near the foot, then about every 2.5 m."""
    count = max(2, int(round((head_y - 0.3 - (foot_y + 0.3)) / 2.5)) + 1)
    return [foot_y + 0.3 + (head_y - 0.6 - foot_y) * i / (count - 1) for i in range(count)]
PORTAL_LADDER_OUT = 0.45             # ladder from the post axis, along the track, outside the girder
PORTAL_LADDER_WIDTH = 0.42
PORTAL_LADDER_STEP = 0.30
PORTAL_LADDER_ABOVE = 1.10           # the stringers run on this far above the deck (hand holds)
MAST_REACH = 2.20                    # console post under a portal girder: this far from the track
MAST_REACHES = (2.20, 2.00, 2.40, 1.80, 2.60)   # ... or a little nearer / further (X = 2200 +- 200)
MAST_SIZE = 0.10                     # square tube 100 x 100 (KS-160.11)
MAST_BELOW_HINGE = 0.15              # it ends this far below the console heel
MAST_STRUT_OUT = 0.71                # strut from the girder this far from the post to its foot
MAST_MIN_GAP = 1.0                   # posts of one portal at least this far apart
MAST_TRACK_CLEARANCE = 1.6           # a post at least this far from any other track axis
COLOR_LATTICE = (104, 106, 108, 255)

# bridges, ledges
FOOTING_LOW = 3.0                    # ground at most this far below the rail: an ordinary pole
FOOTING_HIGH = 3.0
FOOTING_HIGH_GOOD = 2.0              # ground higher than this above the rail buries the pole: a cutting,
                                     # used only if neither a lower spot nor a portal fits
PORTAL_POST_HIGH = 6.5               # a portal post may stand this high on a slope (the girder is at 9.6)
ROAD_COLOUR = (45, 41, 45)           # asphalt in the terrain mesh colours
ROAD_SAMPLES = (-15.0, -10.0, -5.0, 5.0, 10.0, 15.0)   # along the track: asphalt all along = a paved yard
ROAD_YARD_SAMPLES = 3                # ... at least this many of them on asphalt: a yard (a crossing road has 0-1)
ROAD_KEEP_OUT = ((-1808.0, -6744.0, 8.0),)   # (x, z, radius): roads reported in game, not in the colours
FOOTING_LOW_FAR = 6.0                # or this far (tall foundation) / higher, if nothing better
DECK_EDGE_SEARCH = (3.0, 2.7, 2.4, 2.1, 1.8)   # a viaduct deck edge, from the track axis
DECK_LEVEL = (-1.6, 0.3)             # deck surface around the rail top
BRIDGE_GAP = 4.0                     # something this far under the deck: a bridge
BRIDGE_SIDE = 8.0                    # ... or no ground this far to either side (past the deck)
BRACKET_BEAM = (0.50, 0.35)          # steel console under a pole on a viaduct: width, height
BRACKET_DROP = 1.80                  # its strut meets the deck side at most this far below the beam
BRACKET_DROP_DEFAULT = 1.00          # ... where the deck's underside is not known
TRUSS_COLOR = (45, 34, 32)           # the steel of the through-truss bridges in the tile meshes
TRUSS_TOP = (9.0, 14.0)              # truss top members above the rail
TRUSS_WALL = (2.0, 14.0)             # any truss member (not the deck girders)
TRUSS_REACH = 12.0                   # lateral, from the track axis
TRUSS_STEP = 2.0
TRUSS_BEAM_SIZE = (0.25, 0.30)       # cross girder under the top chords: width, height
TRUSS_STRUT_SEARCH = 6.0             # the wires hang from the bridge's own top cross strut nearest
                                     # the support, looked for this far along the track
TRUSS_STRUT_HEIGHTS = (8.8, 14.0)    # ... its underside above the rail
TRUSS_CLAMP = (0.50, 0.14, 0.08)     # channel hook-bolted under the strut: across, along, height
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
OVERLAP_INSULATOR_AT = (1.6, 2.4)    # insulating overlap: insulators in the anchored branch,
                                     # this far from the transition support


class RoadMask:
    """Asphalt in the terrain mesh colours of the mainland tiles (ROAD_COLOUR), from the triangle
    lists `extract_roads` writes (one json per tile, local x/z of each asphalt triangle)."""

    CELL = 8.0

    def __init__(self, road_dir):
        self.road_dir = road_dir
        self.tiles = {}

    def _load(self, tile):
        if tile in self.tiles:
            return self.tiles[tile]
        grid = {}
        path = os.path.join(self.road_dir, tile + ".json") if self.road_dir else None
        if path and os.path.exists(path):
            for tri in json.load(open(path)):
                xs, zs = tri[0::2], tri[1::2]
                for ix in range(int(math.floor(min(xs) / self.CELL)), int(math.floor(max(xs) / self.CELL)) + 1):
                    for iz in range(int(math.floor(min(zs) / self.CELL)), int(math.floor(max(zs) / self.CELL)) + 1):
                        grid.setdefault((ix, iz), []).append(tri)
        self.tiles[tile] = grid
        return grid

    def asphalt(self, world_x, world_z):
        tx = int(math.floor((world_x + 8000 + 500) / 1000))
        tz = int(math.floor((world_z + 12000 + 500) / 1000))
        grid = self._load("mega_island_%d_%d" % (tx, tz))
        x, z = world_x - (tx * 1000 - 8000), world_z - (tz * 1000 - 12000)
        for ax, az, bx, bz, cx, cz in grid.get((int(math.floor(x / self.CELL)), int(math.floor(z / self.CELL))), ()):
            d = (bx - ax) * (cz - az) - (cx - ax) * (bz - az)
            if abs(d) < 1e-12:
                continue
            l1 = ((x - ax) * (cz - az) - (cx - ax) * (z - az)) / d
            l2 = ((bx - ax) * (z - az) - (x - ax) * (bz - az)) / d
            if l1 >= -1e-6 and l2 >= -1e-6 and l1 + l2 <= 1 + 1e-6:
                return True
        return False


def extract_roads(mesh_path):
    """Asphalt triangles of a terrain mesh: [ax, az, bx, bz, cx, cz] (tile-local)."""
    mesh = swmesh.read_mesh_full(mesh_path)
    v, idx = mesh.vertices, mesh.indices
    out = []
    for i in range(0, len(idx) - 2, 3):
        a, b, c = v[idx[i]], v[idx[i + 1]], v[idx[i + 2]]
        if tuple(a[3:6]) == ROAD_COLOUR and tuple(b[3:6]) == ROAD_COLOUR and tuple(c[3:6]) == ROAD_COLOUR:
            out.append([round(a[0], 2), round(a[2], 2), round(b[0], 2), round(b[2], 2), round(c[0], 2), round(c[2], 2)])
    return out


def _on_road(net, x, z, t=None):
    """A pole or guy spot on a road: a reported road, or asphalt that does not run on along the
    track (asphalt along it is a paved yard or platform, where poles do stand)."""
    if any(math.hypot(x - rx, z - rz) < radius for rx, rz, radius in ROAD_KEEP_OUT):
        return True
    roads = getattr(net, "roads", None)
    if roads is None:
        return False
    if not any(roads.asphalt(x + dx, z + dz) for dx, dz in ((0.0, 0.0), (0.4, 0.0), (-0.4, 0.0), (0.0, 0.4), (0.0, -0.4))):
        return False
    if t is None:
        return True
    return sum(1 for d in ROAD_SAMPLES if roads.asphalt(x + t[0] * d, z + t[1] * d)) < ROAD_YARD_SAMPLES


class Trusses:
    """Steel of the through-truss bridges, found by its colour in the tile meshes."""

    CELL = 10.0
    TRI_CELL = 4.0

    def __init__(self, rom_dir):
        self.rom_dir = rom_dir
        self.tiles = {}
        self.triangles = {}

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
                mesh = swmesh.read_mesh_full(mesh_path)
                world = {}
                for n, v in enumerate(mesh.vertices):
                    if (int(v[3]), int(v[4]), int(v[5])) != TRUSS_COLOR:
                        continue
                    x, y, z = signs._transform((v[0], v[1], v[2]), matrix)
                    x, z = x + origin[0], z + origin[1]
                    world[n] = (x, y, z)
                    grid.setdefault((int(math.floor(x / self.CELL)), int(math.floor(z / self.CELL))),
                                    []).append((x, y, z))
                idx = mesh.indices
                for i in range(0, len(idx) - 2, 3):
                    if idx[i] in world and idx[i + 1] in world and idx[i + 2] in world:
                        tri = (world[idx[i]], world[idx[i + 1]], world[idx[i + 2]])
                        x0 = min(q[0] for q in tri)
                        x1 = max(q[0] for q in tri)
                        z0 = min(q[2] for q in tri)
                        z1 = max(q[2] for q in tri)
                        for ix in range(int(math.floor(x0 / self.TRI_CELL)), int(math.floor(x1 / self.TRI_CELL)) + 1):
                            for iz in range(int(math.floor(z0 / self.TRI_CELL)), int(math.floor(z1 / self.TRI_CELL)) + 1):
                                self.triangles.setdefault((ix, iz), []).append(tri)
        self.tiles[tile] = grid
        return grid

    def ray_up(self, x, z):
        """Heights of the truss steel surfaces right above / below a world point."""
        self.near(x, z, 1.0)                      # loads the tile
        hits = []
        for a, b, c in self.triangles.get((int(math.floor(x / self.TRI_CELL)), int(math.floor(z / self.TRI_CELL))), ()):
            d = (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])
            if abs(d) < 1e-12:
                continue
            l1 = ((x - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (z - a[2])) / d
            l2 = ((b[0] - a[0]) * (z - a[2]) - (x - a[0]) * (b[2] - a[2])) / d
            if l1 < -1e-6 or l2 < -1e-6 or l1 + l2 > 1 + 1e-6:
                continue
            hits.append(a[1] + l1 * (b[1] - a[1]) + l2 * (c[1] - a[1]))
        return hits

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


def _truss_strut(trusses, p, t):
    """(offset along the track, underside height) of the bridge's top cross strut nearest the
    track point p (a member of the top wind bracing right over the track axis), or None."""
    offsets = [0.0]
    k = 1
    while k * 0.5 <= TRUSS_STRUT_SEARCH:
        offsets += [k * 0.5, -k * 0.5]
        k += 1
    for a in offsets:
        x, z = p[0] + t[0] * a, p[2] + t[1] * a
        hits = [h for h in trusses.ray_up(x, z) if TRUSS_STRUT_HEIGHTS[0] <= h - p[1] <= TRUSS_STRUT_HEIGHTS[1]]
        if hits:
            # the middle of the strut along the track: step on while it is still overhead
            lo = hi = a
            while hi - a < 2.0 and any(TRUSS_STRUT_HEIGHTS[0] <= h - p[1] <= TRUSS_STRUT_HEIGHTS[1]
                                       for h in trusses.ray_up(p[0] + t[0] * (hi + 0.1), p[2] + t[1] * (hi + 0.1))):
                hi += 0.1
            while a - lo < 2.0 and any(TRUSS_STRUT_HEIGHTS[0] <= h - p[1] <= TRUSS_STRUT_HEIGHTS[1]
                                       for h in trusses.ray_up(p[0] + t[0] * (lo - 0.1), p[2] + t[1] * (lo - 0.1))):
                lo -= 0.1
            mid = (lo + hi) / 2
            x, z = p[0] + t[0] * mid, p[2] + t[1] * mid
            under = [h for h in trusses.ray_up(x, z) if TRUSS_STRUT_HEIGHTS[0] <= h - p[1] <= TRUSS_STRUT_HEIGHTS[1]]
            return mid, (min(under) if under else min(hits))
    return None


def _truss_chord(trusses, p, t, r, side):
    """Outer face of the truss top chord on one side at track point p: (lateral, bottom, top)."""
    top = [(b, h) for _, b, h in trusses.at(p, t, r, 3.0, TRUSS_TOP) if b * side > 1.0]
    if not top:
        return None
    outer = max(b * side for b, _ in top)
    chord = [h for b, h in top if b * side > outer - 0.7]
    low, high = p[1] + min(chord), p[1] + max(chord)
    # the chord's own surfaces just inside its outer face (vertices alone may miss one of them)
    r_side = (r[0] * side, r[1] * side)
    hits = []
    for a in (-1.0, -0.5, 0.0, 0.5, 1.0):
        for inset in (0.1, 0.2):
            x = p[0] + t[0] * a + r_side[0] * (outer - inset)
            z = p[2] + t[1] * a + r_side[1] * (outer - inset)
            hits += [h for h in trusses.ray_up(x, z) if p[1] + TRUSS_TOP[0] <= h <= p[1] + TRUSS_TOP[1]]
    if hits and max(hits) - min(hits) > 0.2:
        low, high = min(hits), max(hits)
    elif high - low < 0.2:
        high = low + 0.5
    return outer, low, high


def _make_truss_portal(trusses, p, t, r):
    """Hanging point in a through truss at track point p: the bridge's own top cross strut (the
    wires hang from it on rods and insulator strings, as on real bridges with the deck below);
    where there is none, a cross girder of our own under the truss top chords."""
    strut = _truss_strut(trusses, p, t)
    if strut is not None:
        a, under = strut
        q = (p[0] + t[0] * a, p[1], p[2] + t[1] * a)
        faces = [_truss_chord(trusses, q, t, r, side) for side in (1.0, -1.0)]
        reach = [f[0] if f else 4.0 for f in faces]
        poles = [[q[0] + r[0] * reach[0], q[1], q[2] + r[1] * reach[0]],
                 [q[0] - r[0] * reach[1], q[1], q[2] - r[1] * reach[1]]]
        centre = [q[0], q[1], q[2]]
        chords = []
        for side, f in zip((1.0, -1.0), faces):
            if f is not None:
                chords.append([q[0] + r[0] * side * f[0], q[2] + r[1] * side * f[0], f[1], f[2]])
        return {"poles": poles, "rail_y": p[1], "centre": centre, "kind": "truss", "strut": True,
                "strut_offset": a, "beam_y": under - TRUSS_CLAMP[2] / 2, "chords": chords}
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
    """World (x, z) of the railroad signals on the mainland, and the (x, z) direction each faces."""
    path = os.path.join(rom_dir, *SIGNAL_PLAYLIST.split("/"))
    if not os.path.exists(path):
        return [], []
    text = open(path, encoding="utf-8", errors="replace").read()
    found, facings = [], []
    for location in re.finditer(r'<l id="\d+" tile="data/tiles/([^"]+)\.xml".*?</l>', text, re.S):
        origin = signs.mainland_origin(location.group(1))
        if origin is None:
            continue
        for zone in re.finditer(r'cargo_zone_import_cargo_type="railroad_signal[^"]*".*?<spawn_transform([^/]*)/>',
                                location.group(0), re.S):
            values = dict(re.findall(r'(\d\d)="([^"]*)"', zone.group(1)))
            found.append((float(values.get("30", 0)) + origin[0], float(values.get("32", 0)) + origin[1]))
            # the signal faces along its local z axis (towards the trains it is for)
            facings.append(_unit2((float(values.get("20", 0)), float(values.get("22", 1)))))
    return found, facings


def _near_signal(net, a, b=None):
    """Is the point a (or the segment a-b), as (x, z), closer than SIGNAL_CLEARANCE to a signal?"""
    for signal in net.signals:
        d = (math.dist(signal, a) if b is None else signs._point_segment_distance(signal, a, b))
        if d < SIGNAL_CLEARANCE:
            return True
    return False


def _shades_signal(net, spot, t=None):
    """Would a pole at `spot` (x, z) stand in front of a signal, in its line along the track,
    where it hides the signal from the drivers coming up to it?"""
    for signal, facing in zip(net.signals, getattr(net, "signal_facings", ())):
        dx, dz = spot[0] - signal[0], spot[1] - signal[1]
        if abs(dx) > SIGNAL_SHADOW_ALONG + 2 or abs(dz) > SIGNAL_SHADOW_ALONG + 2:
            continue
        ahead = dx * facing[0] + dz * facing[1]          # towards the trains
        side = dx * -facing[1] + dz * facing[0]
        if 0.0 < ahead < SIGNAL_SHADOW_ALONG and abs(side) < SIGNAL_SHADOW_SIDE:
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
                  corridor=ROUTE_CORRIDOR, log=print, road_dir=None):
    """Return {"supports": [...], "spans": [...], "portals": [...]} (world coordinates). Each
    item carries the tile whose meshes get its geometry."""
    net = signs.build_network(tiles, prepare_graph)
    net.signals, net.signal_facings = load_signals(rom_dir)
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
    net.roads = RoadMask(road_dir)

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
        dead = _duplicate_portals(portals)
        if len(portals) == count and not dead:
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
    own_anchor_poles, kept_anchors = _anchor_poles(ground, net, runs, supports, portals)
    for run, indices in enumerate(runs):
        for a, b in zip(indices, indices[1:]):
            span = {"tile": supports[a]["tile"], "from": a, "to": b, "run": run}
            if supports[b].get("insulate") and supports[a].get("section_anchor"):
                span["insulators"] = "to"
            elif supports[a].get("insulate") and supports[b].get("section_anchor"):
                span["insulators"] = "from"
            spans.append(span)
    anchors = _mark_anchors(ground, net, runs, supports, portals)
    _assign_stagger(runs, supports, chain_data)
    middle_anchors = _middle_anchors(runs, supports, spans)
    masts = _assign_masts(net, supports, portals)
    _traverse_shared_poles(supports)
    springs = _assign_springs(runs, supports)
    live = {sp["portal"] for sp in supports if sp["portal"] is not None and not sp.get("dropped")}
    for portal in portals:
        portal["tile"] = _tile_of_point(net, portal["centre"])
        if portal["id"] not in live:
            portal["unused"] = True           # its supports were trimmed off
    worst_wire, off_wire = _wire_deviation(chain_data, supports, spans)
    log("contact wire off the track axis: worst %.2f m, spans over %.2f m: %d" % (worst_wire, WIRE_DEVIATION_LIMIT, off_wire))
    for row in sorted(_wire_deviation.details, reverse=True)[:25]:
        log("  off %s" % (row,))
    if FEEDERS:
        feeder_poles, feeder_spans = _plan_feeders(net, chain_data, supports, portals, ground)
    else:
        feeder_poles, feeder_spans = [], []
    log("catenary: route nodes %d, electrified edges %d, wire runs %d, sections %d, supports %d "
        "(on portals %d, on truss girders %d, on viaduct brackets %d, on shared poles %d, anchors %d "
        "(moved to own poles %d), left out %d, trimmed off bridge ends %d), turnout overlaps %d, "
        "section overlaps %d plain + %d insulating, middle anchors %d, spans %d, portals %d "
        "(truss girders %d), DPR spans %d; anchors kept on working poles %d, traverses %d, "
        "console posts %d, spring stays %d, held strings %d"
        % (len(route), len(electrified), len(chain_data), len([r for r in runs if len(r) >= 2]), len(supports),
           sum(1 for sp in supports if sp["portal"] is not None and portals[sp["portal"]].get("kind") != "truss"),
           sum(1 for sp in supports if sp["portal"] is not None and portals[sp["portal"]].get("kind") == "truss"),
           sum(1 for sp in supports if sp.get("bracket") is not None and not sp.get("dropped")),
           shared, anchors, own_anchor_poles, blocked, trimmed,
           overlaps, sections["plain"], sections["insulating"], middle_anchors, len(spans), len(portals),
           sum(1 for portal in portals if portal.get("kind") == "truss"), len(feeder_spans),
           kept_anchors, sum(1 for sp in supports if sp.get("traverse") and not sp.get("dropped")), masts, springs,
           sum(1 for sp in supports if sp.get("garland_fix") and not sp.get("dropped"))))
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
    debug_at = os.environ.get("CAT_DEBUG_AT")
    debug_at = tuple(map(float, debug_at.split(","))) if debug_at else None

    def why(sp, text):
        if debug_at and math.hypot(sp["track"][0] - debug_at[0], sp["track"][2] - debug_at[1]) < debug_at[2]:
            print("extend: run end at (%.1f, %.1f): %s" % (sp["track"][0], sp["track"][2], text))

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
                why(end_support, "no electrified track passes through the end node")
                continue
            plan = None                   # (main run, middle or None, anchor, side of the wire)
            fallback = None
            for main_run, s_node in through:
                main_chain, main_points, main_cumulative, _, _ = chain_data[main_run]
                node_p, t = _polyline_at(main_points, main_cumulative, s_node)
                into = _unit2((node_p[0] - before[0], node_p[2] - before[2]))   # side track -> turnout
                ahead = 1.0 if into[0] * t[0] + into[1] * t[1] > 0 else -1.0
                r = signs.right_of(t)
                offset_side = 1.0 if (before[0] - node_p[0]) * r[0] + (before[2] - node_p[2]) * r[1] > 0 else -1.0
                main = runs[main_run]
                ordered = sorted((i for i in main if (supports[i]["s"] - s_node) * ahead > 0
                                  and supports[i].get("overlap_of") is None),
                                 key=lambda i: (supports[i]["s"] - s_node) * ahead)
                if fallback is None:
                    # if the wire cannot hang on beyond, it is at least anchored beyond the turnout
                    # (on a pole of its own next to the main track), so it runs over the switch
                    for i in ordered:
                        if (supports[i]["portal"] is None and supports[i].get("bracket") is None
                                and FALLBACK_ANCHOR_REACH[0] <= abs(supports[i]["s"] - s_node) <= FALLBACK_ANCHOR_REACH[1]):
                            fallback = (main_run, None, i, offset_side)
                            break
                beyond = [i for i in ordered
                          if math.dist((supports[i]["track"][0], supports[i]["track"][2]),
                                       (end_support["track"][0], end_support["track"][2])) >= OVERLAP_FIRST_SPAN]
                beyond = [i for i in beyond if i not in used_middles
                          and not any(o.get("pole_owner") == i for o in supports)]   # nor one sharing its pole
                if len(beyond) < 2:
                    why(end_support, "run %d: fewer than 2 free main supports beyond" % main_run)
                    continue
                middle, anchor = beyond[0], beyond[1]
                if (supports[anchor]["portal"] is not None or supports[anchor].get("bracket") is not None
                        or supports[middle].get("bracket") is not None
                        or abs(supports[anchor]["s"] - s_node) > OVERLAP_MAX_REACH):
                    why(end_support, "run %d: anchor support unusable (reach %.0f)" % (
                        main_run, abs(supports[anchor]["s"] - s_node)))
                    continue
                plan = (main_run, middle, anchor, offset_side)
                break
            if plan is None:
                plan = fallback
                why(end_support, "fallback anchor beyond the turnout: %s" % (fallback is not None))
            if plan is None:
                continue
            main_run, middle, anchor, offset_side = plan
            if middle is not None:
                used_middles.add(middle)
            added = []
            for source in ((middle, anchor) if middle is not None else (anchor,)):
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
                              (supports[added[0]]["track"][0], supports[added[0]]["track"][2]))
            if (middle is not None and len(indices) > 2 and from_turnout <= JUNCTION_DROP
                    and reach <= MAX_SPAN * 1.25
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
            # where the line runs off the mainland, back to land; elsewhere only off consoles
            # and truss girders (an anchor needs a pole on the ground)
            to_land = edge_ends is None or (k, at_end) in edge_ends
            while indices:
                i = indices[-1] if at_end else indices[0]
                sp = supports[i]
                off = off_land(sp) if to_land else (
                    sp.get("bracket") is not None
                    or (sp["portal"] is not None and portals[sp["portal"]].get("kind") == "truss"))
                if not off:
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
            raised = OVERLAP_RAISE_INSULATING if insulating else OVERLAP_RAISE_PLAIN
            a_part.append(copy_of(ids[n - 2], insulate=insulating, own_console=True, overlap_raise=raised))
            a_part.append(copy_of(ids[n - 1], section_anchor=True))
            sections.append(a_part)
            prefix = [copy_of(ids[0], section_anchor=True),
                      copy_of(ids[1], insulate=insulating, own_console=True, overlap_raise=raised)]
            if insulating:
                prefix.append(copy_of(ids[2], insulating_mid=True, overlap_gap=INSULATING_GAP, overlap_raise=0.0,
                                      own_console=True))
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
    """Middle anchor of every long section: at the support nearest its middle, a rope from the
    messenger down to the contact wire MIDDLE_ANCHOR_ROPE either side (issue 9, sheet 31)."""
    count = 0
    for indices in runs:
        plain = [i for i in indices if supports[i].get("overlap_of") is None and supports[i].get("anchor") is None]
        if len(plain) < 6:
            continue
        length = supports[plain[-1]]["s"] - supports[plain[0]]["s"]
        if abs(length) < MIDDLE_ANCHOR_MIN:
            continue
        middle = (supports[plain[0]]["s"] + supports[plain[-1]]["s"]) / 2
        plain_set = set(plain)
        best = None
        for k in range(1, len(indices) - 1):
            i = indices[k]
            if not (i in plain_set and indices[k - 1] in plain_set and indices[k + 1] in plain_set):
                continue
            if supports[i].get("rider") or supports[i].get("traverse"):
                continue
            d = abs(supports[i]["s"] - middle)
            if best is None or d < best[0]:
                best = (d, i)
        if best is not None:
            supports[best[1]]["middle_anchor"] = True
            count += 1
    return count


def _assign_masts(net, supports, portals):
    """Console posts under the portal girders (KS-160.11): each track's post MAST_REACH from it,
    on the side of the nearer portal post if there is room (clear of the other tracks and posts),
    else on the other side, else none (the wires then hang on a string under the girder). The
    fixator type follows from which side of the wire the post is. Returns the number of posts."""
    by_portal = {}
    for i, sp in enumerate(supports):
        if sp["portal"] is not None and not sp.get("dropped") and sp.get("overlap_of") is None:
            portal = portals[sp["portal"]]
            if portal.get("kind") != "truss":
                by_portal.setdefault(sp["portal"], []).append(i)
    count = 0
    for pid, members in sorted(by_portal.items()):
        a, b = portals[pid]["poles"]
        span = math.hypot(b[0] - a[0], b[2] - a[2])
        across = _unit2((b[0] - a[0], b[2] - a[2]))
        placed = []
        for i in sorted(members, key=lambda i: supports[i]["s"]):
            sp = supports[i]
            p = sp["track"]
            u = (p[0] - a[0]) * across[0] + (p[2] - a[2]) * across[1]
            sides = (-1.0, 1.0) if u < span / 2 else (1.0, -1.0)
            options = [(reach, side) for reach in MAST_REACHES for side in sides]
            for reach, side in options:
                mx, mz = p[0] + across[0] * side * reach, p[2] + across[1] * side * reach
                mu = (mx - a[0]) * across[0] + (mz - a[2]) * across[1]
                if not (0.6 <= mu <= span - 0.6):
                    continue
                if any(math.hypot(mx - x, mz - z) < MAST_MIN_GAP for x, z in placed):
                    continue
                if _track_distance(net, mx, mz) < MAST_TRACK_CLEARANCE:
                    continue
                sp["mast"] = [mx, mz]
                placed.append((mx, mz))
                r = signs.right_of(tuple(sp["t"]))
                mast_side = 1.0 if (mx - p[0]) * r[0] + (mz - p[2]) * r[1] > 0 else -1.0
                if sp.get("fixator") != "overlap":
                    sp["fixator"] = "direct" if sp["stagger"] * mast_side > 0 else "reverse"
                count += 1
                break
    return count


def _traverse_shared_poles(supports):
    """A pole carrying the consoles of two tracks (a shared pole): the consoles go on two
    traverses along the track, like at a transition support."""
    consoles = {}
    for i, sp in enumerate(supports):
        if sp.get("dropped") or sp["portal"] is not None or sp.get("anchor") is not None:
            continue
        if sp.get("fixator") == "overlap":
            continue
        owner = sp.get("pole_owner") if sp.get("pole_owner") is not None else i
        consoles.setdefault(owner, []).append(i)
    for owner, members in consoles.items():
        if len(members) != 2 or supports[owner].get("traverse"):
            continue
        first, second = sorted(members, key=lambda i: supports[i]["s"])
        t = supports[owner]["t"]
        for i, sign in ((first, -1.0), (second, 1.0)):
            sp = supports[i]
            # along the owner's track direction, mapped into this track's direction
            same = sp["t"][0] * t[0] + sp["t"][1] * t[1] >= 0
            sp["console_shift"] = sign * TRAVERSE_HALF * (1.0 if same else -1.0)
        supports[owner]["traverse"] = True


def _assign_springs(runs, supports):
    """Spring stays at the working supports with spans of SPRING_MIN_SPAN and more, not on tight
    curves, not on anchored branches or second wires (issue 9, sheet 32)."""
    count = 0
    for indices in runs:
        for k in range(1, len(indices) - 1):
            sp = supports[indices[k]]
            if (sp.get("anchor") is not None or sp.get("overlap_of") is not None or sp.get("rider")
                    or sp.get("fixator") == "overlap" or sp.get("middle_anchor")
                    or sp.get("radius", 1e9) <= SPRING_MIN_RADIUS):
                continue
            lengths = [math.hypot(supports[j]["track"][0] - sp["track"][0], supports[j]["track"][2] - sp["track"][2])
                       for j in (indices[k - 1], indices[k + 1])]
            if min(lengths) < SPRING_MIN_SPAN:
                continue
            sp["spring"] = True
            count += 1
    return count


def _pole_spots(supports, portals):
    spots = [(sp["pole"][0], sp["pole"][2]) for sp in supports
             if sp["portal"] is None and not sp.get("dropped") and sp.get("pole_owner") is None]
    spots += [(p[0], p[2]) for portal in portals for p in portal["poles"]]
    return spots


def _find_guy(ground, net, sp, pole, towards, spots):
    """Ground anchor for the guy of an anchor pole: straight back from the span, or turned away
    from the track until it keeps clear of every track, pole, signal and roof. Returns
    (good, gx, gz, floor)."""

    def near_pole(a, b):
        for x, z in spots:
            if math.hypot(x - a[0], z - a[1]) < 0.3:
                continue                     # its own pole
            if (min(a[0], b[0]) - 2 < x < max(a[0], b[0]) + 2 and min(a[1], b[1]) - 2 < z < max(a[1], b[1]) + 2
                    and signs._point_segment_distance((x, z), a, b) < GUY_POLE_CLEARANCE):
                return True
        return False

    out = _unit2((pole[0] - sp["track"][0], pole[2] - sp["track"][2]))
    rail_y = sp["track"][1]
    best = None
    for distance in GUY_DISTANCES:
        for turn in GUY_TURNS:
            back = (-towards[0] * math.cos(turn) + out[0] * math.sin(turn),
                    -towards[1] * math.cos(turn) + out[1] * math.sin(turn))
            gx = pole[0] + back[0] * distance
            gz = pole[2] + back[1] * distance
            heights = ground.heights(gx, gz)
            candidates = [h for h in heights if rail_y - 8.0 <= h <= rail_y + 4.0]
            floor = max(candidates) if candidates else None
            roofed = floor is not None and any(floor + 1.0 < h < floor + 15.0 for h in heights)
            clearance = _track_distance(net, gx, gz)
            crossing = (_crosses_track(net.all_tracks, (pole[0], pole[2]), (gx, gz))
                        or near_pole((pole[0], pole[2]), (gx, gz)))
            good = (clearance >= GUY_CLEARANCE and not roofed and floor is not None and not crossing
                    and not _near_signal(net, (pole[0], pole[2]), (gx, gz)) and not _on_road(net, gx, gz))
            # otherwise keep the least bad: on the ground, clear of the tracks
            score = (good, floor is not None, not crossing, not roofed, min(clearance, GUY_CLEARANCE), -distance)
            if best is None or score > best[0]:
                best = (score, gx, gz, floor)
            if good:
                break
        if best[0][0]:
            break
    return best[0][0], best[1], best[2], best[3]


GUY_KEPT_TURN = 0.75                 # an anchor kept on a working pole: its guy at most ~43 deg aside
BRANCH_POLE_CLEARANCE = 0.8          # the anchored branch keeps this far from other poles


def _branch_clear(start, pole, spots):
    """The anchored branch from the last working support (track point `start`) to `pole` passes no
    other pole."""
    a, b = (start[0], start[2]), (pole[0], pole[2])
    for x, z in spots:
        if math.hypot(x - b[0], z - b[1]) < 0.3 or math.hypot(x - a[0], z - a[1]) < 4.0:
            continue
        if (min(a[0], b[0]) - 2 < x < max(a[0], b[0]) + 2 and min(a[1], b[1]) - 2 < z < max(a[1], b[1]) + 2
                and signs._point_segment_distance((x, z), a, b) < BRANCH_POLE_CLEARANCE):
            return False
    return True


def _anchor_alternates(supports, end, neighbour, ends, claimed):
    """Other tracks' working poles (own pole, one console) on the same side within 15 m before the
    end of a run, between it and its last working support: nearest the end first."""
    e, n = supports[end]["track"], supports[neighbour]["track"]
    d = _unit2((e[0] - n[0], e[2] - n[2]))
    length = (e[0] - n[0]) * d[0] + (e[2] - n[2]) * d[1]
    pole = supports[end]["pole"]
    side = (pole[0] - e[0]) * -d[1] + (pole[2] - e[2]) * d[0]
    found = []
    carrying = {x["pole_owner"] for x in supports if x.get("pole_owner") is not None and not x.get("dropped")}
    for i, o in enumerate(supports):
        if (i in (end, neighbour) or i in ends or i in claimed or o.get("dropped") or o["portal"] is not None
                or o.get("pole_owner") is not None or o.get("anchor") is not None or o.get("fixator") == "overlap"
                or o.get("bracket") is not None or not o.get("pole")):
            continue
        if i in carrying:
            continue
        q = o["pole"]
        along = (q[0] - n[0]) * d[0] + (q[2] - n[2]) * d[1]
        lateral = (q[0] - n[0]) * -d[1] + (q[2] - n[2]) * d[0]
        if not (max(10.0, length - 15.0) <= along <= length + 0.5):
            continue
        if lateral * side <= 0 or not (2.0 <= abs(lateral) <= 6.0):
            continue
        found.append((length - along, i))
    return [i for _, i in sorted(found)]


def _anchor_poles(ground, net, runs, supports, portals):
    """Where a wire run ends on a pole that also carries another track's or section's working
    console (overlaps, a turnout wire running on over the main track), it is anchored right on
    that pole, as on a real line, if the pole stands on the ground, carries no other anchoring or
    second wire, and its guy finds clear ground. Otherwise the anchor gets a pole of its own a few
    metres away on the same side (the pulley block, weights and guys would end up among the other
    consoles). Returns (anchors moved to own poles, anchors kept on working poles)."""
    moved = kept = 0
    ends = set()
    for indices in runs:
        if len(indices) >= 2:
            ends.add(indices[0])
            ends.add(indices[-1])

    def users_of(owner):
        return [i for i, o in enumerate(supports) if o.get("pole_owner") == owner and not o.get("dropped")]
    claimed = set()                         # poles that already carry a kept anchor
    for indices in runs:
        if len(indices) < 2:
            continue
        for end, neighbour in ((indices[0], indices[1]), (indices[-1], indices[-2])):
            sp = supports[end]
            if sp["portal"] is not None:
                continue
            sharers = users_of(end)
            if sp.get("pole_owner") is None and not sharers:
                continue
            # the pole and the working support on it
            if sp.get("pole_owner") is not None:
                owner, working = sp["pole_owner"], sp["pole_owner"]
                others = [i for i in users_of(owner) if i != end]
            else:
                owner, working = end, (sharers[0] if len(sharers) == 1 else None)
                others = sharers[1:]
            host = supports[owner]
            pole = host["pole"]

            def can_carry(i, w):
                h = supports[i]
                return (w is not None and i not in claimed and h.get("bracket") is None and not h.get("dropped")
                        and w not in ends and not supports[w].get("traverse")
                        and supports[w].get("overlap_of") is None and not supports[w].get("own_console"))

            # the pole the run ends at, or another track's working pole just before it on the same
            # side (the anchored branch would run through that one on its way)
            options = []
            if not others and can_carry(owner, working):
                options.append((owner, working))
            if sp.get("pole_owner") is not None or not sharers:
                options += [(i, i) for i in _anchor_alternates(supports, end, neighbour, ends, claimed)
                            if can_carry(i, i)]
            spots = _pole_spots(supports, portals)
            done = False
            for carrier, carrier_working in options:
                cpole = supports[carrier]["pole"]
                towards = _unit2((supports[neighbour]["track"][0] - cpole[0], supports[neighbour]["track"][2] - cpole[2]))
                if not _branch_clear(supports[neighbour]["track"], cpole, spots):
                    continue
                good, gx, gz, floor = _find_guy(ground, net, sp, cpole, towards, spots)
                back = _unit2((gx - cpole[0], gz - cpole[2]))
                if not good or -(back[0] * towards[0] + back[1] * towards[1]) < math.cos(GUY_KEPT_TURN):
                    continue                 # a guy turned far aside does not hold the anchoring
                claimed.add(carrier)
                sp["anchor_on_working"] = True
                if carrier != owner:
                    sp["pole_owner"] = carrier
                    sp["pole"] = list(cpole)
                    sp["pole_base"] = supports[carrier]["pole_base"]
                elif carrier_working != end:
                    supports[carrier_working]["rider"] = False
                kept += 1
                done = True
                break
            if done:
                continue
            track = sp["track"]
            other = supports[neighbour]["track"]
            beyond = _unit2((track[0] - other[0], track[2] - other[2]))
            lateral = (sp["pole"][0] - track[0], sp["pole"][2] - track[2])
            spots = _pole_spots(supports, portals)
            # Before the shared pole (on the span side) first: the anchored branch then ends before
            # it and does not run through that pole's cantilever. Such an anchor pole stands a
            # little further out, so its guy (pointing back past the shared pole) clears it.
            reach = math.hypot(*lateral)
            outset = (reach + ANCHOR_POLE_OUTSET) / reach if reach > 1e-6 else 1.0
            options = [(shift, 1.0) for shift in ANCHOR_POLE_SHIFTS] + [(shift, -1.0) for shift in ANCHOR_POLE_SHIFTS]
            for shift, mirror in options:
                widen = outset if shift < 0 else 1.0
                x = track[0] + lateral[0] * mirror * widen + beyond[0] * shift
                z = track[2] + lateral[1] * mirror * widen + beyond[1] * shift
                if (_track_distance(net, x, z) < POLE_CLEARANCE or _near_signal(net, (x, z))
                        or _shades_signal(net, (x, z), sp["t"]) or _on_road(net, x, z, sp["t"])):
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
    return moved, kept


def _polyline_sag(points, cumulative, s0, s1):
    """Largest distance of the track between arc lengths s0 and s1 from the straight line."""
    pa = _polyline_at(points, cumulative, s0)[0]
    pb = _polyline_at(points, cumulative, s1)[0]
    d = _unit2((pb[0] - pa[0], pb[2] - pa[2]))
    worst = 0.0
    for k in range(1, 8):
        q = _polyline_at(points, cumulative, s0 + (s1 - s0) * k / 8.0)[0]
        worst = max(worst, abs((q[0] - pa[0]) * -d[1] + (q[2] - pa[2]) * d[0]))
    return worst


def _chord_sag(chain_data, a, b):
    """How far the track curves away from the straight line between supports a and b of one wire
    run (towards the inside of the curve, metres; 0 if they are not on one chain)."""
    if (chain_data is None or a.get("run") is None or a.get("run") != b.get("run")
            or a["run"] >= len(chain_data) or "s" not in a or "s" not in b):
        return None
    _, points, cumulative, _, _ = chain_data[a["run"]]
    pa, pb = a["track"], b["track"]
    d = _unit2((pb[0] - pa[0], pb[2] - pa[2]))
    worst = 0.0
    for k in range(1, 8):
        q = _polyline_at(points, cumulative, a["s"] + (b["s"] - a["s"]) * k / 8.0)[0]
        worst = max(worst, abs((q[0] - pa[0]) * -d[1] + (q[2] - pa[2]) * d[0]))
    return worst


WIRE_DEVIATION_LIMIT = 0.40


def _wire_deviation(chain_data, supports, spans):
    """(worst lateral distance of the contact wire from the track axis over working spans, number
    of spans beyond WIRE_DEVIATION_LIMIT): a check, the pantograph is 2 x 0.6 m wide."""
    worst, count = 0.0, 0
    for span in spans:
        a, b = supports[span["from"]], supports[span["to"]]
        if (a.get("anchor") is not None or b.get("anchor") is not None or a.get("dropped") or b.get("dropped")
                or a.get("overlap_of") is not None or b.get("overlap_of") is not None
                or a.get("fixator") == "overlap" or b.get("fixator") == "overlap"
                or a.get("run") != b.get("run") or a.get("run", 99999) >= len(chain_data)):
            continue
        _, points, cumulative, _, _ = chain_data[a["run"]]
        ca, cb = _support_points(a)["contact"], _support_points(b)["contact"]
        here = 0.0
        for k in range(0, 11):
            f = k / 10.0
            # the track point level with the wire point (along the track) and the wire's offset from it
            w = (ca[0] + (cb[0] - ca[0]) * f, ca[2] + (cb[2] - ca[2]) * f)
            sq = a["s"] + (b["s"] - a["s"]) * f
            for _ in range(3):
                q, tq = _polyline_at(points, cumulative, sq)
                sq += (w[0] - q[0]) * tq[0] + (w[1] - q[2]) * tq[1]
            q, tq = _polyline_at(points, cumulative, sq)
            here = max(here, abs((w[0] - q[0]) * tq[1] - (w[1] - q[2]) * tq[0]))
        worst = max(worst, here)
        count += here > WIRE_DEVIATION_LIMIT
        if here > WIRE_DEVIATION_LIMIT:
            _wire_deviation.details.append((round(here, 2), round(min(a.get("radius", 1e9), b.get("radius", 1e9))),
                                            round(math.dist((a["track"][0], a["track"][2]), (b["track"][0], b["track"][2])), 1),
                                            round(a["stagger"], 2), round(b["stagger"], 2), a.get("fixator"), b.get("fixator"),
                                            bool(a.get("console_shift")), a["portal"] is not None,
                                            [round(a["track"][0]), round(a["track"][2])]))
    return worst, count


_wire_deviation.details = []


def _assign_stagger(runs, supports, chain_data=None):
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
                # next to a curve without zigzag: towards that curve's outside, or the wire would
                # cut across the inside of the curve in the span between
                position = indices.index(index)
                for j in (position - 1, position + 1):
                    if 0 <= j < len(indices):
                        o = supports[indices[j]]
                        if (o.get("anchor") is None and o.get("overlap_of") is None
                                and o.get("radius", 1e9) < ZIGZAG_MIN_RADIUS):
                            ro = signs.right_of(tuple(o["t"]))
                            outer_o = 1.0 if o.get("turn", 0.0) > 0 else -1.0
                            side = outer_o * (ro[0] * r[0] + ro[1] * r[1])
                            sp["stagger"] = (1.0 if side > 0 else -1.0) * STAGGER
                            break
            else:
                lengths, sags = [], []
                position = indices.index(index)
                for j in (position - 1, position + 1):
                    if 0 <= j < len(indices):
                        o = supports[indices[j]]
                        lengths.append(math.hypot(o["track"][0] - sp["track"][0], o["track"][2] - sp["track"][2]))
                        sag = _chord_sag(chain_data, sp, o)
                        if sag is not None:
                            sags.append(sag)
                span = sum(lengths) / len(lengths) if lengths else MAX_SPAN
                outer = 1.0 if sp.get("turn", 0.0) > 0 else -1.0      # in units of r (right): a left turn (turn > 0) bends towards -r
                # half the chord sag of the spans either side: the wire is as far outward at the
                # support as it is inward at mid span
                half = (max(sags) / 2.0) if sags else span * span / (16.0 * radius)
                sp["stagger"] = outer * min(CURVE_OFFSET_MAX, half)
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
                    centre = 0.0
                    if source.get("radius", 1e9) < ZIGZAG_MIN_RADIUS:
                        # on a curve the pair sits a little outward (both wires within 0.4 m)
                        centre = (1.0 if source.get("turn", 0.0) > 0 else -1.0) * (0.40 - INSULATING_GAP / 2)
                    source["stagger"] = centre + pole_side * INSULATING_GAP / 2
                    source["fixator"] = "direct"
                sp["stagger"] = (source.get("stagger", 0.0)
                                 + sp.get("overlap_gap", OVERLAP_OFFSET) * sp["overlap_side"])
                sp["fixator"] = "overlap"
                sp["raise"] = sp.get("overlap_raise", SECOND_WIRE_RAISE)
                if sp.get("own_console") and sp["portal"] is None:
                    # transition support: each branch on its own console, the two consoles on
                    # two traverses along the track (series 7.501-1 issue 9, sheets 17-28)
                    r = signs.right_of(tuple(sp["t"]))
                    pole_side = 1.0 if ((sp["pole"][0] - sp["track"][0]) * r[0] +
                                        (sp["pole"][2] - sp["track"][2]) * r[1]) > 0 else -1.0
                    sp["fixator"] = "direct" if sp["stagger"] * pole_side > 0 else "reverse"
                    sp["console_shift"] = TRAVERSE_HALF
                    source["console_shift"] = -TRAVERSE_HALF
                    supports[sp["pole_owner"]]["traverse"] = True
                    if sp.get("insulate") or sp.get("insulating_mid"):
                        sp["garland_fix"] = source["garland_fix"] = True
            elif sp.get("anchor") is None and sp["portal"] is None and sp.get("fixator") in ("direct", "reverse"):
                # inner side of a tight curve: the messenger string is held by a fixator
                r = signs.right_of(tuple(sp["t"]))
                pole_side = 1.0 if ((sp["pole"][0] - sp["track"][0]) * r[0] +
                                    (sp["pole"][2] - sp["track"][2]) * r[1]) > 0 else -1.0
                outer = 1.0 if sp.get("turn", 0.0) > 0 else -1.0
                if sp.get("radius", 1e9) <= GARLAND_FIX_RADIUS and pole_side == -outer:
                    sp["garland_fix"] = True


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


def _mark_anchors(ground, net, runs, supports, portals=()):
    """The first and last support of every wire run with a pole become anchor supports."""
    count = 0
    spots = _pole_spots(supports, portals)
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
            _, gx, gz, floor = _find_guy(ground, net, sp, pole, towards, spots)
            sp["anchor"] = {"towards": list(towards),
                            "guy_ground": [gx, floor if floor is not None else sp["track"][1] - 0.8, gz]}
            count += 1
    return count


def _plan_feeders(net, chain_data, supports, portals, ground=None):
    """DPR line (two wires - rail) along the outer side of the electrified tracks: on brackets on
    the outermost poles of the bundle, on stands over the portal girder ends and on the sides of
    the through trusses. Each support links to the nearest suitable one ahead and behind along
    its track (pole on the same side, no electrified track crossed, little sideways jump); the
    mutual links form the lines, their ends are dead-ended on strain strings."""
    electrified = signs.SegmentIndex()
    for chain, points, cumulative, first, last in chain_data:
        for i in range(len(points) - 1):
            if cumulative[i + 1] >= first - END_GAP and cumulative[i] <= last + END_GAP:
                electrified.add((points[i][0], points[i][2]), (points[i + 1][0], points[i + 1][2]), None)

    cands = []
    for sp in supports:
        if sp["portal"] is not None or sp.get("pole_owner") is not None or sp.get("dropped"):
            continue
        pole = sp["pole"]
        out = _unit2((pole[0] - sp["track"][0], pole[2] - sp["track"][2]))
        anchor = sp.get("anchor")
        if anchor is not None:
            # an anchor pole only if its guy keeps off the field side, where the DPR runs
            g = anchor["guy_ground"]
            if (g[0] - pole[0]) * out[0] + (g[2] - pole[2]) * out[1] > 0.8:
                continue
        base_y = pole_base_y(sp)
        cands.append({"pos": [pole[0], pole[2]], "base_y": base_y, "out": list(out), "t": list(sp["t"]),
                      "tile": sp["tile"], "kind": "pole", "bracket_y": base_y + POLE_HEIGHT - FEEDER_BRACKET_DROP,
                      "root_out": pole_radius_at(base_y, base_y + POLE_HEIGHT - FEEDER_BRACKET_DROP),
                      "rail_y": sp["track"][1]})
    for portal in portals:
        if portal.get("unused"):
            continue
        a, b = portal["poles"]
        across = _unit2((b[0] - a[0], b[2] - a[2]))
        along = [across[1], -across[0]]
        for post, sign in ((a, -1.0), (b, 1.0)):
            out = [across[0] * sign, across[1] * sign]
            if portal.get("kind") == "truss":
                # on the outer face of the truss top chord (feeders run outside the trusses)
                chord = None
                for c in portal.get("chords", ()):
                    if (c[0] - portal["centre"][0]) * out[0] + (c[1] - portal["centre"][2]) * out[1] > 0:
                        chord = c
                if chord is None:
                    continue
                pos = [chord[0], chord[1]]
                bracket_y = chord[3] - 0.04
                extra = {"kind": "truss", "root_out": 0.0, "chord_low": max(chord[2], chord[3] - 0.7)}
            else:
                top = portal_beam_y(portal) + portal_beam_size(portal)[1] / 2 + (PORTAL_DECK if portal_twin(portal) else 0.0)
                pos = [post[0], post[2]]
                bracket_y = top + FEEDER_STAND
                extra = {"kind": "portal", "root_out": 0.0, "stand_from": top,
                         "girder_width": portal_beam_size(portal)[0]}
            fp = {"pos": pos, "base_y": portal["rail_y"] - POLE_BASE_BELOW_RAIL, "out": out, "t": along,
                  "tile": portal.get("tile") or _tile_of_point(net, (pos[0], 0.0, pos[1])),
                  "bracket_y": bracket_y, "rail_y": portal["rail_y"]}
            fp.update(extra)
            cands.append(fp)

    # outer supports only: no electrified track out on their field side
    outer = []
    for fp in cands:
        x, z = fp["pos"]
        hit = False
        for sa, sb, _ in electrified.near(x, z, FEEDER_OUTER_CHECK):
            if signs._ray_hits_segment((x, z), tuple(fp["out"]), FEEDER_OUTER_CHECK, sa, sb) is not None:
                hit = True
                break
        if not hit:
            outer.append(fp)
    grid = {}
    for i, fp in enumerate(outer):
        grid.setdefault((int(fp["pos"][0] // 50), int(fp["pos"][1] // 50)), []).append(i)

    def clear_of_ground(fp, q):
        """Both wires of the span stay FEEDER_GROUND_CLEARANCE above the ground (no hills)."""
        if ground is None:
            return True
        length = math.dist(fp["pos"], q["pos"])
        sag = FEEDER_SAG_70M * (length / 70.0) ** 2
        pa, pb = feeder_points(fp), feeder_points(q)
        if math.dist(pa[0], pb[0]) + math.dist(pa[1], pb[1]) > math.dist(pa[0], pb[1]) + math.dist(pa[1], pb[0]):
            pb = pb[::-1]
        for a, b in zip(pa, pb):
            for k in range(1, 20):
                f = k / 20.0
                w = _lerp(a, b, f)
                y = w[1] - 4.0 * sag * f * (1.0 - f)
                heights = ground.heights(w[0], w[2])
                # in a hill: ground at the wire, or no ground under it (the wire is inside the
                # terrain); a bridge or rock high overhead with ground below is fine
                if (any(y - FEEDER_GROUND_CLEARANCE < h < y + 1.0 for h in heights)
                        or (heights and not any(h <= y - FEEDER_GROUND_CLEARANCE for h in heights))):
                    return False                 # (no surface at all: over water)
        return True

    def best_link(i, direction):
        fp = outer[i]
        x, z = fp["pos"]
        t = fp["t"]
        found = []
        cx, cz = int(x // 50), int(z // 50)
        for gx in range(cx - 2, cx + 3):
            for gz in range(cz - 2, cz + 3):
                for j in grid.get((gx, gz), ()):
                    if j == i:
                        continue
                    q = outer[j]
                    dx, dz = q["pos"][0] - x, q["pos"][1] - z
                    along = (dx * t[0] + dz * t[1]) * direction
                    lateral = dx * fp["out"][0] + dz * fp["out"][1]
                    if not (15.0 <= along <= FEEDER_MAX_SPAN):
                        continue
                    if abs(lateral) > 0.2 * along + 1.5:
                        continue
                    if fp["out"][0] * q["out"][0] + fp["out"][1] * q["out"][1] < 0.3:
                        continue
                    if abs(q["bracket_y"] - fp["bracket_y"]) > 5.0:
                        continue
                    found.append((along + FEEDER_SIDE_PENALTY * abs(lateral), j))
        found.sort()
        for rank, (_, j) in enumerate(found):
            a, b = fp["pos"], outer[j]["pos"]
            if _crosses_track(net.all_tracks, tuple(a), tuple(b)):
                return None
            if any(_track_distance(net, a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f) < FEEDER_TRACK_CLEARANCE
                   for f in (0.25, 0.5, 0.75)):
                return None
            if clear_of_ground(fp, outer[j]):
                return j
            if rank >= 2:
                break                        # through a hill: the line ends here
        return None

    picks = [(best_link(i, 1.0), best_link(i, -1.0)) for i in range(len(outer))]
    edges = set()
    for i, (f, b) in enumerate(picks):
        for j in (f, b):
            if j is not None and i in picks[j]:
                edges.add((min(i, j), max(i, j)))
    neighbours = {}
    for i, j in edges:
        neighbours.setdefault(i, []).append(j)
        neighbours.setdefault(j, []).append(i)
    # lines: connected pieces with at least three supports
    seen = set()
    feeder_poles, feeder_spans = [], []
    index_of = {}
    for start in sorted(neighbours):
        if start in seen:
            continue
        stack, piece = [start], []
        seen.add(start)
        while stack:
            i = stack.pop()
            piece.append(i)
            for j in neighbours[i]:
                if j not in seen:
                    seen.add(j)
                    stack.append(j)
        if len(piece) < 3:
            continue
        for i in piece:
            fp = {key: outer[i][key] for key in ("pos", "base_y", "out", "tile", "kind", "bracket_y", "root_out")}
            if "chord_low" in outer[i]:
                fp["chord_low"] = outer[i]["chord_low"]
                fp["t"] = list(outer[i]["t"])
            if "stand_from" in outer[i]:
                fp["stand_from"] = outer[i]["stand_from"]
                fp["girder_width"] = outer[i]["girder_width"]
                fp["t"] = list(outer[i]["t"])
            if len(neighbours[i]) == 1:
                q = outer[neighbours[i][0]]["pos"]
                fp["end"] = list(_unit2((q[0] - fp["pos"][0], q[1] - fp["pos"][1])))
            index_of[i] = len(feeder_poles)
            feeder_poles.append(fp)
        for i in piece:
            for j in neighbours[i]:
                if i < j:
                    feeder_spans.append({"tile": outer[i]["tile"], "from": index_of[i], "to": index_of[j]})
    return feeder_poles, feeder_spans


def _span_at(points, cumulative, s):
    """Span length allowed around arc length s (from the curve radius), and the turn there."""
    _, t_ahead = _polyline_at(points, cumulative, s + 30.0)
    _, t_back = _polyline_at(points, cumulative, s - 30.0)
    turn = _turn(t_back, t_ahead)
    radius = 60.0 / abs(turn) if abs(turn) > 1e-4 else 1e9
    span = min(MAX_SPAN, max(MIN_SPAN, math.sqrt(8.0 * radius * CURVE_SAG_ALLOWANCE)))
    if radius < ZIGZAG_MIN_RADIUS:
        # no zigzag: offsets outward at the supports, inward at mid span, each half the chord sag
        span = min(MAX_SPAN, max(CURVE_MIN_SPAN, math.sqrt(8.0 * radius * 2.0 * CURVE_OFFSET_MAX * 0.9)))
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
            if portal.get("dead"):
                continue
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
        return item_at(min(last, s + portal.get("strut_offset", 0.0)), portal["id"])

    def settle(target, lowest):
        """Support at `target` or, moving back no further than `lowest`, the first spot with
        room for a pole (or under a truss top: a girder to hang from); else a new portal at the
        first spot where one fits; else None."""
        s = target
        cutting = None
        while s >= lowest - 1e-6:
            if trusses is not None and under_truss_top(s):
                item = truss_item(s)
                if item is not None:
                    return item
            item = item_at(s, None)
            if item["side"] is not None:
                if item["footing"]["rank"] != 1.5:
                    return item
                if cutting is None or item["footing"]["cut"] < cutting["footing"]["cut"]:
                    cutting = item
            s -= PORTAL_SEARCH_STEP
        # a portal another track of the bundle has just put up close by is used instead of a new one
        near = [c for c in crossings() if lowest <= c[0] <= target + PORTAL_DUPLICATE]
        if near:
            c, portal_id = min(near, key=lambda c: abs(c[0] - target))
            return item_at(min(c, last), portal_id)
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
        return cutting

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
        # the chord must not cut the curve by more than twice the allowed wire offset
        while target - s > CURVE_MIN_SPAN and _polyline_sag(points, cumulative, s, target) > 2.0 * CURVE_OFFSET_MAX:
            target -= 2.0
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
    near = [h for h in heights if rail_y - FOOTING_LOW_FAR <= h <= rail_y + 15.0]
    above = [h for h in heights if h >= rail_y - FOOTING_LOW_FAR]
    if above and min(above) > rail_y + FOOTING_HIGH_GOOD:
        return 1.5, min(above) - rail_y      # a cutting: the slope would bury the pole (last resort);
                                             # "edge" carries how high the ground is
    if any(rail_y - FOOTING_LOW <= h <= rail_y + FOOTING_HIGH for h in heights):
        return 3, None
    if near:
        return 2, None                       # a slope down: tall foundation
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
        if (_near_signal(net, spot) or _shades_signal(net, spot, (r[1] * -1.0, r[0]))
                or _on_road(net, spot[0], spot[1], (-r[1], r[0]))):
            room = 0.0
        if room < PORTAL_GAP:
            continue
        rank, edge = _footing(ground, p, r, side) if ground is not None else (3, None)
        if rank == 0:
            continue
        if rank == 1.5:
            key = (rank, -edge, room)            # the cutting side where the ground is lower
            footing = {"rank": rank, "edge": None, "cut": edge}
        else:
            key = (rank, 0.0, room)
            footing = {"rank": rank, "edge": edge}
        if best is None or key > best[0]:
            best = (key, side, footing)
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
        candidates = [h for h in ground.heights(x, z) if rail_y - 6.0 <= h <= rail_y + PORTAL_POST_HIGH]
        if not candidates:
            return None                      # a portal post needs ground (not over a viaduct)
        if _shades_signal(net, (x, z), (-r[1], r[0])) or _on_road(net, x, z, (-r[1], r[0])):
            return None
        poles.append([x, max(candidates), z])
    centre = [(poles[0][0] + poles[1][0]) / 2, rail_y, (poles[0][2] + poles[1][2]) / 2]
    return {"poles": poles, "rail_y": rail_y, "centre": centre}


CROSS_CELL = 50.0
PORTAL_DUPLICATE = 15.0              # two portals over the same tracks closer than this: the newer goes


def _duplicate_portals(portals):
    """Portals put up by two tracks of a bundle at nearly the same place (each track found no pole
    spot before the other's portal existed): the newer one is dropped, and the next planning round
    hangs both tracks on the older. Returns the number dropped."""
    dropped = 0
    live = [p for p in portals if not p.get("dead") and p.get("kind") != "truss"]
    for i, a in enumerate(live):
        if a.get("dead"):
            continue
        for b in live[i + 1:]:
            if b.get("dead"):
                continue
            if math.dist((a["centre"][0], a["centre"][2]), (b["centre"][0], b["centre"][2])) < PORTAL_DUPLICATE:
                b["dead"] = True
                dropped += 1
    return dropped


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
            # the strut and its plate reach down the deck side, not below the deck
            ex = p[0] + r[0] * side * (footing["edge"] - 0.2)
            ez = p[2] + r[1] * side * (footing["edge"] - 0.2)
            deck = sorted((h for h in ground.heights(ex, ez) if rail_y - 4.0 <= h <= rail_y + 0.5), reverse=True)
            bottom = None
            for h in deck[1:]:
                if deck[0] - h > 0.4:
                    bottom = h
                    break
            drop = (support["pole_base"] - bottom - 0.15) if bottom is not None else BRACKET_DROP_DEFAULT
            support["bracket_drop"] = max(BRACKET_BEAM[1] + 0.25, min(BRACKET_DROP, drop))
    return support


# ---------------------------------------------------------------- geometry

CAP_MIN_RADIUS = 0.03


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
    if caps and radius >= CAP_MIN_RADIUS:       # ends of thin rods are not seen: no caps, fewer vertices
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


def pole_radius_at(sp_base_y, y, height=POLE_HEIGHT):
    """Radius of the concrete pole (base sp_base_y, `height` tall) at height y: straight up to
    POLE_TAPER_FROM of its height, then tapering to the top."""
    taper = sp_base_y + height * POLE_TAPER_FROM
    f = max(0.0, min(1.0, (y - taper) / (height * (1.0 - POLE_TAPER_FROM))))
    return POLE_RADIUS_GROUND * (1 - f) + POLE_RADIUS_TOP * f


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
    taper = height * POLE_TAPER_FROM
    _cone(builder, (pole[0] - ox, base_y - buried, pole[2] - oz), taper + buried,
          POLE_RADIUS_GROUND, POLE_RADIUS_GROUND, POLE_SIDES, COLOR_CONCRETE, anchor)
    _cone(builder, (pole[0] - ox, base_y + taper, pole[2] - oz), height - taper,
          POLE_RADIUS_GROUND, POLE_RADIUS_TOP, POLE_SIDES, COLOR_CONCRETE, anchor)
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
    drop = sp.get("bracket_drop", BRACKET_DROP)
    strut_foot = (track[0] + out[0] * edge - ox, base_y - drop + 0.1, track[2] + out[1] * edge - oz)
    _tube(builder, strut_top, strut_foot, 0.06, 4, COLOR_STEEL, anchor, caps=True)
    # plate on the deck side where the beam and the strut are bolted on
    plate = signs.Frame((track[0] + out[0] * edge - ox, base_y - drop, track[2] + out[1] * edge - oz), out)
    signs._box(builder, plate, 0.03, width / 2 + 0.05, 0.0, drop - height * 0.5,
               [(0.0, drop - height * 0.5, COLOR_STEEL)], anchor)
    base = signs.Frame((pole[0] - ox, base_y, pole[2] - oz), out)
    signs._box(builder, base, 0.32, 0.32, 0.0, 0.05, [(0.0, 0.05, COLOR_STEEL)], anchor)


def portal_beam_y(portal):
    """Centre height of the cross beam (truss: of the cross girder under its top chords)."""
    if portal.get("kind") == "truss":
        return portal.get("beam_y", portal["rail_y"] + PORTAL_BEAM_HEIGHT)
    return portal["rail_y"] + PORTAL_BEAM_BOTTOM + portal_beam_size(portal)[1] / 2


def portal_twin(portal):
    """Wide portals stand on two posts in a row (along the tracks) on each side."""
    a, b = portal["poles"]
    return math.hypot(b[0] - a[0], b[2] - a[2]) > PORTAL_TWIN_SPAN


def portal_head_half(portal):
    """(along the tracks, across) half size of a portal head's bearing plate."""
    along = (PORTAL_TWIN_OFFSET if portal_twin(portal) else 0.0) + PORTAL_HEAD_HALF + PORTAL_PLATE_MARGIN
    return along, PORTAL_HEAD_HALF + PORTAL_PLATE_MARGIN


def portal_beam_size(portal):
    """(width, height) of the girder: as wide as the heads (along the tracks)."""
    if portal.get("kind") == "truss":
        return (TRUSS_CLAMP[1], TRUSS_CLAMP[2]) if portal.get("strut") else TRUSS_BEAM_SIZE
    height = PORTAL_TWIN_TRUSS_HEIGHT if portal_twin(portal) else PORTAL_TRUSS_HEIGHT
    return 2 * portal_head_half(portal)[0], height


def portal_ladder(portal):
    """The ladder of a wide portal (world): the first post of side a, on its end face (along the
    track, away from the other post). Returns {"post", "foot" (rung plane centre at the lowest rung
    level), "ground", "x" (along the rungs), "z" (from the post towards the climber), "top" (deck
    top)}, or None."""
    if portal.get("kind") == "truss" or not portal_twin(portal):
        return None
    a, b = portal["poles"]
    across = _unit((b[0] - a[0], 0.0, b[2] - a[2]))
    along = (across[2], -across[0])
    out = (-along[0], 0.0, -along[1])                    # away from the other post
    post = (a[0] - along[0] * PORTAL_TWIN_OFFSET, a[2] - along[1] * PORTAL_TWIN_OFFSET)
    base_y = portal["rail_y"] - POLE_BASE_BELOW_RAIL
    ground = max(base_y, a[1])
    width, height = portal_beam_size(portal)
    top = portal_beam_y(portal) + height / 2 + PORTAL_DECK
    foot = (post[0] + out[0] * PORTAL_LADDER_OUT, ground + PORTAL_LADDER_FROM, post[1] + out[2] * PORTAL_LADDER_OUT)
    return {"post": post, "foot": foot, "ground": ground, "x": across, "z": out, "top": top}


def portal_ladders(plan):
    """Game ladder elements for the portal ladders: [(world point at the ladder foot on the ground,
    x axis, z axis, height)]. The element stands in the rung plane; its z axis points from the
    structure towards the climber (as the vanilla railway ladders)."""
    found = []
    for portal in plan.get("portals", []):
        if portal.get("unused"):
            continue
        ladder = portal_ladder(portal)
        if ladder is None:
            continue
        back = PORTAL_LADDER_ELEMENT_BACK
        foot = (ladder["foot"][0] - ladder["z"][0] * back, ladder["ground"], ladder["foot"][2] - ladder["z"][2] * back)
        found.append((foot, ladder["x"], ladder["z"], ladder["top"] - ladder["ground"]))
    return found


def _lattice_beam(builder, start, end, width, height, color, anchor):
    """Box lattice girder from start to end (centre line): four chords, verticals and diagonals on
    the side faces, zigzag bracing on the top and bottom faces."""
    axis = _unit(_sub(end, start))
    side = _unit(_cross((0.0, 1.0, 0.0), axis))      # horizontal, across the girder
    length = math.dist(start, end)
    panels = max(2, int(round(length / PORTAL_PANEL)))
    step = length / panels

    def at(x, y, z):
        """x along the girder from start, y up from the centre line, z across."""
        return _add(_add(_add(start, _mul(axis, x)), (0.0, y, 0.0)), _mul(side, z))

    h, w = height / 2, width / 2
    for y in (-h, h):
        for z in (-w, w):
            _tube(builder, at(0.0, y, z), at(length, y, z), PORTAL_CHORD_RADIUS, 4, color, anchor, caps=True)
    for i in range(panels + 1):
        x = i * step
        for z in (-w, w):
            _tube(builder, at(x, -h, z), at(x, h, z), PORTAL_WEB_RADIUS, 4, color, anchor)
        for y in (-h, h):
            _tube(builder, at(x, y, -w), at(x, y, w), PORTAL_WEB_RADIUS, 4, color, anchor)
        if i == panels:
            break
        x1 = x + step
        flip = i % 2 == 0
        for z in (-w, w):
            _tube(builder, at(x, -h if flip else h, z), at(x1, h if flip else -h, z), PORTAL_WEB_RADIUS, 4,
                  color, anchor)
        for y in (-h, h):
            _tube(builder, at(x, y, -w if flip else w), at(x1, y, w if flip else -w), PORTAL_WEB_RADIUS, 4,
                  color, anchor)


def add_portal(builder, portal, origin):
    """Rigid cross-span: concrete posts beyond the outer tracks with steel heads, a lattice girder
    resting on the heads. Wider than PORTAL_TWIN_SPAN: two posts in a row on each side, a wider
    girder with a deck and railings on top and a ladder up one post. In a through truss: only a
    cross girder under the truss top chords."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    a, b = portal["poles"]
    across = _unit((b[0] - a[0], 0.0, b[2] - a[2]))
    along = (across[2], -across[0])                 # horizontal, along the tracks
    along3 = (along[0], 0.0, along[1])
    beam_y = portal_beam_y(portal)
    width, height = portal_beam_size(portal)
    frame_t = (across[0], across[2])
    if portal.get("kind") == "truss" and portal.get("strut"):
        return                                   # the bridge's own strut; clamps come with the hangers
    if portal.get("kind") == "truss":
        centre = _lerp((a[0], beam_y, a[2]), (b[0], beam_y, b[2]), 0.5)
        frame = signs.Frame(local((centre[0], beam_y - height / 2, centre[2])), frame_t)
        half_length = math.dist(a, b) / 2
        anchor = (centre[0] - ox, centre[2] - oz)
        signs._box(builder, frame, half_length, width / 2, 0.0, height, [(0.0, height, COLOR_TRUSS)], anchor)
        builder.polygon([frame.at(-half_length, -width / 2, 0.0), frame.at(half_length, -width / 2, 0.0),
                         frame.at(half_length, width / 2, 0.0), frame.at(-half_length, width / 2, 0.0)],
                        COLOR_TRUSS, anchor, (0.0, -1.0, 0.0))
        signs.add_box_collision(builder, frame, half_length, width / 2, 0.0, height, anchor)
        return

    twin = portal_twin(portal)
    rail_y = portal["rail_y"]
    base_y = rail_y - POLE_BASE_BELOW_RAIL
    bottom = rail_y + PORTAL_BEAM_BOTTOM               # underside of the girder = top of the plates
    plate_y = bottom - PORTAL_HEAD_PLATE
    head_y = plate_y - PORTAL_HEAD_HEIGHT
    post_top = plate_y - 0.05
    offsets = (-PORTAL_TWIN_OFFSET, PORTAL_TWIN_OFFSET) if twin else (0.0,)
    plate_along, plate_across = portal_head_half(portal)
    corner_along = plate_along - PORTAL_PLATE_MARGIN     # outer corner of the angles
    corner_across = plate_across - PORTAL_PLATE_MARGIN
    leg, thick = PORTAL_ANGLE_LEG, PORTAL_ANGLE_THICK

    for pole in (a, b):
        anchor = (pole[0] - ox, pole[2] - oz)

        def block(c_along, c_across, half_along, half_across, y0, y1, bottom_face=False):
            frame = signs.Frame(local((pole[0] + along[0] * c_along + across[0] * c_across, y0,
                                       pole[2] + along[1] * c_along + across[2] * c_across)), along)
            # Frame: t = along the tracks, r = right of it (= -across or +across): symmetric boxes
            signs._box(builder, frame, half_along, half_across, 0.0, y1 - y0, [(0.0, y1 - y0, COLOR_STEEL)], anchor)
            if bottom_face:
                builder.polygon([frame.at(-half_along, -half_across, 0.0), frame.at(half_along, -half_across, 0.0),
                                 frame.at(half_along, half_across, 0.0), frame.at(-half_along, half_across, 0.0)],
                                COLOR_STEEL, anchor, (0.0, -1.0, 0.0))

        for offset in offsets:
            post = [pole[0] + along[0] * offset, pole[1], pole[2] + along[1] * offset]
            _concrete_pole(builder, post, base_y, post_top, along, origin, (post[0] - ox, post[2] - oz))
        # head: four steel angles at the corners round the post top(s), held by two clamp rings
        # at their foot, the bearing plate on top
        for sa in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                block(sa * (corner_along - leg / 2), sc * (corner_across - thick / 2), leg / 2, thick / 2,
                      head_y, plate_y)
                block(sa * (corner_along - thick / 2), sc * (corner_across - leg / 2), thick / 2, leg / 2,
                      head_y, plate_y)
        for r0, r1 in PORTAL_HEAD_RINGS:
            band = 0.012
            for sc in (-1.0, 1.0):
                block(0.0, sc * (corner_across + band / 2), corner_along + band, band / 2,
                      head_y + r0, head_y + r1, True)
            for sa in (-1.0, 1.0):
                block(sa * (corner_along + band / 2), 0.0, band / 2, corner_across, head_y + r0, head_y + r1, True)
        block(0.0, 0.0, plate_along, plate_across, plate_y, bottom, True)
        # collider: the whole head as one box
        signs.add_box_collision(builder, signs.Frame(local((pole[0], head_y, pole[2])), along),
                                plate_along, plate_across, 0.0, bottom - head_y, anchor)

    # girder: from plate edge to plate edge, as wide as the plates
    start = _add((a[0], beam_y, a[2]), _mul(across, -plate_across))
    end = _add((b[0], beam_y, b[2]), _mul(across, plate_across))
    centre = _lerp(start, end, 0.5)
    anchor = (centre[0] - ox, centre[2] - oz)
    _lattice_beam(builder, local(start), local(end), width, height, COLOR_LATTICE, anchor)
    # collider: one solid box for the girder (and the deck on it)
    girder = signs.Frame(local((centre[0], bottom, centre[2])), frame_t)
    signs.add_box_collision(builder, girder, math.dist(start, end) / 2, width / 2, 0.0,
                            height + (PORTAL_DECK if twin else 0.0), anchor)
    if not twin:
        return
    # deck and railings on top (with an opening where the ladder comes up)
    top = beam_y + height / 2
    deck = signs.Frame(local((centre[0], top, centre[2])), frame_t)
    half_length = math.dist(start, end) / 2
    signs._box(builder, deck, half_length, width / 2, 0.0, PORTAL_DECK, [(0.0, PORTAL_DECK, COLOR_STEEL)], anchor)
    builder.polygon([deck.at(-half_length, -width / 2, 0.0), deck.at(half_length, -width / 2, 0.0),
                     deck.at(half_length, width / 2, 0.0), deck.at(-half_length, width / 2, 0.0)],
                    COLOR_STEEL, anchor, (0.0, -1.0, 0.0))
    # the deck frame's side axis r points along +along; the ladder stands on the -along side, at
    # the first post of side a
    ladder_x = -half_length + plate_across                 # pole a, along the girder
    gap = (ladder_x - PORTAL_LADDER_WIDTH / 2 - 0.15, ladder_x + PORTAL_LADDER_WIDTH / 2 + 0.15)
    rail_top = top + PORTAL_DECK + PORTAL_RAILING
    edge = width / 2 - 0.03
    for side in (-1.0, 1.0):
        pieces = [(-half_length + 0.03, half_length - 0.03)]
        if side < 0:
            pieces = [(-half_length + 0.03, gap[0]), (gap[1], half_length - 0.03)]
        for x0, x1 in pieces:
            if x1 - x0 < 0.1:
                continue
            posts = max(2, int(round((x1 - x0) / PORTAL_RAILING_STEP)) + 1)
            for i in range(posts):
                foot = deck.at(x0 + (x1 - x0) * i / (posts - 1), side * edge, PORTAL_DECK)
                _tube(builder, foot, (foot[0], rail_top, foot[2]), 0.022, 4, COLOR_STEEL, anchor, caps=True)
            for y in (rail_top, top + PORTAL_DECK + PORTAL_RAILING / 2):
                p0, p1 = deck.at(x0, side * edge, 0.0), deck.at(x1, side * edge, 0.0)
                _tube(builder, (p0[0], y, p0[2]), (p1[0], y, p1[2]), 0.022, 4, COLOR_STEEL, anchor, caps=True)
            # collider: a thin box along the railing, so nobody walks off the deck through it
            middle = deck.at((x0 + x1) / 2, side * edge, 0.0)
            signs.add_box_collision(builder, signs.Frame(middle, frame_t), (x1 - x0) / 2, 0.025,
                                    PORTAL_DECK, PORTAL_DECK + PORTAL_RAILING, anchor)
    # ladder (no safety hoops) on the end face of the first post, along the track, outside the
    # girder: the climber comes up beside the deck and steps onto it through the opening
    ladder = portal_ladder(portal)
    post, out = ladder["post"], ladder["z"]
    foot_y = ladder["foot"][1]
    top_y = top + PORTAL_DECK + PORTAL_LADDER_ABOVE
    ladder_anchor = (post[0] - ox, post[1] - oz)
    for s_ in (-PORTAL_LADDER_WIDTH / 2, PORTAL_LADDER_WIDTH / 2):
        x = post[0] + out[0] * PORTAL_LADDER_OUT + across[0] * s_
        z = post[1] + out[2] * PORTAL_LADDER_OUT + across[2] * s_
        _tube(builder, local((x, foot_y, z)), local((x, top_y, z)), 0.025, 4, COLOR_STEEL, ladder_anchor, caps=True)
        # stand-offs: from each stringer in to the post, ending on its surface (the stringers are
        # wider apart than the post near its top, so they converge onto the post's face)
        for y in PORTAL_LADDER_BRACKETS(foot_y, head_y):
            radius = pole_radius_at(base_y, y, post_top - base_y)
            lateral = math.copysign(radius * 0.5, s_)
            depth = math.sqrt(radius * radius - lateral * lateral) * 0.97
            _tube(builder, local((x, y, z)),
                  local((post[0] + out[0] * depth + across[0] * lateral, y,
                         post[1] + out[2] * depth + across[2] * lateral)),
                  0.018, 4, COLOR_STEEL, ladder_anchor, caps=True)
    # clamp bands round the post where the stand-offs meet it
    for y in PORTAL_LADDER_BRACKETS(foot_y, head_y):
        band = pole_radius_at(base_y, y, post_top - base_y) + 0.012
        _tube(builder, local((post[0], y - 0.04, post[1])), local((post[0], y + 0.04, post[1])), band,
              POLE_SIDES, COLOR_STEEL, ladder_anchor, caps=True)
    y = foot_y + 0.15
    while y < top + PORTAL_DECK + 0.1:
        c = (post[0] + out[0] * PORTAL_LADDER_OUT, y, post[1] + out[2] * PORTAL_LADDER_OUT)
        _tube(builder, local(_add(c, _mul(across, -PORTAL_LADDER_WIDTH / 2))),
              local(_add(c, _mul(across, PORTAL_LADDER_WIDTH / 2))), 0.016, 4, COLOR_STEEL, ladder_anchor)
        y += PORTAL_LADDER_STEP


def _messenger_offset(sp):
    """Lateral position (units of r) of the messenger at a support: over the track axis on
    straight track and wide curves (the contact wire zigzags under it), over the contact wire on
    tighter curves and wherever a second wire or branch shares the support."""
    if (sp.get("radius", 1e9) >= ZIGZAG_MIN_RADIUS and sp.get("overlap_of") is None
            and not sp.get("rider") and not sp.get("traverse") and sp.get("fixator") != "overlap"):
        return 0.0
    return sp["stagger"]


def _support_points(sp):
    """Key points of one support (world): the wires over the track, and for a console (on a pole,
    or on a console post under a portal girder) its heel, tie, clevis and fixator points."""
    tx, tz = sp["t"]
    r = signs.right_of((tx, tz))
    track = sp["track"]
    shift = sp.get("console_shift", 0.0)
    if shift:
        # a console on a traverse holds its wires right in front of it: the consoles stay parallel
        track = (track[0] + tx * shift, track[1], track[2] + tz * shift)
    rail_y = track[1]
    wire_y = rail_y + CONTACT_WIRE_HEIGHT + sp.get("raise", 0.0)
    contact = (track[0] + r[0] * sp["stagger"], wire_y, track[2] + r[1] * sp["stagger"])
    m_off = _messenger_offset(sp)
    messenger = (track[0] + r[0] * m_off, rail_y + CONTACT_WIRE_HEIGHT + SYSTEM_HEIGHT, track[2] + r[1] * m_off)
    mast = sp.get("mast")
    if sp["portal"] is not None and mast is None:
        return {"r": r, "messenger": messenger, "contact": contact, "t": (tx, 0.0, tz)}
    if mast is not None:
        axis_x, axis_z = mast[0], mast[1]

        def radius(y):
            return MAST_SIZE / 2
    else:
        base_y = pole_base_y(sp)
        axis_x, axis_z = sp["pole"][0], sp["pole"][2]

        def radius(y):
            return pole_radius_at(base_y, y) * POLE_FLAT_FACTOR
    if shift:
        # on a traverse beside the pole: the heel and the tie sit on its outer face
        axis_x += tx * shift
        axis_z += tz * shift
        base_radius = radius

        def radius(y):
            return base_radius(y) / POLE_FLAT_FACTOR + TRAVERSE_SIZE[0]
    towards = _unit2((messenger[0] - axis_x, messenger[2] - axis_z))
    to_track = (towards[0], 0.0, towards[1])

    def from_pole(dist_out, height):
        return (axis_x + to_track[0] * dist_out, rail_y + height, axis_z + to_track[2] * dist_out)

    hinge = from_pole(radius(rail_y + CANTILEVER_HINGE), CANTILEVER_HINGE)
    tie = from_pole(radius(rail_y + TIE_HEIGHT), TIE_HEIGHT)
    clevis = (messenger[0], messenger[1] + GARLAND_LENGTH, messenger[2])
    tube_dir = _unit(_sub(clevis, hinge))

    def on_tube(y):
        """Point of the console tube at height y (world)."""
        f = (y - hinge[1]) / max(1e-6, clevis[1] - hinge[1])
        return _lerp(hinge, clevis, f)

    contact_out = math.hypot(contact[0] - axis_x, contact[2] - axis_z)
    return {"r": r, "to_track": to_track, "messenger": messenger, "contact": contact,
            "hinge": hinge, "tie": tie, "clevis": clevis, "tube_dir": tube_dir, "on_tube": on_tube,
            "contact_out": contact_out, "from_pole": from_pole, "axis": (axis_x, axis_z),
            "t": (tx, 0.0, tz)}


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


def _insulator_chain(builder, a, b, anchor, sheds=3, shed_radius=0.11, core_radius=0.03,
                     spacing=SHED_SPACING_FACTOR):
    """Ribbed porcelain rod insulator from a to b: the sheds sit close together in the middle on
    the insulator body, steel fittings run on to both ends."""
    axis = _unit(_sub(b, a))
    length = math.dist(a, b)
    pitch = length / sheds * spacing
    body = min(length, sheds * pitch + 0.04)
    body_start = _add(a, _mul(axis, (length - body) / 2))
    body_end = _add(body_start, _mul(axis, body))
    _tube(builder, a, b, core_radius * 0.6, 4, COLOR_STEEL, anchor)
    _tube(builder, body_start, body_end, core_radius, 6, COLOR_INSULATOR, anchor)
    for i in range(sheds):
        centre = _add(body_start, _mul(axis, 0.02 + pitch * (i + 0.5)))
        half = min(0.022, pitch * 0.25)       # sheds stay apart: a gap at least as wide as a shed
        _tube(builder, _add(centre, _mul(axis, -half)), _add(centre, _mul(axis, half)), shed_radius, 6,
              COLOR_INSULATOR, anchor, caps=True)


def _disc_string(builder, a, b, anchor, discs=GARLAND_DISCS, radius=DISC_RADIUS):
    """String of glass cap-and-pin insulators (PS-70) from a (the fixed end) to b: a steel cap
    and a glass disc each, one after the other, steel links at both ends."""
    axis = _unit(_sub(b, a))
    length = math.dist(a, b)
    body = min(length * 0.9, discs * DISC_PITCH)
    start = _add(a, _mul(axis, (length - body) / 2))
    end = _add(start, _mul(axis, body))
    _tube(builder, a, b, 0.016, 4, COLOR_STEEL, anchor)       # links, pins and caps read as one steel rod
    pitch = body / discs
    for i in range(discs):
        cap0 = _add(start, _mul(axis, pitch * i))
        _tube(builder, _add(cap0, _mul(axis, pitch * 0.42)), _add(cap0, _mul(axis, pitch * 0.62)), radius, 6,
              COLOR_GLASS, anchor, caps=True)


def _saddle(builder, messenger, t, anchor, local):
    """Messenger saddle at the foot of a suspension string."""
    half = (t[0] * 0.13, 0.0, t[2] * 0.13)
    _tube(builder, local(_add(_sub(messenger, half), (0.0, 0.03, 0.0))),
          local(_add(_add(messenger, half), (0.0, 0.03, 0.0))), 0.032, 6, COLOR_STEEL, anchor, caps=True)


def _messenger_near(messenger, t, along, drop=0.0):
    return (messenger[0] + t[0] * along, messenger[1] - drop, messenger[2] + t[2] * along)


def _draw_console(builder, sp, k, local, anchor, base_y=None, mast=False):
    """Steel console (earthed): heel band, tube to just past the clevis, tie, the messenger on its
    insulator string and saddle; on the inner side of tight curves the string is held by a
    fixator from the tube."""
    hinge, tie, clevis, tube_dir = k["hinge"], k["tie"], k["clevis"], k["tube_dir"]
    messenger = k["messenger"]
    if not mast and not sp.get("console_shift"):
        for point in (hinge, tie):
            band_radius = pole_radius_at(base_y, point[1]) + 0.015
            centre = sp["pole"]
            _tube(builder, local((centre[0], point[1] - CLAMP_BAND_HEIGHT / 2, centre[2])),
                  local((centre[0], point[1] + CLAMP_BAND_HEIGHT / 2, centre[2])), band_radius,
                  POLE_SIDES, COLOR_STEEL, anchor)
    _tube(builder, local(hinge), local(_add(clevis, _mul(tube_dir, CANTILEVER_OVERHANG))),
          TUBE_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    tie_end = _add(clevis, _mul(tube_dir, CANTILEVER_OVERHANG * 0.6))
    _tube(builder, local(tie), local(tie_end), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
    # clevis (bugel) round the tube, the insulator string and the saddle under it
    _tube(builder, local(_add(clevis, (0.0, -0.07, 0.0))), local(_add(clevis, (0.0, 0.05, 0.0))),
          0.045, 6, COLOR_STEEL, anchor, caps=True)
    _disc_string(builder, local(_add(clevis, (0.0, -0.07, 0.0))), local(_add(messenger, (0.0, 0.06, 0.0))), anchor)
    _saddle(builder, messenger, k["t"], anchor, local)
    if sp.get("garland_fix"):
        # fixator Sh-1: a rod with an insulator from the tube to the foot of the string
        start = k["on_tube"](messenger[1] + 0.06)
        foot = _add(messenger, (0.0, 0.06, 0.0))
        if math.dist(start, foot) > 0.45:
            mid = _lerp(start, foot, 0.30 / math.dist(start, foot))
            _insulator_chain(builder, local(start), local(_lerp(start, foot, 0.62)), anchor,
                             sheds=4, shed_radius=0.06, core_radius=0.022)
            _tube(builder, local(_lerp(start, foot, 0.62)), local(foot), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)


def _fixator_geometry(sp, k, kind):
    """Points of a direct or reverse fixator (world): where it leaves the console tube, the end of
    its insulator, the main rod's end, the stand's foot, the bend of the steady arm, the clamp on
    the wire, and where the two support strings from the messenger hold the main rod."""
    contact = k["contact"]
    rail_y = sp["track"][1]
    wire_h = contact[1] - rail_y
    from_pole, axis = k["from_pole"], k["axis"]
    out = math.hypot(contact[0] - axis[0], contact[2] - axis[1])
    clamp = (contact[0], contact[1] + 0.02, contact[2])

    def dist_out(p):
        return math.hypot(p[0] - axis[0], p[2] - axis[1])

    g = {"clamp": clamp, "kind": kind}
    if kind == "reverse":
        a0 = k["on_tube"](rail_y + wire_h + REVERSE_ATTACH)
        end = from_pole(out + MAIN_FIXATOR_BEYOND, wire_h + REVERSE_MAIN_AT_WIRE - 0.04)
        direction = _unit(_sub(end, a0))
        ins_end = _add(a0, _mul(direction, FIX_INSULATOR_LENGTH))
        stand_foot = _add(end, (0.0, -STAND_DROP, 0.0))
        bend = from_pole(out + STEADY_ARM_BEND, wire_h + 0.08)
        f = (out + 0.15 - dist_out(a0)) / max(0.1, dist_out(end) - dist_out(a0))
        g.update(a0=a0, ins_end=ins_end, end=end, stand_foot=stand_foot, bend=bend, strings=_lerp(a0, end, f))
        return g
    a0 = k["on_tube"](rail_y + wire_h + DIRECT_ATTACH)
    d0 = dist_out(a0)
    ins_end = from_pole(d0 + FIX_INSULATOR_LENGTH, wire_h + DIRECT_ATTACH - 0.02)
    stand_out = max(out - STEADY_ARM_REACH, d0 + FIX_INSULATOR_LENGTH + 0.25)
    end = from_pole(stand_out, wire_h + DIRECT_MAIN)
    stand_foot = _add(end, (0.0, -STAND_DROP, 0.0))
    bend = from_pole(out - STEADY_ARM_BEND, wire_h + 0.08) if out - stand_out > STEADY_ARM_BEND + 0.15 else None
    g.update(a0=a0, ins_end=ins_end, end=end, stand_foot=stand_foot, bend=bend, strings=_lerp(ins_end, end, 0.75))
    return g


def string_foot(sp):
    """Where the two support strings of a support's fixator end (world), or None: they run from
    the messenger STRING_ALONG either side of the support (drawn with the spans, so they meet
    the messenger as the span draws it)."""
    if sp.get("dropped") or sp.get("anchor") is not None:
        return None
    kind = sp.get("fixator", "direct")
    if kind not in ("direct", "reverse"):
        return None
    if sp["portal"] is not None and sp.get("mast") is None:
        return None
    return _fixator_geometry(sp, _support_points(sp), kind)["strings"]


def _draw_fixator(builder, sp, k, local, anchor, kind, attach_on=None, raise_arm=0.0):
    """Fixator from the console tube to the contact wire. kind "overlap": a second wire held from
    another support's console (attach_on: the host's key points; host_fix: its fixator)."""
    if kind == "overlap":
        _draw_second_arm(builder, sp, k, local, anchor, attach_on)
        return
    g = _fixator_geometry(sp, k, kind)
    _insulator_chain(builder, local(g["a0"]), local(g["ins_end"]), anchor, sheds=FIX_INSULATOR_SHEDS,
                     shed_radius=0.065, core_radius=0.024, spacing=FIX_SHED_SPACING)
    radius = TUBE_RADIUS * (0.85 if kind == "reverse" else 0.75)
    _tube(builder, local(g["ins_end"]), local(g["end"]), radius, 4, COLOR_STEEL, anchor, caps=True)
    _tube(builder, local(_add(g["end"], (0.0, 0.05, 0.0))), local(g["stand_foot"]), 0.02, 4, COLOR_STEEL, anchor, caps=True)
    if g["bend"] is not None:
        _tube(builder, local(g["stand_foot"]), local(g["bend"]), 0.018, 4, COLOR_STEEL, anchor, caps=True)
        _tube(builder, local(g["bend"]), local(g["clamp"]), 0.018, 4, COLOR_STEEL, anchor, caps=True)
    else:
        _tube(builder, local(g["stand_foot"]), local(g["clamp"]), 0.018, 4, COLOR_STEEL, anchor, caps=True)


def _draw_second_arm(builder, sp, k, local, anchor, host):
    """Second wire on another track's console (at a turnout): its steady arm hangs from the host
    fixator's main rod where that rod passes over the wire; otherwise its own arm runs from the
    host's tube level above every part of the host's fixator, then a stand drops to the wire."""
    contact = k["contact"]
    rail_y = sp["track"][1]
    clamp = (contact[0], contact[1] + 0.02, contact[2])
    axis = host["axis"]
    out = math.hypot(contact[0] - axis[0], contact[2] - axis[1])
    to_track = host["to_track"]

    def at(dist_out, y):
        return (axis[0] + to_track[0] * dist_out, y, axis[1] + to_track[2] * dist_out)

    def dist_out(p):
        return math.hypot(p[0] - axis[0], p[2] - axis[1])

    stand_out = out - STEADY_ARM_BEND if out > dist_out(host["hinge"]) + 1.0 else out
    fix = host.get("fixator_geometry")
    if fix is not None:
        r0, r1 = dist_out(fix["ins_end"]), dist_out(fix["end"])
        if r0 + 0.05 <= stand_out <= r1 - 0.05:
            f = (stand_out - r0) / (r1 - r0)
            on_rod = _lerp(fix["ins_end"], fix["end"], f)
            if on_rod[1] - clamp[1] > 0.10:
                foot = at(stand_out, max(clamp[1] + 0.06, on_rod[1] - 0.25))
                _tube(builder, local(_add(on_rod, (0.0, 0.03, 0.0))), local(foot), 0.018, 4, COLOR_STEEL, anchor, caps=True)
                _tube(builder, local(foot), local(clamp), 0.018, 4, COLOR_STEEL, anchor, caps=True)
            else:
                _tube(builder, local(on_rod), local(clamp), 0.018, 4, COLOR_STEEL, anchor, caps=True)
            return
    a0 = host["on_tube"](rail_y + SECOND_ARM_ATTACH)
    d0 = dist_out(a0)
    level = a0[1] - 0.02
    ins_end = at(d0 + FIX_INSULATOR_LENGTH, level)
    _insulator_chain(builder, local(a0), local(ins_end), anchor, sheds=FIX_INSULATOR_SHEDS,
                     shed_radius=0.065, core_radius=0.024, spacing=FIX_SHED_SPACING)
    top = at(max(stand_out, d0 + FIX_INSULATOR_LENGTH + 0.10), level)
    _tube(builder, local(ins_end), local(top), 0.022, 4, COLOR_STEEL, anchor, caps=True)
    foot = (top[0], clamp[1] + 0.08, top[2])
    _tube(builder, local(_add(top, (0.0, 0.03, 0.0))), local(foot), 0.02, 4, COLOR_STEEL, anchor, caps=True)
    _tube(builder, local(foot), local(clamp), 0.018, 4, COLOR_STEEL, anchor, caps=True)


def _draw_traverses(builder, sp, local, anchor):
    """Two traverses (channels) along the track on the track side of a pole carrying two consoles:
    one at the heel height, one at the tie height."""
    base_y = pole_base_y(sp)
    pole, track = sp["pole"], sp["track"]
    tx, tz = sp["t"]
    to_track = _unit2((track[0] - pole[0], track[2] - pole[2]))
    depth, height = TRAVERSE_SIZE
    for h in (CANTILEVER_HINGE, TIE_HEIGHT):
        y = track[1] + h
        radius = pole_radius_at(base_y, y)
        centre = (pole[0] + to_track[0] * (radius + depth / 2), y - height / 2, pole[2] + to_track[1] * (radius + depth / 2))
        frame = signs.Frame(local(centre), (tx, tz))
        signs._box(builder, frame, TRAVERSE_HALF + 0.15, depth / 2, 0.0, height, [(0.0, height, COLOR_STEEL)], anchor)
        builder.polygon([frame.at(-TRAVERSE_HALF - 0.15, -depth / 2, 0.0), frame.at(TRAVERSE_HALF + 0.15, -depth / 2, 0.0),
                         frame.at(TRAVERSE_HALF + 0.15, depth / 2, 0.0), frame.at(-TRAVERSE_HALF - 0.15, depth / 2, 0.0)],
                        COLOR_STEEL, anchor, (0.0, -1.0, 0.0))
        _tube(builder, local((pole[0], y - height / 2 - 0.03, pole[2])), local((pole[0], y + height / 2 + 0.03, pole[2])),
              radius + 0.015, POLE_SIDES, COLOR_STEEL, anchor, caps=True)


def add_portal_hanger(builder, sp, portal, origin):
    """Wires of one track at a portal: a console post (square tube with a strut) under the girder
    carrying an ordinary console and fixator; where no post fits, and in through trusses, the
    messenger hangs on an insulator string under the girder and a rigid rod holds the wire."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    k = _support_points(sp)
    messenger, contact = k["messenger"], k["contact"]
    anchor = (messenger[0] - ox, messenger[2] - oz)
    beam_bottom = portal_beam_y(portal) - portal_beam_size(portal)[1] / 2
    if sp.get("mast") is None and portal.get("strut"):
        # bridge with the deck below: a channel hook-bolted under the top cross strut, a steel rod
        # down from it, the insulator string with the saddle at its foot, a rigid rod to the wire
        a, b = portal["poles"]
        across = _unit2((b[0] - a[0], b[2] - a[2]))
        clamp_top = beam_bottom + TRUSS_CLAMP[2]
        frame = signs.Frame(local((messenger[0], beam_bottom, messenger[2])), across)
        signs._box(builder, frame, TRUSS_CLAMP[0] / 2, TRUSS_CLAMP[1] / 2, 0.0, TRUSS_CLAMP[2],
                   [(0.0, TRUSS_CLAMP[2], COLOR_TRUSS)], anchor)
        builder.polygon([frame.at(-TRUSS_CLAMP[0] / 2, -TRUSS_CLAMP[1] / 2, 0.0), frame.at(TRUSS_CLAMP[0] / 2, -TRUSS_CLAMP[1] / 2, 0.0),
                         frame.at(TRUSS_CLAMP[0] / 2, TRUSS_CLAMP[1] / 2, 0.0), frame.at(-TRUSS_CLAMP[0] / 2, TRUSS_CLAMP[1] / 2, 0.0)],
                        COLOR_TRUSS, anchor, (0.0, -1.0, 0.0))
        foot = (messenger[0], messenger[1] + 0.06, messenger[2])
        string_top = (messenger[0], min(beam_bottom, foot[1] + GARLAND_LENGTH + 0.05), messenger[2])
        if beam_bottom - string_top[1] > 0.03:
            _tube(builder, local((messenger[0], clamp_top - 0.02, messenger[2])), local(string_top), ROD_RADIUS, 4,
                  COLOR_STEEL, anchor, caps=True)
        _disc_string(builder, local(string_top), local(foot), anchor)
        _saddle(builder, messenger, k["t"], anchor, local)
        _tube(builder, local(messenger), local((contact[0], contact[1] + 0.02, contact[2])),
              TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)
        return
    if sp.get("mast") is None:
        top = (messenger[0], beam_bottom, messenger[2])
        foot = (messenger[0], messenger[1] + 0.06, messenger[2])
        string_end = (messenger[0], max(foot[1], beam_bottom - GARLAND_LENGTH - 0.05), messenger[2])
        _disc_string(builder, local(top), local(string_end), anchor)
        if string_end[1] > foot[1] + 0.03:
            _tube(builder, local(string_end), local(foot), ROD_RADIUS, 4, COLOR_STEEL, anchor, caps=True)
        _saddle(builder, messenger, k["t"], anchor, local)
        _tube(builder, local(messenger), local((contact[0], contact[1] + 0.02, contact[2])),
              TUBE_RADIUS * 0.8, 4, COLOR_STEEL, anchor, caps=True)
        return
    mast = sp["mast"]
    rail_y = sp["track"][1]
    bottom_y = rail_y + CANTILEVER_HINGE - MAST_BELOW_HINGE
    m_anchor = (mast[0] - ox, mast[1] - oz)
    a, b = portal["poles"]
    across = _unit2((b[0] - a[0], b[2] - a[2]))
    along = (across[1], -across[0])
    frame = signs.Frame(local((mast[0], bottom_y, mast[1])), along)
    signs._box(builder, frame, MAST_SIZE / 2, MAST_SIZE / 2, 0.0, beam_bottom - bottom_y,
               [(0.0, beam_bottom - bottom_y, COLOR_STEEL)], m_anchor)
    builder.polygon([frame.at(-MAST_SIZE / 2, -MAST_SIZE / 2, 0.0), frame.at(MAST_SIZE / 2, -MAST_SIZE / 2, 0.0),
                     frame.at(MAST_SIZE / 2, MAST_SIZE / 2, 0.0), frame.at(-MAST_SIZE / 2, MAST_SIZE / 2, 0.0)],
                    COLOR_STEEL, m_anchor, (0.0, -1.0, 0.0))
    # frame of angles round the girder's bottom chords where the post is bolted on
    width = portal_beam_size(portal)[0]

    def girder_clamp(x, z):
        """Angles laid across under the girder's bottom chords (from chord to chord), bolted on."""
        frame = signs.Frame(local((x, beam_bottom - 0.08, z)), along)
        signs._box(builder, frame, width / 2 + 0.04, 0.07, 0.0, 0.08, [(0.0, 0.08, COLOR_STEEL)], m_anchor)
        builder.polygon([frame.at(-width / 2 - 0.04, -0.07, 0.0), frame.at(width / 2 + 0.04, -0.07, 0.0),
                         frame.at(width / 2 + 0.04, 0.07, 0.0), frame.at(-width / 2 - 0.04, 0.07, 0.0)],
                        COLOR_STEEL, m_anchor, (0.0, -1.0, 0.0))

    girder_clamp(mast[0], mast[1])
    # strut: from the girder, MAST_STRUT_OUT away from the track, down to the post's foot
    away = _unit2((mast[0] - sp["track"][0], mast[1] - sp["track"][2]))
    girder_clamp(mast[0] + away[0] * MAST_STRUT_OUT, mast[1] + away[1] * MAST_STRUT_OUT)
    strut_top = (mast[0] + away[0] * MAST_STRUT_OUT, beam_bottom - 0.08, mast[1] + away[1] * MAST_STRUT_OUT)
    strut_foot = (mast[0] + away[0] * MAST_SIZE / 2, bottom_y + 0.25, mast[1] + away[1] * MAST_SIZE / 2)
    _tube(builder, local(strut_top), local(strut_foot), 0.030, 4, COLOR_STEEL, m_anchor, caps=True)
    _draw_console(builder, sp, k, local, m_anchor, mast=True)
    _draw_fixator(builder, sp, k, local, m_anchor, sp.get("fixator", "reverse"))


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
        if sp.get("traverse"):
            _draw_traverses(builder, sp, local, anchor)
    if sp.get("anchor") is not None:
        add_anchor(builder, sp, origin)
        return
    kind = sp.get("fixator", "direct")
    if kind != "overlap":
        _draw_console(builder, sp, k, local, anchor, base_y)
        _draw_fixator(builder, sp, k, local, anchor, kind)
        return
    # second wire on another support's console (turnout): its messenger on its own string from
    # the host's tube, its arm from the host's tube higher up
    host_index = sp.get("overlap_of")
    if host_index is None:
        host_index = sp.get("pole_owner")
    host = None
    if supports is not None and host_index is not None:
        h = supports[host_index]
        if h.get("anchor") is None and h["portal"] is None and h.get("fixator") != "overlap":
            host = _support_points(h)
            if h.get("fixator", "direct") in ("direct", "reverse"):
                host["fixator_geometry"] = _fixator_geometry(h, host, h.get("fixator", "direct"))
    if host is None:
        _draw_console(builder, sp, k, local, anchor, base_y)
        _draw_fixator(builder, sp, k, local, anchor, "direct")
        return
    messenger = k["messenger"]
    d = math.hypot(messenger[0] - host["axis"][0], messenger[2] - host["axis"][1])
    dh = math.hypot(host["clevis"][0] - host["axis"][0], host["clevis"][2] - host["axis"][1])
    if abs(d - dh) > 0.12:
        top = host["on_tube"](min(host["clevis"][1], host["hinge"][1] + (host["clevis"][1] - host["hinge"][1]) * d / max(dh, 0.1)))
        top = (messenger[0], top[1], messenger[2])
        _tube(builder, local(_add(top, (0.0, -0.06, 0.0))), local(_add(top, (0.0, 0.05, 0.0))), 0.04, 6,
              COLOR_STEEL, anchor, caps=True)
        if top[1] - messenger[1] > 0.3:
            _disc_string(builder, local(_add(top, (0.0, -0.06, 0.0))), local(_add(messenger, (0.0, 0.06, 0.0))), anchor)
            _saddle(builder, messenger, k["t"], anchor, local)
    _draw_fixator(builder, sp, k, local, anchor, "overlap", attach_on=host)


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
    _disc_string(builder, out(MESSENGER_INSULATOR[0], MESSENGER_ANCHOR_HEIGHT),
                 out(MESSENGER_INSULATOR[1], MESSENGER_ANCHOR_HEIGHT), anchor, discs=ANCHOR_DISCS)
    rope(out(MESSENGER_END_OUT, MESSENGER_ANCHOR_HEIGHT), out(MESSENGER_INSULATOR[1], MESSENGER_ANCHOR_HEIGHT),
         ROD_RADIUS)
    rope(out(MESSENGER_INSULATOR[0], MESSENGER_ANCHOR_HEIGHT),
         out(surface(MESSENGER_ANCHOR_HEIGHT), MESSENGER_ANCHOR_HEIGHT), ROD_RADIUS)
    # contact wire: clamp, rope, insulator, rope to the outer moving pulley
    rope(out(CONTACT_END_OUT, h), out(CONTACT_INSULATOR[1], h), ROD_RADIUS)
    _disc_string(builder, out(CONTACT_INSULATOR[0], h), out(CONTACT_INSULATOR[1], h), anchor,
                 discs=ANCHOR_DISCS)
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


def _feeder_beam(fp):
    """(root, tip, height): the bracket beam of a DPR support, world."""
    x, z = fp["pos"]
    ox_, oz_ = fp["out"]
    y = fp["bracket_y"]
    root_out = fp.get("root_out", 0.0)
    return (x + ox_ * root_out, y, z + oz_ * root_out), (x + ox_ * (FEEDER_ARM[1] + 0.15), y, z + oz_ * (FEEDER_ARM[1] + 0.15)), y


def feeder_points(fp):
    """Where the two DPR wires are held at a support (world)."""
    x, z = fp["pos"]
    y = fp["bracket_y"] - GARLAND_LENGTH - 0.05
    points = [(x + fp["out"][0] * out, y, z + fp["out"][1] * out) for out in FEEDER_ARM]
    if fp.get("end"):
        tw = fp["end"]
        y = fp["bracket_y"] - 0.12
        points = [(p[0] + tw[0] * (ANCHOR_DISCS * DISC_PITCH + 0.25), y, p[2] + tw[1] * (ANCHOR_DISCS * DISC_PITCH + 0.25))
                  for p in points]
    return points


def add_feeder_pole(builder, fp, origin):
    """DPR bracket KFD on the field side: a horizontal beam near the pole top with a strut under it
    (on a portal: on a stand on the girder end; in a through truss: on the truss side), the two
    wires on insulator strings (at the end of a line: on strain strings towards the line)."""
    ox, oz = origin

    def local(p):
        return (p[0] - ox, p[1], p[2] - oz)

    x, z = fp["pos"]
    anchor = (x - ox, z - oz)
    root, tip, y = _feeder_beam(fp)
    away = (fp["out"][0], 0.0, fp["out"][1])
    _tube(builder, local(root), local(tip), 0.035, 4, COLOR_STEEL, anchor, caps=True)
    kind = fp.get("kind", "pole")
    if kind == "pole":
        radius = pole_radius_at(fp["base_y"], y)
        for h in (y, y - FEEDER_STRUT_DROP):
            _tube(builder, local((x, h - 0.04, z)), local((x, h + 0.04, z)), pole_radius_at(fp["base_y"], h) + 0.015,
                  POLE_SIDES, COLOR_STEEL, anchor, caps=True)
        strut_foot = (x + away[0] * radius, y - FEEDER_STRUT_DROP, z + away[2] * radius)
    else:
        if kind == "portal":
            stand_foot = (x, fp["stand_from"], z)
            _tube(builder, local(stand_foot), local((x, y + 0.08, z)), 0.045, 4, COLOR_STEEL, anchor, caps=True)
            # the stand is bolted to a channel laid across the girder top (from chord to chord)
            span = fp.get("girder_width", 0.45) / 2 + 0.03
            frame = signs.Frame(local((x, fp["stand_from"], z)), tuple(fp.get("t", (-away[2], away[0]))))
            signs._box(builder, frame, span, 0.07, 0.0, 0.06, [(0.0, 0.06, COLOR_STEEL)], anchor)
            builder.polygon([frame.at(-span, -0.07, 0.0), frame.at(span, -0.07, 0.0),
                             frame.at(span, 0.07, 0.0), frame.at(-span, 0.07, 0.0)], COLOR_STEEL, anchor, (0.0, -1.0, 0.0))
        if kind == "truss":
            # bolted to the outer face of the top chord: a plate over the chord's side, the brace
            # from the chord's foot
            low = fp.get("chord_low", y - 0.4)
            frame = signs.Frame(local((x, low, z)), tuple(fp.get("t", (-away[2], away[0]))))
            signs._box(builder, frame, 0.18, 0.015, 0.0, y + 0.04 - low, [(0.0, y + 0.04 - low, COLOR_STEEL)], anchor)
            strut_foot = (x + away[0] * 0.02, low + 0.03, z + away[2] * 0.02)
        else:
            strut_foot = (x + away[0] * 0.05, y - FEEDER_STRUT_DROP * 0.8, z + away[2] * 0.05)
    strut_top = (x + away[0] * (FEEDER_ARM[0] - FEEDER_STRUT_IN), y - 0.03, z + away[2] * (FEEDER_ARM[0] - FEEDER_STRUT_IN))
    _tube(builder, local(strut_foot), local(strut_top), 0.025, 4, COLOR_STEEL, anchor, caps=True)
    for out, held in zip(FEEDER_ARM, feeder_points(fp)):
        top = (x + away[0] * out, y - 0.04, z + away[2] * out)
        if fp.get("end"):
            hook = (top[0], held[1], top[2])
            _tube(builder, local(top), local(hook), 0.012, 4, COLOR_STEEL, anchor, caps=True)
            _disc_string(builder, local(hook), local(held), anchor, discs=ANCHOR_DISCS)
        else:
            _disc_string(builder, local(top), local((held[0], held[1] + WIRE_RADIUS * 0.5, held[2])), anchor)


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

    def messenger_at(f):
        """On the messenger as drawn: straight pieces between points of the parabola."""
        f = min(1.0, max(0.0, f))
        piece = min(pieces - 1, int(f * pieces))
        f0, f1 = piece / pieces, (piece + 1) / pieces
        q0, q1 = _lerp(m0, m1, f0), _lerp(m0, m1, f1)
        q0 = (q0[0], q0[1] - 4.0 * sag * f0 * (1.0 - f0), q0[2])
        q1 = (q1[0], q1[1] - 4.0 * sag * f1 * (1.0 - f1), q1[2])
        return _lerp(q0, q1, (f - f0) * pieces)

    # droppers
    count = int(length // DROPPER_SPACING)
    for i in range(1, count + 1):
        f = i / (count + 1)
        c = _lerp(c0, c1, f)
        m = messenger_at(f)
        if m[1] - c[1] < 0.25:
            continue
        _tube(builder, local(c), local(m), DROPPER_RADIUS, 3, COLOR_MESSENGER, anchor)
    for end, sp in ((0.0, a), (1.0, b)):
        towards = 1.0 if end == 0.0 else -1.0

        def f_at(d):
            return end + towards * d / max(length, 1e-6)

        foot = string_foot(sp)
        if foot is not None and length > 2 * STRING_ALONG + 1.0:
            # support string of the fixator: from its main rod up to the messenger in this span
            _tube(builder, local(foot), local(messenger_at(f_at(STRING_ALONG))), DROPPER_RADIUS, 3, COLOR_STEEL, anchor)
        if sp.get("spring") and length > 2 * SPRING_ALONG + 2.0:
            # spring stay: from the messenger SPRING_ALONG into the span down to the spring's
            # dropper and on to the support, where it runs under the saddle
            here = messenger_at(end)
            low = here[1] - SPRING_BELOW
            far = messenger_at(f_at(SPRING_ALONG))
            mid = messenger_at(f_at(SPRING_DROPPER))
            mid = (mid[0], low, mid[2])
            centre = (here[0], low, here[2])
            _tube(builder, local(far), local(mid), DROPPER_RADIUS * 1.3, 3, COLOR_STEEL, anchor)
            _tube(builder, local(mid), local(centre), DROPPER_RADIUS * 1.3, 3, COLOR_STEEL, anchor)
            c = _lerp(c0, c1, f_at(SPRING_DROPPER))
            if low - c[1] > 0.2:
                _tube(builder, local(c), local(mid), DROPPER_RADIUS, 3, COLOR_MESSENGER, anchor)
        if sp.get("middle_anchor"):
            # middle anchor: rope from the messenger at the support down to the contact wire
            d = min(MIDDLE_ANCHOR_ROPE, 0.45 * length)
            top = messenger_at(end)
            clamp = _lerp(c0, c1, f_at(d))
            _tube(builder, local(top), local((clamp[0], clamp[1] + 0.03, clamp[2])), DROPPER_RADIUS * 1.4,
                  4, COLOR_STEEL, anchor, caps=True)
            _tube(builder, local((clamp[0], clamp[1] - 0.02, clamp[2])), local((clamp[0], clamp[1] + 0.06, clamp[2])),
                  0.03, 6, COLOR_STEEL, anchor, caps=True)
    if span.get("insulators") in ("from", "to") and length > 6.0:
        # insulating overlap: strain insulators in both wires of the anchored branch, near the
        # transition support
        near, far = OVERLAP_INSULATOR_AT
        if span["insulators"] == "from":
            f0, f1 = near / length, far / length
        else:
            f0, f1 = 1.0 - near / length, 1.0 - far / length
        _disc_string(builder, local(messenger_at(f0)), local(messenger_at(f1)), anchor, discs=ANCHOR_DISCS)
        _disc_string(builder, local(_lerp(c0, c1, f0)), local(_lerp(c0, c1, f1)), anchor, discs=ANCHOR_DISCS)


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
