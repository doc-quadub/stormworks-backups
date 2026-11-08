"""Soviet / Russian track kilometre posts and picket posts (GOST 8442-65, drawing 1).

Two parts:
  * plan_signs()  - where the posts go: distance along the track network from the zero point
                    at North Harbour, every 100 m; kilometre posts on whole kilometres, picket
                    posts in between. Two networks: Sawyer (mega_island_X_Y tiles, zero at
                    North Harbour) and Meier (arid_island_X_Y tiles, zero at the port switch
                    2430,-26500; the shipyard dock line there carries no posts).
  * add_sign()    - the geometry (baked into the tile track meshes, so no extra draw calls)
                    and a box collider for each post.

Coordinates: x east, z north, y up. right_of((x, z)) is the right-hand side when facing (x, z).
Numbers are drawn from rectangles on a 3 x 5 grid (vertex colours only, no textures), laid
into the face itself so digits and background never overlap (no z-fighting).
"""

import heapq
import math
import os
import re

from swmesh import parse_transform, read_phys
import rails

# ---------------------------------------------------------------- dimensions (metres)

POST_SIDE = 0.16                 # square posts; GOST drawing value not available, assumed
POST_BURIED = 0.40               # posts go this deep below the ground point (slopes, bumps)

KM_POST_HEIGHT = 2.10            # ground to the bottom of the plate
KM_POST_BLACK = 0.25             # black base band
KM_PLATE_HEIGHT = 0.27
KM_PLATE_THICKNESS = 0.03
KM_PLATE_WIDTH = {1: 0.30, 2: 0.30, 3: 0.36, 4: 0.45}
KM_DIGIT = dict(height=0.17, columns=(0.020, 0.035, 0.020),
                rows=(0.020, 0.055, 0.020, 0.055, 0.020),
                gap={1: 0.0, 2: 0.030, 3: 0.019, 4: 0.024})

PICKET_BLACK = 0.21              # 0 .. 0.21 black
PICKET_WHITE_BELOW = 0.05        # then white
PICKET_NUMBER_BAND = 0.15        # white band with the black number
PICKET_WHITE_ABOVE = 0.05
PICKET_TOP = 0.50                # total height; the rest above the bands is a pyramid
PICKET_DIGIT = dict(height=0.11, columns=(0.015, 0.020, 0.015),
                    rows=(0.015, 0.0325, 0.015, 0.0325, 0.015),
                    gap={1: 0.0, 2: 0.010})

WHISTLE_POST_HEIGHT = 1.75       # ground to the bottom of the plate (drawing 26)
WHISTLE_POST_BLACK = 0.25        # black base band
WHISTLE_BAND = 0.10              # black band under the plate
WHISTLE_PLATE_WIDTH = 0.35       # R175 top
WHISTLE_PLATE_HEIGHT = 0.47
WHISTLE_PLATE_BOTTOM = 0.22      # bottom width
WHISTLE_ROUND_FROM = 0.295       # the round top starts this high (centre of R175)
WHISTLE_GREEN = 0.17             # green zone at the bottom
WHISTLE_BORDER = 0.012           # black edge
WHISTLE_LETTER = dict(centre=0.32, half_width=0.065, half_height=0.12, stroke=0.025, gap=0.05)
WHISTLE_ROW = 0.006              # the face is laid out in rows at most this tall (no overlapping faces)
COLOR_GREEN = (36, 118, 62, 255)
COLOR_PLATE_BACK = (112, 112, 110, 255)

COLOR_BLACK = (28, 28, 28, 255)
COLOR_POST = (205, 205, 200, 255)      # "white, rather slightly grey"
COLOR_PLATE = (236, 236, 232, 255)
COLOR_DIGIT = (18, 18, 18, 255)

# 3 x 5 digit bitmaps, top row first; '1' = black cell
DIGITS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "010", "010", "010", "010"),   # a plain stroke (with a flag it read as a 7)
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "011", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "001", "001", "001"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
}

# ---------------------------------------------------------------- placement rules

ZERO_TILE = "mega_island_9_8"            # North Harbour
ZERO_NODES = ("train_18", "train_20")    # switches at the east throat of the station

# Meier (arid island): zero at the switch by the port station, 2430,-26500. Behind it (north) the
# station dead ends (map left) and the shipyard dock line (map right): the double track running
# straight north-north-west to the dock ("Etrain Terminal") and the curve joining it from the
# station switch at 2323,-26413. The dock line and that curve never get posts, pickets or
# overhead line (author's rule) - see DOCK_LINES / in_dock_line().
MEIER_ZERO = (2430.0, -26500.0)
MEIER_ZERO_RADIUS = 9.0                  # graph nodes this close to the zero point start the count
DOCK_LINES = (
    # (polyline in world x, z, half width): a point belongs to the dock line if it lies within the
    # half width of the polyline and not before its first point
    (((2430.5, -26500.0), (2165.2, -25510.0)), 7.0),                       # dock double track
    (((2323.4, -26413.0), (2338, -26407), (2346, -26402), (2353, -26396), (2359, -26390),
      (2365, -26383), (2369, -26375), (2373, -26368), (2376, -26359), (2378, -26350),
      (2378, -26333), (2377, -26321), (2372.45, -26291.7), (2366, -26270)), 5.0),   # curve to it
)


def in_dock_line(x, z):
    for line, half_width in DOCK_LINES:
        for i, (a, b) in enumerate(zip(line, line[1:])):
            dx, dz = b[0] - a[0], b[1] - a[1]
            length2 = dx * dx + dz * dz
            t = ((x - a[0]) * dx + (z - a[1]) * dz) / length2
            if i == 0 and t * math.sqrt(length2) < 3.0:
                continue                  # before the start (the line it branches off)
            t = min(1.0, max(0.0, t))
            if math.hypot(a[0] + dx * t - x, a[1] + dz * t - z) <= half_width:
                return True
    return False

PICKET_SPACING = 100.0
MIN_LINE_REACH = 1000.0      # a track counts as a line (not a siding) if it goes on this far
RIGHTMOST_SEARCH = 12.0      # another line closer than this on the right -> not the outer track
MIN_CLEARANCE = 2.6          # post (its nearest edge) to any track axis
LATERAL_OFFSETS = (3.1, 3.6, 4.2, 5.0, 5.7)   # GOST: >= 3.1 m from the axis (5.7 m in cuttings)
GROUND_BELOW_RAIL = 4.0      # accepted ground: from this far below the rail top ...
GROUND_ABOVE_RAIL = 3.0      # ... up to this far above it (cuttings: posts stand on the slope)
HEADROOM = 2.7               # nothing solid allowed above the ground point up to this height
TILE_HALF = 500.0
JOIN_TOLERANCE = 1.0         # border nodes of neighbouring tiles closer than this are joined
JOIN_EDGE_MARGIN = 3.0       # ... if both lie within this of their tile edge


def mainland_origin(tile):
    m = re.fullmatch(r"mega_island_(\d+)_(\d+)", tile)
    return None if m is None else (int(m.group(1)) * 1000 - 8000, int(m.group(2)) * 1000 - 12000)


# tile families with a regular 1 km grid: (tile prefix, world x, z of tile 0_0 centre)
FAMILIES = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}


def tile_origin(tile, family=None):
    """World x, z of the tile centre for the grid families (optionally only of `family`)."""
    m = re.fullmatch(r"(mega_island|arid_island)_(\d+)_(\d+)", tile)
    if m is None or (family is not None and m.group(1) != family):
        return None
    ox, oz = FAMILIES[m.group(1)]
    return (int(m.group(2)) * 1000 + ox, int(m.group(3)) * 1000 + oz)


def right_of(t):
    return (t[1], -t[0])


# ---------------------------------------------------------------- geometry

def number_layout(text, spec, area_width):
    """Column boundaries (u, left to right as seen by the viewer) and which (column, row) cells
    are black, for a number centred in [-area_width/2, area_width/2]."""
    count = len(text)
    gap = spec["gap"][count]
    digit_width = sum(spec["columns"])
    total = count * digit_width + (count - 1) * gap
    u = -total / 2
    widths, owner = [], []          # owner: (digit index, column) or None for gaps/margins
    margin = (area_width - total) / 2
    widths.append(margin)
    owner.append(None)
    for d in range(count):
        if d:
            widths.append(gap)
            owner.append(None)
        for c, w in enumerate(spec["columns"]):
            widths.append(w)
            owner.append((d, c))
    widths.append(margin)
    owner.append(None)
    return widths, owner


def face_rects(text, spec, area_width, area_height, number_bottom):
    """Rectangles (u0, u1, v0, v1, colour) tiling a face [-w/2, w/2] x [0, h] with the number.
    number_bottom = v of the bottom of the digits."""
    widths, owner = number_layout(text, spec, area_width)
    rows = list(spec["rows"])                       # top row first
    heights = [number_bottom] + rows[::-1] + [area_height - number_bottom - sum(rows)]
    row_index = [None] + list(range(len(rows) - 1, -1, -1)) + [None]   # bitmap row per band
    rects = []
    v = 0.0
    for h, r in zip(heights, row_index):
        if h > 1e-6:
            # cells of this band left to right, neighbours of the same colour merged
            runs = []
            u = -area_width / 2
            for w, own in zip(widths, owner):
                black = (r is not None and own is not None
                         and DIGITS[text[own[0]]][r][own[1]] == "1")
                color = COLOR_DIGIT if black else None
                if w > 1e-9:
                    if runs and runs[-1][2] is color:
                        runs[-1][1] = u + w
                    else:
                        runs.append([u, u + w, color])
                u += w
            rects += [(u0, u1, v, v + h, color) for u0, u1, color in runs]
        v += h
    return rects


class Frame:
    """Local frame of a post: origin on the ground, t along the track (increasing distance),
    r to the right of it (away from the track), y up."""

    def __init__(self, origin, t):
        self.o = origin
        self.t = (t[0], 0.0, t[1])
        r = right_of(t)
        self.r = (r[0], 0.0, r[1])

    def at(self, along, side, up):
        return (self.o[0] + self.t[0] * along + self.r[0] * side,
                self.o[1] + up,
                self.o[2] + self.t[2] * along + self.r[2] * side)


def _box(builder, frame, half_t, half_r, y0, y1, colors_by_band, anchor, top=True):
    """Upright box around the frame origin. colors_by_band: [(y_from, y_to, colour)]."""
    sides = [((1, 0), (half_t, half_r)), ((-1, 0), (half_t, half_r)),
             ((0, 1), (half_t, half_r)), ((0, -1), (half_t, half_r))]
    for (st, sr), (ht, hr) in sides:
        outward = (frame.t[0] * st + frame.r[0] * sr, 0.0, frame.t[2] * st + frame.r[2] * sr)
        if st:
            a, b = (st * ht, -hr), (st * ht, hr)
        else:
            a, b = (-ht, sr * hr), (ht, sr * hr)
        for ya, yb, color in colors_by_band:
            builder.polygon([frame.at(a[0], a[1], ya), frame.at(b[0], b[1], ya),
                             frame.at(b[0], b[1], yb), frame.at(a[0], a[1], yb)],
                            color, anchor, outward)
    if top:
        builder.polygon([frame.at(-half_t, -half_r, y1), frame.at(half_t, -half_r, y1),
                         frame.at(half_t, half_r, y1), frame.at(-half_t, half_r, y1)],
                        colors_by_band[-1][2], anchor, (0.0, 1.0, 0.0))


def _numbered_face(builder, frame, facing, along, half_width, v_base, rects, anchor):
    """Face with outward normal facing * t at distance `along`; rects in (u, v) where u runs to
    the viewer's right and v up from v_base."""
    # viewer looks along -facing*t; their right-hand side is right_of(-facing*t) = -facing*r
    outward = (frame.t[0] * facing, 0.0, frame.t[2] * facing)
    for u0, u1, v0, v1, color in rects:
        s0, s1 = -facing * u0, -facing * u1
        builder.polygon([frame.at(along, s0, v_base + v0), frame.at(along, s1, v_base + v0),
                         frame.at(along, s1, v_base + v1), frame.at(along, s0, v_base + v1)],
                        color or COLOR_PLATE, anchor, outward)


def add_box_collision(builder, frame, half_t, half_r, y0, y1, anchor):
    corners = {}
    for st in (-1, 1):
        for sr in (-1, 1):
            for sy, y in ((-1, y0), (1, y1)):
                corners[(st, sr, sy)] = frame.at(st * half_t, sr * half_r, y)
    faces = [((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
             ((-1, 0, 0), [(-1, -1, -1), (-1, 1, -1), (-1, 1, 1), (-1, -1, 1)]),
             ((0, 1, 0), [(-1, 1, -1), (1, 1, -1), (1, 1, 1), (-1, 1, 1)]),
             ((0, -1, 0), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
             ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
             ((0, 0, -1), [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1)])]
    for (st, sr, sy), keys in faces:
        quad = [corners[k] for k in keys]
        outward = (frame.t[0] * st + frame.r[0] * sr, float(sy),
                   frame.t[2] * st + frame.r[2] * sr)
        if rails.dot(rails.polygon_normal(quad), outward) < 0:
            quad = [quad[0], quad[3], quad[2], quad[1]]
        # game winding: (b-a)x(c-a) points into the solid
        builder.collision.append(((quad[0], quad[2], quad[1]), anchor))
        builder.collision.append(((quad[0], quad[3], quad[2]), anchor))


def add_km_post(builder, frame, number_behind, number_ahead):
    anchor = (frame.o[0], frame.o[2])
    half = POST_SIDE / 2
    _box(builder, frame, half, half, -POST_BURIED, KM_POST_HEIGHT,
         [(-POST_BURIED, KM_POST_BLACK, COLOR_BLACK), (KM_POST_BLACK, KM_POST_HEIGHT, COLOR_POST)],
         anchor)
    digits = max(len(str(number_behind)), len(str(number_ahead)))
    width = KM_PLATE_WIDTH[min(4, digits)]
    half_t = KM_PLATE_THICKNESS / 2
    y0, y1 = KM_POST_HEIGHT, KM_POST_HEIGHT + KM_PLATE_HEIGHT
    bottom = (KM_PLATE_HEIGHT - KM_DIGIT["height"]) / 2
    # the face looking back along the track shows the kilometre that ends here
    for facing, number in ((-1, number_behind), (1, number_ahead)):
        rects = face_rects(str(number), KM_DIGIT, width, KM_PLATE_HEIGHT, bottom)
        _numbered_face(builder, frame, facing, facing * half_t, width / 2, y0, rects, anchor)
    # plate edges
    w = width / 2
    for sr in (-1, 1):
        builder.polygon([frame.at(-half_t, sr * w, y0), frame.at(half_t, sr * w, y0),
                         frame.at(half_t, sr * w, y1), frame.at(-half_t, sr * w, y1)],
                        COLOR_PLATE, anchor, (frame.r[0] * sr, 0.0, frame.r[2] * sr))
    for y, sy in ((y0, -1), (y1, 1)):
        builder.polygon([frame.at(-half_t, -w, y), frame.at(half_t, -w, y),
                         frame.at(half_t, w, y), frame.at(-half_t, w, y)],
                        COLOR_PLATE, anchor, (0.0, float(sy), 0.0))
    add_box_collision(builder, frame, half, half, -POST_BURIED, KM_POST_HEIGHT, anchor)
    add_box_collision(builder, frame, half_t, w, y0, y1, anchor)


def add_picket_post(builder, frame, number_behind, number_ahead):
    anchor = (frame.o[0], frame.o[2])
    half = POST_SIDE / 2
    white_from = PICKET_BLACK
    white_to = PICKET_BLACK + PICKET_WHITE_BELOW + PICKET_NUMBER_BAND + PICKET_WHITE_ABOVE
    # black base on all four sides, plain white band on the two sides facing the track / away
    for sr in (-1, 1):
        outward = (frame.r[0] * sr, 0.0, frame.r[2] * sr)
        for ya, yb, color in ((-POST_BURIED, white_from, COLOR_BLACK), (white_from, white_to, COLOR_POST)):
            builder.polygon([frame.at(-half, sr * half, ya), frame.at(half, sr * half, ya),
                             frame.at(half, sr * half, yb), frame.at(-half, sr * half, yb)],
                            color, anchor, outward)
    bottom = PICKET_WHITE_BELOW + (PICKET_NUMBER_BAND - PICKET_DIGIT["height"]) / 2
    for facing, number in ((-1, number_behind), (1, number_ahead)):
        outward = (frame.t[0] * facing, 0.0, frame.t[2] * facing)
        builder.polygon([frame.at(facing * half, -half, -POST_BURIED), frame.at(facing * half, half, -POST_BURIED),
                         frame.at(facing * half, half, white_from), frame.at(facing * half, -half, white_from)],
                        COLOR_BLACK, anchor, outward)
        rects = [(u0, u1, v0, v1, c if c else COLOR_POST) for u0, u1, v0, v1, c in
                 face_rects(str(number), PICKET_DIGIT, POST_SIDE, white_to - white_from, bottom)]
        _numbered_face(builder, frame, facing, facing * half, half, white_from, rects, anchor)
    # pyramid top
    apex = frame.at(0.0, 0.0, PICKET_TOP)
    corners = [frame.at(-half, -half, white_to), frame.at(half, -half, white_to),
               frame.at(half, half, white_to), frame.at(-half, half, white_to)]
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        mid = ((a[0] + b[0]) / 2 - frame.o[0], 0.0, (a[2] + b[2]) / 2 - frame.o[2])
        builder.polygon([a, b, apex], COLOR_POST, anchor, (mid[0], 0.05, mid[2]))
    add_box_collision(builder, frame, half, half, -POST_BURIED, PICKET_TOP, anchor)


def _whistle_half_width(v):
    """Half width of the "C" plate outline at height v above its bottom (0 outside)."""
    if v < 0.0 or v > WHISTLE_PLATE_HEIGHT:
        return 0.0
    if v <= WHISTLE_ROUND_FROM:
        return WHISTLE_PLATE_BOTTOM / 2 + (WHISTLE_PLATE_WIDTH - WHISTLE_PLATE_BOTTOM) / 2 * v / WHISTLE_ROUND_FROM
    r = WHISTLE_PLATE_WIDTH / 2
    dv = v - WHISTLE_ROUND_FROM
    return math.sqrt(max(0.0, r * r - dv * dv))


def _whistle_layers():
    """Paint layers of the plate face, bottom to top: (left(v), right(v), colour, v range).
    Edges are functions of the height v, so each row of the face is cut along the true shapes
    (trapezoids), not in steps."""
    L = WHISTLE_LETTER
    border = WHISTLE_BORDER

    def ellipse(a, b):
        return lambda v: a * math.sqrt(max(0.0, 1.0 - ((v - L["centre"]) / b) ** 2))

    def inner(v):
        top = _whistle_half_width(v + border) if v + border <= WHISTLE_PLATE_HEIGHT else 0.0
        low = _whistle_half_width(v - border) if v >= border else 0.0
        return max(0.0, min(top, low) - border)

    outer_c = ellipse(L["half_width"], L["half_height"])
    inner_c = ellipse(L["half_width"] - L["stroke"], L["half_height"] - L["stroke"])
    c_lo, c_hi = L["centre"] - L["half_height"], L["centre"] + L["half_height"]
    i_lo, i_hi = L["centre"] - L["half_height"] + L["stroke"], L["centre"] + L["half_height"] - L["stroke"]
    g_lo, g_hi = L["centre"] - L["gap"], L["centre"] + L["gap"]
    neg = lambda f: (lambda v: -f(v))
    zero = lambda v: 0.0
    return [
        (neg(_whistle_half_width), _whistle_half_width, COLOR_BLACK, (0.0, WHISTLE_PLATE_HEIGHT)),
        (neg(inner), inner, COLOR_GREEN, (0.0, WHISTLE_GREEN)),
        (neg(inner), inner, COLOR_PLATE, (WHISTLE_GREEN, WHISTLE_PLATE_HEIGHT)),
        # the letter: an elliptic ring open on the right (the gap between g_lo and g_hi)
        (neg(outer_c), neg(inner_c), COLOR_DIGIT, (i_lo, i_hi)),
        (inner_c, outer_c, COLOR_DIGIT, (i_lo, g_lo)),
        (inner_c, outer_c, COLOR_DIGIT, (g_hi, i_hi)),
        (neg(outer_c), outer_c, COLOR_DIGIT, (c_lo, i_lo)),
        (neg(outer_c), outer_c, COLOR_DIGIT, (i_hi, c_hi)),
    ]


def _whistle_row_breaks():
    """Row boundaries of the face: every WHISTLE_ROW plus every height where a layer starts or ends."""
    marks = {0.0, WHISTLE_PLATE_HEIGHT, WHISTLE_ROUND_FROM}
    for _, _, _, (lo, hi) in _whistle_layers():
        marks.update((lo, hi))
    k = 1
    while k * WHISTLE_ROW < WHISTLE_PLATE_HEIGHT:
        marks.add(k * WHISTLE_ROW)
        k += 1
    marks = sorted(marks)
    out = [marks[0]]
    for m in marks[1:]:
        if m - out[-1] > 1e-4:
            out.append(m)
    return out


def _whistle_row_pieces(v0, v1):
    """Trapezoids [((u0 at v0, u1 at v0), (u0 at v1, u1 at v1), colour)] of one row: at the row's
    middle the top layer decides each piece's colour; every edge is the layer edge function,
    evaluated at both row ends, so neighbouring pieces and rows share their edges exactly."""
    vm = (v0 + v1) / 2
    layers = [l for l in _whistle_layers() if l[3][0] - 1e-9 <= vm <= l[3][1] + 1e-9]
    edges = []
    for left, right, _, _ in layers:
        edges += [left, right]
    cuts = sorted(edges, key=lambda f: f(vm))
    pieces = []
    for fa, fb in zip(cuts, cuts[1:]):
        a, b = fa(vm), fb(vm)
        m = (a + b) / 2
        color = None
        if b - a < 1e-6:
            # vanishing here but maybe not at the row's ends (the plate top): part of the
            # piece on its left, so no slit opens between them
            color = pieces[-1][2] if pieces and pieces[-1][1] is fa else None
        else:
            for left, right, c, _ in layers:          # the last layer wins
                if left(vm) <= m <= right(vm):
                    color = c
        if color is None:
            continue
        if pieces and pieces[-1][2] is color and pieces[-1][1] is fa:
            pieces[-1][1] = fb
        else:
            pieces.append([fa, fb, color])
    return [((fa(v0), fb(v0)), (fa(v1), fb(v1)), c) for fa, fb, c in pieces]


def add_whistle_sign(builder, frame):
    """Sign "C" (sound the whistle), GOST 8442-65 drawing 26. frame.t: the direction of the train
    it is meant for; the plate stands across the track and faces the train."""
    anchor = (frame.o[0], frame.o[2])
    half = POST_SIDE / 2
    y0 = WHISTLE_POST_HEIGHT
    _box(builder, frame, half, half, -POST_BURIED, y0 + 0.05,
         [(-POST_BURIED, WHISTLE_POST_BLACK, COLOR_BLACK), (WHISTLE_POST_BLACK, y0 - WHISTLE_BAND, COLOR_POST),
          (y0 - WHISTLE_BAND, y0 + 0.05, COLOR_BLACK)], anchor)
    half_t = KM_PLATE_THICKNESS / 2
    face = (-frame.t[0], 0.0, -frame.t[2])
    back = (frame.t[0], 0.0, frame.t[2])
    breaks = _whistle_row_breaks()
    for v0, v1 in zip(breaks, breaks[1:]):
        for (a0, b0), (a1, b1), color in _whistle_row_pieces(v0, v1):
            quad = [(a0, v0), (b0, v0), (b1, v1), (a1, v1)]
            quad = [p for k, p in enumerate(quad) if math.dist(p, quad[k - 1]) > 1e-6]   # the top row: a triangle
            if len(quad) >= 3:
                builder.polygon([frame.at(-half_t, u, y0 + v) for u, v in quad], color, anchor, face)
    # back and edge of the plate
    # (the straight sides as single edges: collinear points would make zero-area fan triangles)
    outline = [(0.0, _whistle_half_width(0.0))]
    steps = 12
    for k in range(steps + 1):
        v = WHISTLE_ROUND_FROM + (WHISTLE_PLATE_HEIGHT - WHISTLE_ROUND_FROM) * k / steps
        outline.append((v, _whistle_half_width(v)))
    ring = []
    for u, v in [(hw, v) for v, hw in outline] + [(-hw, v) for v, hw in outline[::-1]]:
        if not ring or math.hypot(u - ring[-1][0], v - ring[-1][1]) > 1e-6:
            ring.append((u, v))
    builder.polygon([frame.at(half_t, u, y0 + v) for u, v in ring], COLOR_PLATE_BACK, anchor, back)
    for (ua, va), (ub, vb) in zip(ring, ring[1:] + ring[:1]):
        nx, nv = vb - va, -(ub - ua)
        outward = (frame.r[0] * nx, nv, frame.r[2] * nx)
        builder.polygon([frame.at(-half_t, ua, y0 + va), frame.at(half_t, ua, y0 + va),
                         frame.at(half_t, ub, y0 + vb), frame.at(-half_t, ub, y0 + vb)],
                        COLOR_BLACK, anchor, outward)
    add_box_collision(builder, frame, half, half, -POST_BURIED, y0, anchor)
    add_box_collision(builder, frame, half_t, WHISTLE_PLATE_WIDTH / 2, y0, y0 + WHISTLE_PLATE_HEIGHT, anchor)


def add_sign(builder, sign):
    """sign: dict from plan_signs (tile-local position)."""
    frame = Frame(tuple(sign["pos"]), tuple(sign["dir"]))
    if sign["kind"] == "whistle":
        add_whistle_sign(builder, frame)
    elif sign["kind"] == "km":
        add_km_post(builder, frame, sign["behind"], sign["ahead"])
    else:
        add_picket_post(builder, frame, sign["behind"], sign["ahead"])


# ---------------------------------------------------------------- whistle signs "C" before bridges

WHISTLE_DISTANCE = 500.0        # sign this far before the bridge (along the track) ...
WHISTLE_FURTHER = 100.0         # ... or up to this much further where there is no place ...
WHISTLE_NEARER = 250.0          # ... or nearer, down to WHISTLE_DISTANCE - this
WHISTLE_MIN_APPROACH = 150.0    # a shorter approach (track end, junction): no sign
BRIDGE_SAMPLE = 3.0             # bridge test every this many metres along the lines
BRIDGE_MIN_LENGTH = 20.0        # shorter decks (culverts) are not bridges
BRIDGE_MERGE = 9.0              # deck samples closer than this belong to one bridge
BRIDGE_SIDE = 7.5               # the deck must be open at this distance on at least one side
BRIDGE_AIR = 2.0                # ... and have this much air under it
BRIDGE_ROW_GAP = 150.0          # bridges closer than this one after another get one sign


def is_bridge(ground, x, z, rail_y, t):
    """A track point on a bridge deck: a surface at the deck level with air under it (or water:
    nothing at all), open beside the track on at least one side (not a pier, platform or wide
    embankment)."""
    hs = sorted(ground.heights(x, z), reverse=True)
    deck = [h for h in hs if rail_y - 1.5 <= h <= rail_y + 0.1]
    if not deck:
        return False
    top = max(deck)
    below = [h for h in hs if h < top - 0.05]
    if below and top - below[0] <= 2.5:
        bottom = below[0]                   # the underside of the deck
        below = below[1:]
    else:
        bottom = top
    if below and bottom - below[0] < BRIDGE_AIR:
        return False                        # a slab on the ground
    r = right_of(t)
    for side in (1.0, -1.0):
        side_h = ground.heights(x + r[0] * side * BRIDGE_SIDE, z + r[1] * side * BRIDGE_SIDE)
        if any(top - 2.0 <= h <= top + 2.0 for h in side_h):
            continue                        # deck, platform or ground level beside the track
        if not below and side_h:
            continue                        # a single sheet with ground beside: an embankment
        return True
    return False


def _plan_whistle_signs(net, ground, signs, placed_posts, point_on, walk, spot, log):
    """Signal sign "C" (sound the whistle, GOST 8442-65 drawing 26) before every bridge, for each
    direction, WHISTLE_DISTANCE before its end on the right of the rightmost track."""
    nodes_world, dist, lines, line_index = net.nodes_world, net.dist, net.lines, net.line_index
    edge_tile = net.edge_tile
    before = {}
    for a, b in lines:
        before.setdefault(b, []).append(a)

    def tangent(a, b):
        pa, pb = nodes_world[a], nodes_world[b]
        tx, tz = pb[0] - pa[0], pb[2] - pa[2]
        tl = math.hypot(tx, tz)
        return (tx / tl, tz / tl) if tl > 1e-6 else None

    def walk_back(a, b, f, extra):
        """(edge, fraction) `extra` metres back along the line from (a, b, f)."""
        while True:
            length = max(1e-6, dist[b] - dist[a])
            g = f - extra / length
            if g >= 0.0:
                return (a, b), g
            extra -= f * length
            prev = before.get(a)
            if not prev:
                return None
            ta = tangent(a, b)
            pa = nodes_world[a]
            nb = max(prev, key=lambda c: (ta[0] * (pa[0] - nodes_world[c][0]) + ta[1] * (pa[2] - nodes_world[c][2]))
                     / max(1e-6, math.hypot(pa[0] - nodes_world[c][0], pa[2] - nodes_world[c][2])))
            a, b, f = nb, a, 1.0

    # 1. deck samples along every line edge, and where each line edge goes on or off a bridge
    samples = []                     # (x, z, edge, f, on_bridge)
    per_edge = {}
    for a, b in lines:
        t = tangent(a, b)
        if t is None:
            continue
        length = dist[b] - dist[a]
        n = max(1, int(math.ceil(length / BRIDGE_SAMPLE)))
        flags = []
        for k in range(n + 1):
            f = k / n
            p = point_on(a, b, f)
            flags.append((f, is_bridge(ground, p[0], p[2], p[1], t), p))
        per_edge[(a, b)] = flags
    # 2. bridges: deck samples grouped by proximity; short ones dropped
    cell = {}
    deck_points = []
    for edge, flags in per_edge.items():
        for f, on, p in flags:
            if on:
                deck_points.append((p[0], p[2]))
    parent = list(range(len(deck_points)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i, (x, z) in enumerate(deck_points):
        cx, cz = int(x // BRIDGE_MERGE), int(z // BRIDGE_MERGE)
        for gx in (cx - 1, cx, cx + 1):
            for gz in (cz - 1, cz, cz + 1):
                for j in cell.get((gx, gz), ()):
                    if math.hypot(deck_points[j][0] - x, deck_points[j][1] - z) < BRIDGE_MERGE:
                        parent[find(i)] = find(j)
        cell.setdefault((cx, cz), []).append(i)
    groups = {}
    for i in range(len(deck_points)):
        groups.setdefault(find(i), []).append(i)
    bridge_of = {}
    bridges = []
    for members in groups.values():
        xs = [deck_points[i] for i in members]
        span = math.hypot(max(p[0] for p in xs) - min(p[0] for p in xs), max(p[1] for p in xs) - min(p[1] for p in xs))
        if span < BRIDGE_MIN_LENGTH:
            continue
        for i in members:
            bridge_of[deck_points[i]] = len(bridges)
        bridges.append(xs)
    BRIDGES.append((net.family, [(sum(p[0] for p in xs) / len(xs), sum(p[1] for p in xs) / len(xs), len(xs))
                                 for xs in bridges]))
    # 3. approach ends: a line edge going on a bridge (increasing distance) or coming off it
    approaches = []              # (bridge, edge, f, travel)  travel +1: with the distance, -1: against it
    for (a, b), flags in per_edge.items():
        for (f0, on0, p0), (f1, on1, p1) in zip(flags, flags[1:]):
            if on0 == on1:
                continue
            key = (p1[0], p1[2]) if on1 else (p0[0], p0[2])
            bridge = bridge_of.get(key)
            if bridge is None:
                continue
            if on1:
                approaches.append((bridge, (a, b), f0, 1.0))      # onto the bridge with the distance
            else:
                approaches.append((bridge, (a, b), f1, -1.0))     # onto it against the distance
    # an edge start on a bridge whose previous edge is off it (transition exactly at a node) is
    # caught by the previous edge's last sample, which is the same point
    def on_bridge_deck(edge, g):
        """Is this point of the line (its own track, not a neighbour's) on a found bridge?"""
        flags = per_edge.get(edge)
        if not flags:
            return False
        _, on, p = flags[int(round(g * (len(flags) - 1)))]
        return on and (p[0], p[2]) in bridge_of

    placed = []
    count = {"bridges": len(bridges), "whistle": 0, "not_outer": 0, "no_place": 0, "short": 0, "follows_bridge": 0}
    for bridge, (a, b), f, travel in sorted(approaches, key=lambda e: (e[0], e[3])):
        # a train that has just come off another deck (bridges in a row, a viaduct over piers)
        # had its sign before the first one
        follows = False
        for extra in range(5, int(BRIDGE_ROW_GAP) + 1, 5):
            at = walk_back(a, b, f, extra) if travel > 0 else walk(a, b, f, extra)
            if at is None:
                break
            if on_bridge_deck(at[0], at[1]):
                follows = True
                break
        if follows:
            count["follows_bridge"] += 1
            continue
        site = None
        for extra in ([WHISTLE_DISTANCE + d for d in range(0, int(WHISTLE_FURTHER) + 1, 5)]
                      + [WHISTLE_DISTANCE - d for d in range(5, int(WHISTLE_NEARER) + 1, 5)]):
            at = walk_back(a, b, f, extra) if travel > 0 else walk(a, b, f, extra)
            if at is None:
                continue
            (ea, eb), g = at
            q = point_on(ea, eb, g)
            t = tangent(ea, eb)
            if t is None:
                continue
            d = (t[0] * travel, t[1] * travel)              # the train's direction
            r = right_of(d)
            outer = True
            for sa, sb, info in line_index.near(q[0], q[2], RIGHTMOST_SEARCH):
                if info in ((ea, eb), (eb, ea)):
                    continue
                if _ray_hits_segment((q[0], q[2]), r, RIGHTMOST_SEARCH, sa, sb) is not None:
                    outer = False
                    break
            if not outer:
                site = "not_outer"
                break
            found = spot(q, d, 1.0, 0.0, WHISTLE_PLATE_WIDTH / 2)
            if found is None:
                site = site or "no_place"
                continue
            if any(math.hypot(found[0] - x, found[2] - z) < 30.0 and dd[0] * d[0] + dd[1] * d[1] > 0.5
                   for x, z, dd in placed):
                site = "duplicate"
                break
            site = (found, (ea, eb), d)
            break
        if site is None:
            count["short"] += 1
            continue
        if isinstance(site, str):
            if site in count:
                count[site] += 1
            continue
        (x, y, z), edge, d = site
        placed.append((x, z, d))
        placed_posts.append((x, z))
        tile = edge_tile[edge]
        ox, oz = tile_origin(tile)
        signs.setdefault(tile, []).append({"kind": "whistle", "pos": [x - ox, y, z - oz], "dir": [d[0], d[1]]})
        count["whistle"] += 1
    log("whistle signs: %s" % count)


# ---------------------------------------------------------------- ground (tile physics)

class Ground:
    """Vertical ray casts against the vanilla collision meshes of mainland tiles."""

    CELL = 8.0

    def __init__(self, rom_dir, family="mega_island"):
        self.rom_dir = rom_dir
        self.family = family
        self.tiles = {}

    def _load(self, tile):
        if tile in self.tiles:
            return self.tiles[tile]
        grid = {}
        path = os.path.join(self.rom_dir, "data", "tiles", tile + ".xml")
        if os.path.exists(path):
            text = open(path, encoding="utf-8", errors="replace").read()
            for m in re.finditer(r"<physics_mesh ([^>]*)>\s*<transform([^/]*)/>", text):
                attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
                if "parent_id" in attrs or "file_name" not in attrs:
                    continue          # doors and other moving parts
                phys_path = os.path.join(self.rom_dir, *attrs["file_name"].split("/"))
                if not os.path.exists(phys_path):
                    continue
                matrix = parse_transform(m.group(2))
                for part in read_phys(phys_path):
                    for i in range(0, len(part) - 2, 3):
                        tri = [_transform(p, matrix) for p in part[i:i + 3]]
                        x0 = min(p[0] for p in tri)
                        x1 = max(p[0] for p in tri)
                        z0 = min(p[2] for p in tri)
                        z1 = max(p[2] for p in tri)
                        for ix in range(int(math.floor(x0 / self.CELL)), int(math.floor(x1 / self.CELL)) + 1):
                            for iz in range(int(math.floor(z0 / self.CELL)), int(math.floor(z1 / self.CELL)) + 1):
                                grid.setdefault((ix, iz), []).append(tri)
        self.tiles[tile] = grid
        return grid

    def heights(self, world_x, world_z):
        """All surface heights at a world point (any tile of the family)."""
        fx, fz = FAMILIES[self.family]
        tx = int(math.floor((world_x - fx + 500) / 1000))
        tz = int(math.floor((world_z - fz + 500) / 1000))
        tile = "%s_%d_%d" % (self.family, tx, tz)
        origin = (tx * 1000 + fx, tz * 1000 + fz)
        x, z = world_x - origin[0], world_z - origin[1]
        grid = self._load(tile)
        hits = []
        for a, b, c in grid.get((int(math.floor(x / self.CELL)), int(math.floor(z / self.CELL))), ()):
            d = (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])
            if abs(d) < 1e-12:
                continue
            l1 = ((x - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (z - a[2])) / d
            l2 = ((b[0] - a[0]) * (z - a[2]) - (x - a[0]) * (b[2] - a[2])) / d
            if l1 < -1e-6 or l2 < -1e-6 or l1 + l2 > 1 + 1e-6:
                continue
            hits.append(a[1] + l1 * (b[1] - a[1]) + l2 * (c[1] - a[1]))
        return hits


def _transform(p, m):
    return (p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0] + m[3][0],
            p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1] + m[3][1],
            p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] + m[3][2])


# ---------------------------------------------------------------- planning

class SegmentIndex:
    CELL = 20.0

    def __init__(self):
        self.grid = {}

    def add(self, a, b, info):
        steps = int(math.hypot(b[0] - a[0], b[1] - a[1]) // (self.CELL / 2)) + 1
        cells = set()
        for i in range(steps + 1):
            f = i / steps
            cells.add((int(math.floor((a[0] + (b[0] - a[0]) * f) / self.CELL)),
                       int(math.floor((a[1] + (b[1] - a[1]) * f) / self.CELL))))
        for c in cells:
            self.grid.setdefault(c, []).append((a, b, info))

    def near(self, x, z, radius):
        r = int(math.ceil(radius / self.CELL))
        cx, cz = int(math.floor(x / self.CELL)), int(math.floor(z / self.CELL))
        seen = set()
        for ix in range(cx - r, cx + r + 1):
            for iz in range(cz - r, cz + r + 1):
                for item in self.grid.get((ix, iz), ()):
                    if id(item) not in seen:
                        seen.add(id(item))
                        yield item


def _point_segment_distance(p, a, b):
    dx, dz = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dz * dz
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / length2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dz)


def _ray_hits_segment(p, d, max_s, a, b):
    """Distance along ray p + s*d (0 < s < max_s) to segment a-b, or None."""
    ex, ez = b[0] - a[0], b[1] - a[1]
    den = d[0] * ez - d[1] * ex
    if abs(den) < 1e-9:
        return None
    wx, wz = a[0] - p[0], a[1] - p[1]
    s = (wx * ez - wz * ex) / den
    u = (wx * d[1] - wz * d[0]) / den
    if 0.05 < s < max_s and -1e-6 <= u <= 1 + 1e-6:
        return s
    return None


FAILED = []      # diagnostics of the last plan_signs run: posts that found no place
BRIDGES = []     # diagnostics: bridges found by the last run [(family, [(x, z), ...]), ...]


class Network:
    """The mainland track graph in world coordinates, as drawn, with distances from the zero
    point. Shared by the track posts and the overhead line."""


def build_network(tiles, prepare_graph, family="mega_island"):
    """prepare_graph(tile) -> (nodes, node_y, inserted, border_tangents, stray), the same graph
    the tiles step draws (smoothed, border nodes joined). family: "mega_island" (Sawyer, zero at
    North Harbour) or "arid_island" (Meier, zero at MEIER_ZERO, dock line left out)."""
    mainland = [t for t in tiles if tile_origin(t, family) is not None]
    nodes_world = {}       # (tile, id) -> (x, rail_top_y, z) world
    links = {}             # (tile, id) -> [(tile, id), ...]
    for tile in mainland:
        ox, oz = tile_origin(tile)
        nodes, node_y, _, _, stray = prepare_graph(tile)
        if family == "arid_island":
            stray = set(stray) | {k for k, (p, _) in nodes.items() if in_dock_line(p[0] + ox, p[2] + oz)}
        for k, (p, ls) in nodes.items():
            if k in stray:
                continue
            nodes_world[(tile, k)] = (p[0] + ox, node_y[k] + rails.RAIL_TOP_OFFSET, p[2] + oz)
            links[(tile, k)] = [(tile, o) for o in ls if o in nodes and o != k and o not in stray]

    # join tiles: a node on a tile edge and the matching node of the neighbouring tile become one
    # node (the first of the pair stands for both). Edge nodes are usually track ends, but some
    # are switches sitting exactly on the border.
    def on_edge(key):
        x, _, z = nodes_world[key]
        ox, oz = tile_origin(key[0])
        return max(abs(x - ox), abs(z - oz)) >= TILE_HALF - 0.6

    def near_edge(key):
        # border nodes of a track crossing the edge at a shallow angle may lie a little inside
        # one tile and outside the other (seen at -4500,-29400 on Meier)
        x, _, z = nodes_world[key]
        ox, oz = tile_origin(key[0])
        margin = JOIN_EDGE_MARGIN if family == "arid_island" else 0.6    # Sawyer: as released
        return max(abs(x - ox), abs(z - oz)) >= TILE_HALF - margin

    by_cell = {}
    for key, (x, y, z) in nodes_world.items():
        if near_edge(key):
            by_cell.setdefault((round(x), round(z)), []).append(key)
    alias = {}
    for key, (x, y, z) in nodes_world.items():
        if key in alias or not near_edge(key):
            continue
        for cx in (round(x) - 1, round(x), round(x) + 1):
            for cz in (round(z) - 1, round(z), round(z) + 1):
                for other in by_cell.get((cx, cz), ()):
                    if other[0] == key[0] or other in alias or other == key:
                        continue
                    q = nodes_world[other]
                    if math.hypot(q[0] - x, q[2] - z) < JOIN_TOLERANCE and abs(q[1] - y) < 0.6:
                        alias[other] = key
    for k in list(alias):
        target = alias[k]
        while target in alias and alias[target] != target:
            target = alias[target]
        alias[k] = target
    joins = len(alias)
    merged = {}
    edge_tile = {}
    for key, others in links.items():
        a = alias.get(key, key)
        merged.setdefault(a, [])
        for o in others:
            b = alias.get(o, o)
            if b != a and b not in merged[a]:
                merged[a].append(b)
            edge_tile[(a, b)] = key[0]
            edge_tile[(b, a)] = key[0]
    for a in list(merged):
        for b in merged[a]:
            if a not in merged.setdefault(b, []):
                merged[b].append(a)
    links = merged
    nodes_world = {k: v for k, v in nodes_world.items() if k in links}

    def length(a, b):
        return math.dist(nodes_world[a], nodes_world[b])

    # distance from the zero point (Dijkstra)
    dist = {}
    if family == "arid_island":
        heap = [(0.0, k) for k, p in nodes_world.items()
                if math.hypot(p[0] - MEIER_ZERO[0], p[2] - MEIER_ZERO[1]) < MEIER_ZERO_RADIUS]
    else:
        heap = [(0.0, (ZERO_TILE, k)) for k in ZERO_NODES if (ZERO_TILE, k) in nodes_world]
    assert heap, "zero point nodes not found"
    while heap:
        d, key = heapq.heappop(heap)
        if key in dist:
            continue
        dist[key] = d
        for o in links[key]:
            if o not in dist:
                heapq.heappush(heap, (d + length(key, o), o))

    # edges along which the distance grows like the path length: oriented low -> high
    downstream = {}
    edges = []
    for a in dist:
        for b in links[a]:
            if b in dist and dist[b] > dist[a]:
                if dist[b] - dist[a] >= length(a, b) - 0.5:
                    downstream.setdefault(a, []).append(b)
                    edges.append((a, b))
    # reach: how far the track goes on beyond a node, along the growing distance.
    # open: the track beyond the node does not just stop at a buffer - it leads to a tile exit
    # (the modular sea lines) or to the point where two distance waves meet (a loop).
    reach = {}
    is_open = {}
    for key in sorted(dist, key=lambda k: -dist[k]):
        after = downstream.get(key, ())
        reach[key] = max([dist[key]] + [reach[o] for o in after])
        if after:
            is_open[key] = any(is_open[o] for o in after)
        else:
            is_open[key] = len(links[key]) >= 2 or on_edge(key)
    # A line (not a siding): an open track, or one going on for at least MIN_LINE_REACH past the
    # edge, or the plain continuation (no junction) of a line edge.
    upstream = {}
    for a, b in edges:
        upstream.setdefault(b, []).append(a)
    line_set = set()
    for a, b in sorted(edges, key=lambda e: dist[e[0]]):
        if is_open[b] or reach[b] - dist[a] >= MIN_LINE_REACH:
            line_set.add((a, b))
        elif len(links[a]) == 2 and any((u, a) in line_set for u in upstream.get(a, ())):
            line_set.add((a, b))
    lines = [e for e in edges if e in line_set]

    all_tracks = SegmentIndex()
    line_index = SegmentIndex()
    seen = set()
    for a in nodes_world:
        for b in links[a]:
            if frozenset((a, b)) in seen:
                continue
            seen.add(frozenset((a, b)))
            pa, pb = nodes_world[a], nodes_world[b]
            all_tracks.add((pa[0], pa[2]), (pb[0], pb[2]), (a, b))
            if (a, b) in line_set or (b, a) in line_set:
                line_index.add((pa[0], pa[2]), (pb[0], pb[2]), (a, b))

    net = Network()
    net.nodes_world, net.links, net.edge_tile = nodes_world, links, edge_tile
    net.dist, net.line_set, net.lines = dist, line_set, lines
    net.all_tracks, net.line_index = all_tracks, line_index
    net.on_edge, net.joins, net.mainland = on_edge, joins, mainland
    net.family = family
    return net


KM_SHIFT_MAX = 100.0         # a kilometre post may move on this far past an obstacle ...
PICKET_SHIFT_MAX = 95.0      # ... a picket post less than the picket spacing, else it is left out
SHIFT_STEP = 5.0
IGNORE_OPPOSITE_RIGHT = {"mega_island": False, "arid_island": True}   # Sawyer: as released
MIN_POST_GAP = 6.0           # posts closer than this to one another: the later one moves on
LEFT_SEARCH = 25.0           # tracks of the bundle on the left: the post goes beyond the last one
GROUND_ABOVE_RAIL_POST = 1.5 # ground higher than this above the rail: the post would be in the slope
EYE_HEIGHT = 3.0             # driver's eye above the rail top (visibility check)
SIGHT_STEP = 0.5


def plan_signs(rom_dir, tiles, prepare_graph, log=print):
    """Return {tile: [sign dict]} for the Sawyer and Meier networks."""
    signs = {}
    for family, name in (("mega_island", "Sawyer"), ("arid_island", "Meier")):
        part = plan_network_signs(rom_dir, tiles, prepare_graph, family, lambda text: log(name + ": " + text))
        for tile, items in part.items():
            signs.setdefault(tile, []).extend(items)
    return signs


def plan_network_signs(rom_dir, tiles, prepare_graph, family, log=print):
    """Return {tile: [sign dict]} for the tiles of one network.

    Every 100 m along each line (distance from North Harbour) one post per bundle of tracks: on the
    right of the rightmost track (right-hand running); where there is no place there - no ground,
    the post would be in a slope, something blocks the driver's view of it, a track too close - on
    the left of the leftmost track of the bundle; where neither side has a place, a kilometre post
    moves on (up to KM_SHIFT_MAX) to the first place past the obstacle, a picket post likewise but
    less than the picket spacing, else it is left out."""
    net = build_network(tiles, prepare_graph, family)
    nodes_world, links, edge_tile = net.nodes_world, net.links, net.edge_tile
    dist, line_set, lines = net.dist, net.line_set, net.lines
    all_tracks, line_index = net.all_tracks, net.line_index
    mainland, joins = net.mainland, net.joins
    after = {}
    for a, b in lines:
        after.setdefault(a, []).append(b)

    ground = Ground(rom_dir, family)
    signs = {}
    placed_posts = []           # (x, z) of the posts so far: two posts never share a place
    stats = {"candidates": 0, "not_outer": 0, "no_place": 0, "km": 0, "picket": 0, "left": 0, "moved": 0}

    def point_on(a, b, f):
        pa, pb = nodes_world[a], nodes_world[b]
        return (pa[0] + (pb[0] - pa[0]) * f, pa[1] + (pb[1] - pa[1]) * f, pa[2] + (pb[2] - pa[2]) * f)

    def walk(a, b, f, extra):
        """(edge, fraction) `extra` metres further on along the line from (a, b, f)."""
        while True:
            length = max(1e-6, dist[b] - dist[a])
            g = f + extra / length
            if g <= 1.0:
                return (a, b), g
            extra -= (1.0 - f) * length
            nxt = after.get(b)
            if not nxt:
                return None
            ta = (nodes_world[b][0] - nodes_world[a][0], nodes_world[b][2] - nodes_world[a][2])
            # the straightest continuation
            nb = max(nxt, key=lambda c: (ta[0] * (nodes_world[c][0] - nodes_world[b][0]) + ta[1] * (nodes_world[c][2] - nodes_world[b][2]))
                     / max(1e-6, math.hypot(nodes_world[c][0] - nodes_world[b][0], nodes_world[c][2] - nodes_world[b][2])))
            a, b, f = b, nb, 0.0

    def visible(eye, target):
        """No ground or structure between the driver's eye and the post top."""
        dx, dy, dz = target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]
        length = math.hypot(dx, dz)
        steps = max(1, int(length / SIGHT_STEP))
        for i in range(1, steps):
            f = i / steps
            x, y, z = eye[0] + dx * f, eye[1] + dy * f, eye[2] + dz * f
            hs = ground.heights(x, z)
            if hs and min(hs) > y:          # the line of sight runs inside a hill or wall
                return False
        return True

    def spot(p, t, side, start, half_width):
        """A place for a post beside the track point p on `side` (+1 right, -1 left), `start`
        metres out: (x, y, z) or (None, reason)."""
        r = right_of(t)
        for offset in LATERAL_OFFSETS:
            lateral = start + offset + half_width
            x, z = p[0] + r[0] * side * lateral, p[2] + r[1] * side * lateral
            clear = all(_point_segment_distance((x, z), sa, sb) >= MIN_CLEARANCE + half_width
                        for sa, sb, _ in all_tracks.near(x, z, MIN_CLEARANCE + 2))
            if not clear:
                continue
            hs = ground.heights(x, z)
            candidates = [h for h in hs if p[1] - GROUND_BELOW_RAIL <= h <= p[1] + GROUND_ABOVE_RAIL]
            if not candidates:
                continue
            y = max(candidates)
            if y > p[1] + GROUND_ABOVE_RAIL_POST:
                continue                        # in the slope: hardly visible, half buried
            blocked = False
            for du, dv in ((0, 0), (half_width, 0), (-half_width, 0), (0, 0.2), (0, -0.2)):
                sx = x + r[0] * du + t[0] * dv
                sz = z + r[1] * du + t[1] * dv
                near = sorted(h for h in ground.heights(sx, sz) if y - 1.0 < h < y + HEADROOM)
                if near and near[-1] - near[0] > 0.3:
                    blocked = True
                    break
            if blocked:
                continue
            if any(math.hypot(x - px, z - pz) < MIN_POST_GAP for px, pz in placed_posts):
                continue                        # another post (a line met from the other side)
            eye = (p[0] - t[0] * 40.0, p[1] + EYE_HEIGHT, p[2] - t[1] * 40.0)    # seen from 40 m before it
            if not visible(eye, (x, y + KM_POST_HEIGHT, z)):
                continue
            return (x, y, z)
        return None

    def bundle_width(p, t, side):
        """How far out the last track of the bundle lies on `side` (0 if none)."""
        r = right_of(t)
        d = (r[0] * side, r[1] * side)
        widest, reach = 0.0, LEFT_SEARCH
        for sa, sb, _ in all_tracks.near(p[0], p[2], reach):
            s_hit = _ray_hits_segment((p[0], p[2]), d, reach, sa, sb)
            if s_hit is not None and s_hit > 0.5:
                widest = max(widest, s_hit)
        return widest

    def _runs_opposite(edge, t):
        u, v = edge
        if u not in dist or v not in dist:
            return False
        pu, pv = nodes_world[u], nodes_world[v]
        gx, gz = pv[0] - pu[0], pv[2] - pu[2]
        if dist[v] < dist[u]:
            gx, gz = -gx, -gz
        gl = math.hypot(gx, gz)
        return gl > 1e-6 and (gx * t[0] + gz * t[1]) / gl < -0.5

    for a, b in lines:
        da, db = dist[a], dist[b]
        first = int(math.floor(da / PICKET_SPACING)) + 1
        last = int(math.floor(db / PICKET_SPACING))
        if db - da < 1e-6:
            continue
        for m in range(max(first, 1), last + 1):
            mark = m * PICKET_SPACING
            stats["candidates"] += 1
            f = (mark - da) / (db - da)
            p = point_on(a, b, f)
            pa, pb = nodes_world[a], nodes_world[b]
            tx, tz = pb[0] - pa[0], pb[2] - pa[2]
            tl = math.hypot(tx, tz)
            if tl < 1e-6:
                continue
            t = (tx / tl, tz / tl)
            r = right_of(t)
            # only the outermost line on the right-hand side carries the posts
            outer = True
            for sa, sb, info in line_index.near(p[0], p[2], RIGHTMOST_SEARCH):
                if info == (a, b):
                    continue
                if _ray_hits_segment((p[0], p[2]), r, RIGHTMOST_SEARCH, sa, sb) is not None:
                    if IGNORE_OPPOSITE_RIGHT[family] and _runs_opposite(info, t):
                        # the track on the right counts the other way (two distance waves met on
                        # a double line): it is not this line's outer track, else neither carries posts
                        continue
                    outer = False
                    break
            if not outer:
                stats["not_outer"] += 1
                continue
            km = mark % 1000.0 < 1e-6
            kilometre = int(round(mark // 1000))
            if km:
                behind, ahead = kilometre, kilometre + 1
                half_width = KM_PLATE_WIDTH[min(4, len(str(ahead)))] / 2
            else:
                picket = int(round((mark - kilometre * 1000) / PICKET_SPACING))
                behind, ahead = picket, picket + 1
                half_width = POST_SIDE / 2
            placed = None
            shift = 0.0
            limit = KM_SHIFT_MAX if km else PICKET_SHIFT_MAX
            while shift <= limit + 1e-6:
                at = walk(a, b, f, shift)
                if at is None:
                    break
                (ea, eb), g = at
                q = point_on(ea, eb, g)
                qa, qb = nodes_world[ea], nodes_world[eb]
                ql = math.hypot(qb[0] - qa[0], qb[2] - qa[2])
                tq = ((qb[0] - qa[0]) / ql, (qb[2] - qa[2]) / ql) if ql > 1e-6 else t
                found = spot(q, tq, 1.0, 0.0, half_width)
                side = 1.0
                if found is None:
                    found = spot(q, tq, -1.0, bundle_width(q, tq, -1.0), half_width)
                    side = -1.0
                if found is not None:
                    placed = (found, (ea, eb), tq, side)
                    break
                shift += SHIFT_STEP
            if placed is None:
                stats["no_place"] += 1
                FAILED.append({"at": [p[0], p[1], p[2]], "mark": mark})
                continue
            (x, y, z), edge, tq, side = placed
            placed_posts.append((x, z))
            stats["left"] += side < 0
            stats["moved"] += shift > 0
            tile = edge_tile[edge]
            ox, oz = tile_origin(tile)
            signs.setdefault(tile, []).append({
                "kind": "km" if km else "picket",
                "pos": [x - ox, y, z - oz],
                "dir": [tq[0], tq[1]],
                "behind": behind, "ahead": ahead, "distance": mark})
            stats["km" if km else "picket"] += 1
    log("tiles %d, nodes %d, tile joins %d, reached %d, line edges %d; %s"
        % (len(mainland), len(nodes_world), joins, len(dist), len(lines), stats))
    _plan_whistle_signs(net, ground, signs, placed_posts, point_on, walk, spot, log)
    return signs
