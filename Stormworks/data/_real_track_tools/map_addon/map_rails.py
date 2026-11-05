"""Extract one map line per railway route from every tile with train tracks.

python3 map_rails.py <rom> <scan.json> <out.json>

Per tile (tile local x/z, decimetres in the output):
  1. graph from <train_tracks>, split into strokes: at every junction the two straightest edges
     continue each other (a train's through route), the rest start new strokes;
  2. greedy cover: strokes reaching the tile border first, longest first; every drawn line covers
     all track within COVER metres (the second track of a double line, station tracks);
  3. an uncovered piece is drawn when it reaches the border or leaves the station area
     (farther than YARD from drawn lines); sidings, loops and short spurs inside it are dropped;
  4. Douglas-Peucker simplification.
"""
import json, math, os, re, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from swmesh import read_tracks

STEP = 2.0          # sampling along tracks, m
COVER = 12.0        # track within this distance of a drawn line is "the same line" on the map
YARD = 75.0         # pieces that never get farther than this from drawn lines are station tracks
BORDER = 480.0      # a dead end this far from the tile centre continues in the next tile
STROKE_TURN = 35.0  # max deflection, deg, for two edges at a junction to form one stroke
MIN_PIECE = 25.0    # shorter uncovered pieces are dropped unless they touch the border
SIMPLIFY = 1.0      # Douglas-Peucker tolerance, m
JOIN = 15.0         # line ends closer than this are joined
MERGE_WALK = 200.0  # how far a joining branch is followed into the covered zone, m


def graph(path):
    nodes = read_tracks(path)
    pos = {k: (v[0][0], v[0][2]) for k, v in nodes.items()}
    adj = defaultdict(set)
    for k, (_, links) in nodes.items():
        for o in links:
            if o in pos and o != k:
                adj[k].add(o)
                adj[o].add(k)
    return pos, adj


def _unit(a, b):
    dx, dz = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dz) or 1.0
    return dx / n, dz / n


def strokes(pos, adj):
    """Pair edges at every node by straightness; return lists of node ids."""
    pair = {}                                   # (node, neighbour) -> neighbour continuing the line
    cos_lim = math.cos(math.radians(STROKE_TURN))
    for k, ns in adj.items():
        ns = list(ns)
        if len(ns) < 2:
            continue
        cands = []
        for i in range(len(ns)):
            for j in range(i + 1, len(ns)):
                u = _unit(pos[k], pos[ns[i]])
                v = _unit(pos[k], pos[ns[j]])
                straight = -(u[0] * v[0] + u[1] * v[1])     # 1 = perfectly straight through
                cands.append((straight, ns[i], ns[j]))
        cands.sort(reverse=True)
        used = set()
        for straight, a, b in cands:
            if straight < cos_lim or a in used or b in used:
                continue
            used.update((a, b))
            pair[(k, a)] = b
            pair[(k, b)] = a
    seen = set()
    out = []

    def walk(a, b):
        line = [a, b]
        seen.add((a, b)); seen.add((b, a))
        while True:
            prev, cur = line[-2], line[-1]
            nxt = pair.get((cur, prev))
            if nxt is None or (cur, nxt) in seen:
                return line
            seen.add((cur, nxt)); seen.add((nxt, cur))
            line.append(nxt)

    # start from stroke ends first so open strokes are walked from an end
    for k in adj:
        for o in adj[k]:
            if (k, o) not in seen and (k, o) not in pair:
                line = walk(k, o)
                out.append(line)
    for k in adj:                                 # what is left are closed loops
        for o in adj[k]:
            if (k, o) not in seen:
                out.append(walk(k, o))
    return out


def sample(line, pos):
    pts = [pos[line[0]]]
    for a, b in zip(line, line[1:]):
        pa, pb = pos[a], pos[b]
        d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        n = max(1, int(math.ceil(d / STEP)))
        for i in range(1, n + 1):
            t = i / n
            pts.append((pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t))
    return pts


def dp(pts, tol):
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        ax, az = pts[a]; bx, bz = pts[b]
        dx, dz = bx - ax, bz - az
        L = math.hypot(dx, dz)
        best, bi = -1.0, -1
        for i in range(a + 1, b):
            px, pz = pts[i]
            if L < 1e-9:
                d = math.hypot(px - ax, pz - az)
            else:
                d = abs(dx * (pz - az) - dz * (px - ax)) / L
            if d > best:
                best, bi = d, i
        if best > tol:
            keep[bi] = True
            stack.append((a, bi)); stack.append((bi, b))
    return [p for p, k in zip(pts, keep) if k]


class Grid:
    def __init__(self, cell):
        self.cell = cell
        self.g = defaultdict(list)

    def key(self, x, z):
        return int(math.floor(x / self.cell)), int(math.floor(z / self.cell))

    def add(self, x, z, item):
        self.g[self.key(x, z)].append(item)

    def near(self, x, z, r):
        cx, cz = self.key(x, z)
        k = int(math.ceil(r / self.cell))
        for i in range(cx - k, cx + k + 1):
            for j in range(cz - k, cz + k + 1):
                yield from self.g.get((i, j), ())


def tile_lines(pos, adj, is_border=None):
    sls = strokes(pos, adj)
    S = [sample(l, pos) for l in sls]
    ends = []
    for l in sls:
        e = []
        for k in (l[0], l[-1]):
            if l[0] == l[-1] and len(l) > 2:
                e.append("loop")
            elif len(adj[k]) == 1:
                if is_border is not None:
                    e.append("border" if is_border(k) else "free")
                else:
                    x, z = pos[k]
                    e.append("border" if max(abs(x), abs(z)) > BORDER else "free")
            else:
                e.append("junction")
        ends.append(e)
    covered = [[False] * len(s) for s in S]
    dead = [[False] * len(s) for s in S]
    allpts = Grid(COVER)
    for i, s in enumerate(S):
        for j, (x, z) in enumerate(s):
            allpts.add(x, z, (i, j))
    drawn = Grid(COVER)
    drawn_far = Grid(YARD)
    result = []

    def draw(pts):
        for (x, z) in pts:
            drawn.add(x, z, (x, z))
            drawn_far.add(x, z, (x, z))
            for (i, j) in allpts.near(x, z, COVER):
                if not covered[i][j]:
                    px, pz = S[i][j]
                    if (px - x) ** 2 + (pz - z) ** 2 <= COVER * COVER:
                        covered[i][j] = True
        result.append(pts)

    def nearest_drawn(x, z, r):
        best, bp = r * r, None
        for (px, pz) in drawn.near(x, z, r):
            d = (px - x) ** 2 + (pz - z) ** 2
            if d < best:
                best, bp = d, (px, pz)
        return bp

    def dist_drawn(x, z, r):
        p = nearest_drawn(x, z, r)
        return (math.hypot(p[0] - x, p[1] - z), p) if p else (None, None)

    def merge_in(i, j, step):
        """Follow stroke i from sample j (step -1/+1) into the covered zone up to where it comes
        closest to a drawn line, so a branch joins the line along its own track, not by a stub."""
        s = S[i]
        best = None
        walked = 0.0
        k = j + step
        while 0 <= k < len(s) and walked <= MERGE_WALK:
            d, p = dist_drawn(s[k][0], s[k][1], COVER * 1.5)
            if d is None:
                break
            if best is None or d < best[0] - 0.05:
                best = (d, k, p)
            elif d > best[0] + 2.0:
                break
            if d < 0.5:
                break
            walked += STEP
            k += step
        if best is None:
            return []
        d, k, p = best
        return [p]

    def far_from_drawn(pts):
        for (x, z) in pts[::3]:
            hit = False
            for (px, pz) in drawn_far.near(x, z, YARD):
                if (px - x) ** 2 + (pz - z) ** 2 <= YARD * YARD:
                    hit = True
                    break
            if not hit:
                return True
        return False

    while True:
        runs = []
        for i, s in enumerate(S):
            j = 0
            n = len(s)
            while j < n:
                if covered[i][j] or dead[i][j]:
                    j += 1
                    continue
                a = j
                while j < n and not covered[i][j] and not dead[i][j]:
                    j += 1
                b = j - 1
                kinds = []
                for idx, side in ((a, 0), (b, 1)):
                    if (side == 0 and idx > 0) or (side == 1 and idx < n - 1):
                        nb = idx - 1 if side == 0 else idx + 1
                        d = -1 if side == 0 else 1
                        while 0 <= nb < n and dead[i][nb] and not covered[i][nb]:
                            nb += d                     # skipped station track: look past it
                        att = 0 <= nb < n and covered[i][nb]
                        if not att and not (0 <= nb < n):
                            kinds.append(ends[i][side] if ends[i][side] == "border" else "free")
                        else:
                            kinds.append("attached" if att else "free")
                    else:
                        kinds.append(ends[i][side])
                runs.append((i, a, b, kinds))
        if not runs:
            break

        def prio(r):
            i, a, b, kinds = r
            return (kinds.count("border"), (b - a))
        runs.sort(key=prio, reverse=True)
        chosen = None
        for r in runs:
            i, a, b, kinds = r
            pending = [k for k in kinds if k == "junction"]
            if pending and result and "border" not in kinds:
                continue                                  # wait: its junction may get drawn
            chosen = r
            break
        if chosen is None:
            chosen = runs[0]
        i, a, b, kinds = chosen
        pts = S[i][a:b + 1]
        length = STEP * (b - a)
        kinds = ["free" if k in ("junction", "loop") else k for k in kinds]
        if not result:
            ok = True
        elif "border" in kinds:
            ok = far_from_drawn(pts) or length < MIN_PIECE and "attached" in kinds
        elif length < MIN_PIECE:
            ok = False
        elif not result:
            ok = True
        else:
            ok = far_from_drawn(pts)
        if not ok:
            for j in range(a, b + 1):
                dead[i][j] = True
            continue
        head = merge_in(i, a, -1) if kinds[0] == "attached" else []
        tail = merge_in(i, b, +1) if kinds[1] == "attached" else []
        draw(head[::-1] + list(pts) + tail)
    return finish(result)


def finish(lines):
    """Join line ends that stop next to another line end (tile joints without a node pair) and
    drop crumbs."""
    lines = [list(l) for l in lines if length_of([l]) >= 12.0]
    changed = True
    while changed:
        changed = False
        ends = []
        for i, l in enumerate(lines):
            ends.append((i, 0)); ends.append((i, 1))
        best = None
        for a in range(len(ends)):
            i, ea = ends[a]
            pa = lines[i][0] if ea == 0 else lines[i][-1]
            for b in range(a + 1, len(ends)):
                j, eb = ends[b]
                if i == j:
                    continue
                pb = lines[j][0] if eb == 0 else lines[j][-1]
                d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                if d < JOIN and (best is None or d < best[0]):
                    best = (d, i, ea, j, eb)
        if best:
            d, i, ea, j, eb = best
            li = lines[i] if ea == 1 else lines[i][::-1]
            lj = lines[j] if eb == 0 else lines[j][::-1]
            merged = li + lj[1:] if d < 0.01 else li + lj
            lines = [l for k, l in enumerate(lines) if k not in (i, j)] + [merged]
            changed = True
    return [dp(l, SIMPLIFY) for l in lines]


def origin(t):
    m = re.match(r"(mega|arid)_island_(\d+)_(\d+)$", t)
    if not m:
        return None
    X, Y = int(m.group(2)), int(m.group(3))
    if m.group(1) == "mega":
        return (X * 1000 - 8000, Y * 1000 - 12000)
    return (X * 1000 - 24000, Y * 1000 - 40000)


def length_of(lines):
    return sum(math.hypot(p[0] - q[0], p[1] - q[1]) for l in lines for p, q in zip(l, l[1:]))


def main():
    rom, scan_path, out_path = sys.argv[1:4]
    scan = json.load(open(scan_path))
    tiles = scan["tiles"]
    partners = scan.get("partners", {})
    wpos, wadj, wlocal = {}, defaultdict(set), {}
    out = {"world": [], "tiles": {}}
    track_km = {"world": 0.0, "tiles": 0.0}
    for t in tiles:
        pos, adj = graph(os.path.join(rom, "data", "tiles", t + ".xml"))
        if not pos:
            continue
        L = sum(math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1]) for a in adj for b in adj[a] if a < b)
        o = origin(t)
        if o is None:
            lines = tile_lines(pos, adj)
            out["tiles"][t] = [[(round(x * 10), round(z * 10)) for x, z in l] for l in lines]
            track_km["tiles"] += L
            continue
        track_km["world"] += L
        for k, (x, z) in pos.items():
            wpos[(t, k)] = (x + o[0], z + o[1])
            wlocal[(t, k)] = (x, z)
        for a in adj:
            for b in adj[a]:
                wadj[(t, a)].add((t, b))
    # tile joints: collect pairs (scan partners + own pairing of loose border ends), then merge
    # each pair into one node; two nodes 1 mm apart would give the stroke walk a random direction
    pairs = []
    for t, nodes in partners.items():
        for k, v in nodes.items():
            a, b = (t, k), (v[0], v[1])
            if a in wpos and b in wpos and a != b:
                pairs.append((a, b))
    for k in wpos:
        wadj.setdefault(k, set())
    paired = {x for p in pairs for x in p}
    loose = Grid(4.0)
    for k in wpos:
        if k not in paired and len(wadj[k]) == 1 and max(map(abs, wlocal[k])) > BORDER:
            loose.add(wpos[k][0], wpos[k][1], k)
    for k in list(wpos):
        if k in paired or len(wadj[k]) != 1 or max(map(abs, wlocal[k])) <= BORDER:
            continue
        best = None
        for o in loose.near(wpos[k][0], wpos[k][1], 3.0):
            if o[0] == k[0] or o in paired:
                continue
            d = math.hypot(wpos[o][0] - wpos[k][0], wpos[o][1] - wpos[k][1])
            if d < 3.0 and (best is None or d < best[0]):
                best = (d, o)
        if best:
            pairs.append((k, best[1])); paired.update((k, best[1]))
    parent = {}

    def root(x):
        while parent.get(x, x) != x:
            x = parent[x]
        return x
    joints = 0
    for a, b in pairs:
        ra, rb = root(a), root(b)
        if ra == rb:
            continue
        parent[rb] = ra
        joints += 1
        wpos[ra] = ((wpos[ra][0] + wpos[rb][0]) / 2, (wpos[ra][1] + wpos[rb][1]) / 2)
        for n in wadj.pop(rb):
            wadj[n].discard(rb)
            if n != ra:
                wadj[n].add(ra); wadj[ra].add(n)
        del wpos[rb]
    wlocal = {k: v for k, v in wlocal.items() if k in wpos}

    def is_border(k):
        x, z = wlocal[k]
        return max(abs(x), abs(z)) > BORDER
    world = tile_lines(wpos, wadj, is_border)
    out["world"] = [[(round(x * 10), round(z * 10)) for x, z in l] for l in world]
    json.dump(out, open(out_path, "w"))
    tl = [l for v in out["tiles"].values() for l in v]
    print("joints", joints,
          "world: track km %.1f drawn km %.1f lines %d points %d" % (
              track_km["world"] / 1000, length_of(world) / 1000, len(world), sum(map(len, world))),
          "| other tiles %d: track km %.1f drawn km %.1f lines %d points %d" % (
              len(out["tiles"]), track_km["tiles"] / 1000, length_of(tl) / 10000, len(tl), sum(map(len, tl))))


if __name__ == "__main__":
    main()
