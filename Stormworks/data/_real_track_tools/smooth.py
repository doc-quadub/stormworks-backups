"""Smooth a tile's train track graph by inserting nodes along cubic Hermite curves.

The curve passes exactly through every original node; straight runs are left untouched.
Limits (so the track does not wander away from the vanilla ballast strip, bridges, platforms):
  * at most MAX_DEVIATION metres away from the original straight segment,
  * new nodes roughly every MAX_TURN_PER_PIECE degrees of direction change.
Heights are interpolated linearly between original nodes (no vertical overshoot).
"""

import math

MAX_DEVIATION = 0.30
MAX_TURN_PER_PIECE = math.radians(1.0)
MIN_TURN_TO_SMOOTH = math.radians(0.25)
MAX_PIECES = 24
NEW_NODE_PREFIX = "train_rt_"
# The game treats track nodes lying close together as connected (that is how tiles join), so a
# new node must never come near a node it is not linked to: two gently diverging branches got
# their curve points 0.02 - 0.7 m apart and trains jumped from one branch to the other.
# Vanilla never has unlinked nodes closer than ~1.09 m.
MIN_NODE_GAP = 1.5


def _norm2(x, z):
    length = math.hypot(x, z)
    return (x / length, z / length) if length > 1e-9 else (0.0, 0.0)


def _dir(a, b):
    return _norm2(b[0] - a[0], b[2] - a[2])


def _angle(u, v):
    return math.acos(max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1])))


def node_pairs(nodes):
    """Same pairing rule as rails.build_chains: at a junction the straightest two links pair up.
    Returns {node: {neighbour: paired neighbour or None}}."""
    result = {}
    for k, (p, links) in nodes.items():
        nbrs = list(dict.fromkeys(o for o in links if o in nodes and o != k))
        pairing = {o: None for o in nbrs}
        if len(nbrs) == 2 and not is_junction(nodes, k):
            pairing[nbrs[0]], pairing[nbrs[1]] = nbrs[1], nbrs[0]
        elif len(nbrs) >= 3:
            best = None
            for i in range(len(nbrs)):
                for j in range(i + 1, len(nbrs)):
                    da = _dir(p, nodes[nbrs[i]][0])
                    db = _dir(p, nodes[nbrs[j]][0])
                    straight = -(da[0] * db[0] + da[1] * db[1])
                    if best is None or straight > best[0]:
                        best = (straight, nbrs[i], nbrs[j])
            _, a, b = best
            pairing[a], pairing[b] = b, a
        result[k] = pairing
    return result


def is_junction(nodes, k):
    """A switch: three or more links, or two links leaving on the same side (a switch on a tile
    border whose third link is in the neighbouring tile)."""
    nbrs = list(set(o for o in nodes[k][1] if o in nodes and o != k))
    if len(nbrs) >= 3:
        return True
    if len(nbrs) == 2:
        p = nodes[k][0]
        a, b = _dir(p, nodes[nbrs[0]][0]), _dir(p, nodes[nbrs[1]][0])
        return a[0] * b[0] + a[1] * b[1] > 0
    return False


def node_tangent(nodes, pairs, k, towards, border_tangents):
    """Unit horizontal tangent at node k for the edge leaving towards `towards`."""
    p = nodes[k][0]
    chord = _dir(p, nodes[towards][0])
    other = pairs[k].get(towards)
    if k in border_tangents:
        t = border_tangents[k]
    elif other is not None and is_junction(nodes, other):
        # the edge on the other side ends at a junction and stays straight (vanilla): carry
        # its direction on, so the curve joins it without a kink
        t = _dir(nodes[other][0], p)
    elif other is not None:
        # through route: Catmull-Rom style tangent from the paired neighbours
        t = _dir(nodes[other][0], nodes[towards][0])
    else:
        t = chord
    if t[0] * chord[0] + t[1] * chord[1] < 0:
        t = (-t[0], -t[1])
    if t == (0.0, 0.0):
        t = chord
    return t


def _hermite(a, b, ta, tb, scale, f):
    """Horizontal cubic Hermite point between a and b (tangents are unit, scaled by `scale`)."""
    h00 = 2 * f ** 3 - 3 * f ** 2 + 1
    h10 = f ** 3 - 2 * f ** 2 + f
    h01 = -2 * f ** 3 + 3 * f ** 2
    h11 = f ** 3 - f ** 2
    x = h00 * a[0] + h10 * ta[0] * scale + h01 * b[0] + h11 * tb[0] * scale
    z = h00 * a[2] + h10 * ta[1] * scale + h01 * b[2] + h11 * tb[1] * scale
    return x, z


def _max_deviation(a, b, ta, tb, scale):
    chord = _dir(a, b)
    normal = (-chord[1], chord[0])
    worst = 0.0
    for i in range(1, 16):
        x, z = _hermite(a, b, ta, tb, scale, i / 16)
        worst = max(worst, abs((x - a[0]) * normal[0] + (z - a[2]) * normal[1]))
    return worst


def smooth_graph(nodes, node_y, border_tangents=None, skip=frozenset()):
    """Return (new_nodes, new_y, inserted_ids).

    nodes: {id: ((x, y, z), [links])}; node_y: {id: lifted y}.
    border_tangents: {id: unit (x, z) tangent} for border nodes shared with a neighbouring tile.
    skip: node ids whose edges must stay as they are (e.g. stray pieces that are not drawn).
    """
    border_tangents = border_tangents or {}
    pairs = node_pairs(nodes)
    grid = {}

    def cell(x, z):
        return (int(math.floor(x / MIN_NODE_GAP)), int(math.floor(z / MIN_NODE_GAP)))

    def register(k, x, z):
        grid.setdefault(cell(x, z), []).append((k, x, z))

    def clear_of_others(x, z, own):
        cx, cz = cell(x, z)
        for ix in (cx - 1, cx, cx + 1):
            for iz in (cz - 1, cz, cz + 1):
                for k, kx, kz in grid.get((ix, iz), ()):
                    if k not in own and math.hypot(kx - x, kz - z) < MIN_NODE_GAP:
                        return False
        return True

    for k, (p, _) in nodes.items():
        register(k, p[0], p[2])
    new_nodes = {k: (p, list(links)) for k, (p, links) in nodes.items()}
    new_y = dict(node_y)
    inserted = []
    counter = 0
    done = set()
    for a, (pa, links) in nodes.items():
        for b in links:
            if b not in nodes or b == a or frozenset((a, b)) in done:
                continue
            done.add(frozenset((a, b)))
            if a in skip or b in skip:
                continue
            if is_junction(nodes, a) or is_junction(nodes, b):
                # Junction edges keep the vanilla geometry: the game picks the route at a switch
                # (forward / left / right) from the directions of these links, and bending them
                # broke switching and squeezed the turnout curve into a sharp hook.
                continue
            pb = nodes[b][0]
            length = math.hypot(pb[0] - pa[0], pb[2] - pa[2])
            if length < 1.0:
                continue
            ta = node_tangent(nodes, pairs, a, b, border_tangents)
            tb_out = node_tangent(nodes, pairs, b, a, border_tangents)
            tb = (-tb_out[0], -tb_out[1])                     # direction of travel a -> b
            chord = _dir(pa, pb)
            turn = _angle(ta, chord) + _angle(tb, chord)
            if turn < MIN_TURN_TO_SMOOTH:
                continue
            scale = length
            deviation = _max_deviation(pa, pb, ta, tb, scale)
            if deviation > MAX_DEVIATION:
                scale *= MAX_DEVIATION / deviation
            pieces = min(MAX_PIECES, max(2, int(math.ceil(turn / MAX_TURN_PER_PIECE))))
            # pieces no shorter than MIN_NODE_GAP, so even nodes two steps apart stay apart
            pieces = min(pieces, int(length // MIN_NODE_GAP))
            if pieces < 2:
                continue
            chain = [a]
            for i in range(1, pieces):
                f = i / pieces
                x, z = _hermite(pa, pb, ta, tb, scale, f)
                y = pa[1] + (pb[1] - pa[1]) * f
                if not clear_of_others(x, z, (a, b)):
                    continue          # would sit next to another track's node: leave it out
                counter += 1
                nid = "%s%d" % (NEW_NODE_PREFIX, counter)
                new_nodes[nid] = ((x, y, z), [])
                new_y[nid] = node_y[a] + (node_y[b] - node_y[a]) * f
                inserted.append(nid)
                chain.append(nid)
            chain.append(b)
            if len(chain) == 2:
                continue
            for nid in chain[1:-1]:
                register(nid, new_nodes[nid][0][0], new_nodes[nid][0][2])
            # relink: a -> first new ... last new -> b
            new_nodes[a][1][:] = [chain[1] if o == b else o for o in new_nodes[a][1]]
            new_nodes[b][1][:] = [chain[-2] if o == a else o for o in new_nodes[b][1]]
            for i in range(1, len(chain) - 1):
                new_nodes[chain[i]][1].extend([chain[i - 1], chain[i + 1]])
    fixed = set(skip) | set(border_tangents)
    for k in nodes:
        if is_junction(nodes, k):
            fixed.add(k)
            fixed.update(o for o in nodes[k][1] if o in nodes)    # junction edges stay vanilla
    counter = fair_graph(new_nodes, new_y, fixed, inserted, counter)
    return new_nodes, new_y, inserted


# Fairing: the Hermite pieces pass exactly through every vanilla node, so wherever the vanilla
# polyline is irregular (a node a little off the line, short and long segments mixed, kinks of
# 2 - 8 degrees at single nodes) the curvature still jumps and trains jerk; on long curves of
# large radius as much as on short tight ones. Each run of plain nodes between fixed ones
# (switches and their edges, tile borders, track ends) is resampled every FAIR_SPACING metres,
# smoothed by a bi-Laplacian (removes curvature jumps, keeps a circular curve circular) with the
# shift limited to FAIR_MAX_SHIFT sideways and FAIR_MAX_RISE in height, and its nodes are moved
# onto the result; more nodes are added where the faired curve turns.
FAIR_SPACING = 3.0
FAIR_MAX_SHIFT = 0.25
FAIR_MAX_RISE = 0.15
FAIR_ITERATIONS = 400
FAIR_RATE = 0.1
FAIR_MAX_TURN = math.radians(0.75)
FAIR_MIN_PIECE = 0.8


def _fair_runs(nodes, fixed):
    """Runs of nodes [a, ..., z]: a and z fixed or not of degree 2, all between of degree 2."""
    def nbrs(k):
        return [o for o in dict.fromkeys(nodes[k][1]) if o in nodes and o != k]

    def plain(k):
        return k not in fixed and len(nbrs(k)) == 2

    runs, seen = [], set()
    for a in nodes:
        if plain(a):
            continue
        for b in nbrs(a):
            if (a, b) in seen:
                continue
            run = [a, b]
            seen.add((a, b))
            while plain(run[-1]):
                nxt = [o for o in nbrs(run[-1]) if o != run[-2]]
                if not nxt or nxt[0] in run:
                    break
                run.append(nxt[0])
            seen.add((run[-1], run[-2]))
            if len(run) >= 3:
                runs.append(run)
    return runs


def fair_graph(nodes, node_y, fixed, inserted, counter):
    import numpy as np
    grid = {}

    def cell(x, z):
        return (int(math.floor(x / MIN_NODE_GAP)), int(math.floor(z / MIN_NODE_GAP)))

    for k, (p, _) in nodes.items():
        grid.setdefault(cell(p[0], p[2]), set()).add(k)

    def clear(x, z, own):
        cx, cz = cell(x, z)
        for ix in (cx - 1, cx, cx + 1):
            for iz in (cz - 1, cz, cz + 1):
                for k in grid.get((ix, iz), ()):
                    if k not in own:
                        q = nodes[k][0]
                        if math.hypot(q[0] - x, q[2] - z) < MIN_NODE_GAP:
                            return False
        return True

    def move(k, x, y, z):
        p = nodes[k][0]
        grid[cell(p[0], p[2])].discard(k)
        nodes[k] = ((x, y, z), nodes[k][1])
        grid.setdefault(cell(x, z), set()).add(k)

    for run in _fair_runs(nodes, fixed):
        pts = np.array([nodes[k][0] for k in run], dtype=float)
        lift = np.array([node_y[k] - nodes[k][0][1] for k in run])
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 2]))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = s[-1]
        m = int(math.ceil(total / FAIR_SPACING))
        if m < 6:
            continue
        u = np.linspace(0.0, total, m + 1)
        dense0 = np.stack([np.interp(u, s, pts[:, i]) for i in range(3)], axis=1)
        dense = dense0.copy()
        for _ in range(FAIR_ITERATIONS):
            d = dense[:-4] - 4 * dense[1:-3] + 6 * dense[2:-2] - 4 * dense[3:-1] + dense[4:]
            dense[2:-2] -= FAIR_RATE * d
            shift = dense[2:-2] - dense0[2:-2]
            horizontal = np.hypot(shift[:, 0], shift[:, 2])
            scale = np.minimum(1.0, FAIR_MAX_SHIFT / np.maximum(horizontal, 1e-9))
            shift[:, 0] *= scale
            shift[:, 2] *= scale
            shift[:, 1] = np.clip(shift[:, 1], -FAIR_MAX_RISE, FAIR_MAX_RISE)
            dense[2:-2] = dense0[2:-2] + shift

        def at(sv):
            return [float(np.interp(sv, u, dense[:, i])) for i in range(3)]

        # move the run's own nodes onto the faired curve (not next to another track's node)
        for i, k in enumerate(run[1:-1], start=1):
            x, y, z = at(s[i])
            own = {run[i - 1], k, run[i + 1]}
            if clear(x, z, own):
                move(k, x, y, z)
                node_y[k] = y + lift[i]
        # more nodes where the faired curve turns
        for i in range(len(run) - 1):
            a, b = run[i], run[i + 1]
            length = s[i + 1] - s[i]
            if length < 2 * FAIR_MIN_PIECE:
                continue
            e = 0.25
            p0, p1 = at(max(0.0, s[i] - e)), at(s[i] + e)
            q0, q1 = at(s[i + 1] - e), at(min(total, s[i + 1] + e))
            h0 = math.atan2(p1[2] - p0[2], p1[0] - p0[0])
            h1 = math.atan2(q1[2] - q0[2], q1[0] - q0[0])
            turn = abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)
            pieces = min(int(math.ceil(turn / FAIR_MAX_TURN)), int(length // FAIR_MIN_PIECE), MAX_PIECES)
            if pieces < 2:
                continue
            chain = [a]
            for j in range(1, pieces):
                sv = s[i] + length * j / pieces
                x, y, z = at(sv)
                if not clear(x, z, {a, b}):
                    continue
                counter += 1
                nid = "%s%d" % (NEW_NODE_PREFIX, counter)
                nodes[nid] = ((x, y, z), [])
                grid.setdefault(cell(x, z), set()).add(nid)
                f = (sv - s[i]) / length
                node_y[nid] = y + lift[i] + (lift[i + 1] - lift[i]) * f
                inserted.append(nid)
                chain.append(nid)
            chain.append(b)
            if len(chain) == 2:
                continue
            nodes[a][1][:] = [chain[1] if o == b else o for o in nodes[a][1]]
            nodes[b][1][:] = [chain[-2] if o == a else o for o in nodes[b][1]]
            for j in range(1, len(chain) - 1):
                nodes[chain[j]][1].extend([chain[j - 1], chain[j + 1]])
    return counter


def train_tracks_xml(original_text, new_nodes, new_y, newline, raise_kept=0.0):
    """Rebuild the <train_tracks> block: original nodes keep their transform attributes (the height
    changes; x/z only for border nodes that were moved) and any extra attributes of the <track>
    element (seasonal_flags), inserted nodes are appended. Original nodes the graph does not
    contain (seasonal-only nodes, which the track reader skips) are kept as they are, lifted by
    raise_kept like the rest of the graph, so no link points to a missing node."""
    import re
    block_match = re.search(r"<train_tracks>.*?</train_tracks>", original_text, re.S)
    original_attrs = {}
    element_attrs = {}
    kept_blocks = []
    for m in re.finditer(r'<track id="([^"]+)"([^>]*)>(\s*<transform([^/]*)/>.*?)</track>',
                         block_match.group(0), re.S):
        node_id, extra, body, transform = m.group(1), m.group(2), m.group(3), m.group(4)
        if node_id in new_nodes:
            original_attrs[node_id] = transform
            element_attrs[node_id] = extra
        else:
            old_y = re.search(r'\b31="([^"]*)"', transform)
            if old_y:
                lifted = transform.replace(old_y.group(0), '31="%.6f"' % (float(old_y.group(1)) + raise_kept))
            else:
                lifted = transform.rstrip() + ' 31="%.6f"' % raise_kept
            kept_blocks.append('\t\t<track id="%s"%s>%s</track>'
                               % (node_id, extra, body.replace(transform, lifted, 1)))
    order = list(original_attrs) + [k for k in new_nodes if k not in original_attrs]
    lines = ["<train_tracks>"]
    for k in order:
        p, links = new_nodes[k]
        if k in original_attrs:
            attrs = original_attrs[k]
            old = dict(re.findall(r'\b(3[012])="([^"]*)"', attrs))
            values = {"31": new_y[k]}
            if abs(float(old.get("30", 0)) - p[0]) > 1e-5:
                values["30"] = p[0]
            if abs(float(old.get("32", 0)) - p[2]) > 1e-5:
                values["32"] = p[2]
            for key, number in values.items():
                text = '%s="%.6f"' % (key, number)
                if key in old:
                    attrs = re.sub(r'\b%s="[^"]*"' % key, text, attrs)
                else:
                    attrs = attrs.rstrip() + " " + text
        else:
            attrs = ' 30="%.6f" 31="%.6f" 32="%.6f"' % (p[0], new_y[k], p[2])
        lines.append('\t\t<track id="%s"%s>' % (k, element_attrs.get(k, "")))
        lines.append("\t\t\t<transform%s/>" % attrs)
        if links:
            lines.append("\t\t\t<links>")
            for o in links:
                lines.append('\t\t\t\t<link id="%s"/>' % o)
            lines.append("\t\t\t</links>")
        else:
            lines.append("\t\t\t<links/>")
        lines.append("\t\t</track>")
    lines.extend(kept_blocks)
    lines.append("\t</train_tracks>")
    block = newline.join(lines)
    return original_text[:block_match.start()] + block + original_text[block_match.end():]
