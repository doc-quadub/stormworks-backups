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
        if len(nbrs) == 2:
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


def node_tangent(nodes, pairs, k, towards, border_tangents):
    """Unit horizontal tangent at node k for the edge leaving towards `towards`."""
    p = nodes[k][0]
    chord = _dir(p, nodes[towards][0])
    if k in border_tangents:
        t = border_tangents[k]
    else:
        other = pairs[k].get(towards)
        if other is not None:
            # through route: Catmull-Rom style tangent from the paired neighbours
            t = _dir(nodes[other][0], nodes[towards][0])
        else:
            through = [(a, b) for a, b in pairs[k].items() if b is not None]
            if through:
                # diverging branch of a junction: start tangent to the through route
                a, b = through[0]
                t = _dir(nodes[a][0], nodes[b][0])
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
            chain = [a]
            for i in range(1, pieces):
                f = i / pieces
                x, z = _hermite(pa, pb, ta, tb, scale, f)
                y = pa[1] + (pb[1] - pa[1]) * f
                counter += 1
                nid = "%s%d" % (NEW_NODE_PREFIX, counter)
                new_nodes[nid] = ((x, y, z), [])
                new_y[nid] = node_y[a] + (node_y[b] - node_y[a]) * f
                inserted.append(nid)
                chain.append(nid)
            chain.append(b)
            # relink: a -> first new ... last new -> b
            new_nodes[a][1][:] = [chain[1] if o == b else o for o in new_nodes[a][1]]
            new_nodes[b][1][:] = [chain[-2] if o == a else o for o in new_nodes[b][1]]
            for i in range(1, len(chain) - 1):
                new_nodes[chain[i]][1].extend([chain[i - 1], chain[i + 1]])
    return new_nodes, new_y, inserted


def train_tracks_xml(original_text, new_nodes, new_y, newline):
    """Rebuild the <train_tracks> block: original nodes keep their transform attributes (the height
    changes; x/z only for border nodes that were moved), inserted nodes are appended."""
    import re
    block_match = re.search(r"<train_tracks>.*?</train_tracks>", original_text, re.S)
    original_attrs = {m.group(1): m.group(2) for m in re.finditer(
        r'<track id="([^"]+)">\s*<transform([^/]*)/>', block_match.group(0))}
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
        lines.append('\t\t<track id="%s">' % k)
        lines.append("\t\t\t<transform%s/>" % attrs)
        if links:
            lines.append("\t\t\t<links>")
            for o in links:
                lines.append('\t\t\t\t<link id="%s"/>' % o)
            lines.append("\t\t\t</links>")
        else:
            lines.append("\t\t\t<links/>")
        lines.append("\t\t</track>")
    lines.append("\t</train_tracks>")
    block = newline.join(lines)
    return original_text[:block_match.start()] + block + original_text[block_match.end():]
