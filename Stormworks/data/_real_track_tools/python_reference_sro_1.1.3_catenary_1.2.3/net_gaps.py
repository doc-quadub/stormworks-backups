"""python3 net_gaps.py <rom> <work> family: track not counted as line (no posts) grouped in clusters."""
import sys, json, math
sys.path.insert(0, ".")
import build_world, signs
rom, work, fam = sys.argv[1:4]
scan = json.load(open(work + "/scan.json"))
net = signs.build_network(scan["tiles"], lambda t: build_world.prepare_graph(rom, t, scan), fam)
nw, links, dist, ls = net.nodes_world, net.links, net.dist, net.line_set
seen = set(); groups = {}
for a in nw:
    for b in links[a]:
        e = frozenset((a, b))
        if e in seen: continue
        seen.add(e)
        if (a, b) in ls or (b, a) in ls: continue
        L = math.dist(nw[a], nw[b])
        state = "unreached" if a not in dist or b not in dist else "notline"
        c = (round(nw[a][0], -3), round(nw[a][2], -3), state)
        g = groups.setdefault(c, [0, None, None])
        g[0] += L
        if g[1] is None: g[1] = (round(nw[a][0]), round(nw[a][2]), a, round(dist.get(a, -1)), round(dist.get(b, -1)), round(L))
for c, g in sorted(groups.items(), key=lambda kv: -kv[1][0]):
    if g[0] > 150: print(c, round(g[0]), g[1])
