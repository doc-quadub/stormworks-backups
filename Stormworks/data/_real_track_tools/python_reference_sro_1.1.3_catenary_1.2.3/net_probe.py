"""python3 net_probe.py <rom> <work> family x z r: network nodes near a point with dist, links, line edges."""
import sys, json, math
sys.path.insert(0, ".")
import build_world, signs
rom, work, fam = sys.argv[1:4]
X, Z, R = map(float, sys.argv[4:7])
scan = json.load(open(work + "/scan.json"))
net = signs.build_network(scan["tiles"], lambda t: build_world.prepare_graph(rom, t, scan), fam)
nw, links, dist, ls = net.nodes_world, net.links, net.dist, net.line_set
for k, p in sorted(nw.items(), key=lambda kv: kv[1][0]):
    if math.hypot(p[0] - X, p[2] - Z) < R:
        print(k[0][12:], k[1], "(%.1f,%.2f,%.1f)" % p, "d=%.0f" % dist.get(k, -1), "edge" if net.on_edge(k) else "",
              [(o[0][12:] + ":" + o[1], round(math.dist(p, nw[o]), 1), "L" if (k, o) in ls or (o, k) in ls else "-") for o in links[k]])
