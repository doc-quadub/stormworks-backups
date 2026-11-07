import json, math, os, sys
sys.path.insert(0, '.')
import catenary, signs
rom = os.path.expandvars('$HOME/mnt/Stormworks/rom')
P = json.load(open(sys.argv[1])); S = P['supports']
G = json.load(open('../_cat_full/work/diag.json')); pos, links = G['pos'], G['links']
idx = signs.SegmentIndex()
for k, ls in links.items():
    for o in ls:
        if k < o and o in pos:
            idx.add((pos[k][0], pos[k][2]), (pos[o][0], pos[o][2]), None)
def tdist(x, z):
    return min([signs._point_segment_distance((x, z), a, b) for a, b, _ in idx.near(x, z, 10)] + [10])
sig, fac = catenary.load_signals(rom)
class N: pass
net = N(); net.signals, net.signal_facings = sig, fac
bad_guy, cross, brk, shade = [], [], [], []
for i, sp in enumerate(S):
    if sp.get('dropped'): continue
    a = sp.get('anchor')
    if a:
        g = a['guy_ground']; d = tdist(g[0], g[2])
        if d < 2.6: bad_guy.append((i, round(d, 2), [round(v) for v in sp['pole']]))
        if catenary._crosses_track(idx, (sp['pole'][0], sp['pole'][2]), (g[0], g[2])): cross.append((i, [round(v) for v in sp['pole']]))
        if sp.get('bracket') is not None or g[1] < sp['track'][1] - 6: brk.append((i, [round(v) for v in sp['pole']], sp.get('bracket'), round(sp['track'][1] - g[1], 1)))
    if sp.get('pole') is not None and sp['portal'] is None and sp.get('pole_owner') is None:
        if catenary._shades_signal(net, (sp['pole'][0], sp['pole'][2])): shade.append((i, [round(v) for v in sp['pole']]))
print("guy blocks < 2.6 m from a track:", bad_guy)
print("guy crossing a track:", cross)
print("anchors on consoles / guy > 6 m down:", brk)
print("poles in front of signals:", shade)
for x, z in ((3784, -8263), (1545, -3765), (6965, -9230), (1332, -3767)):
    near = [(i, [round(v, 1) for v in sp['pole']], 'A' if sp.get('anchor') else '', sp.get('overlap_of')) for i, sp in enumerate(S)
            if not sp.get('dropped') and sp.get('pole') and math.hypot(sp['pole'][0] - x, sp['pole'][2] - z) < 25]
    print((x, z), near)
