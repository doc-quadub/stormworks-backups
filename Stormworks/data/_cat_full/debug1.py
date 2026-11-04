import json, math, os, re, sys
sys.path.insert(0, '.')
import catenary, signs
rom = os.path.expandvars('$HOME/mnt/Stormworks/rom')
P = json.load(open('work/catenary_t18.json')); S = P['supports']
G = json.load(open('work/diag.json')); pos, links = G['pos'], G['links']
def nodes_near(x, z, r):
    out = []
    for k, p in pos.items():
        if math.hypot(p[0] - x, p[2] - z) < r:
            out.append((k, [round(v, 2) for v in p], links.get(k)))
    return out
for x, z in ((2850, -4504), (5527, -7580)):
    print("=== graph near", x, z)
    for n in sorted(nodes_near(x, z, 45), key=lambda a: a[1][0]): print("  ", n)
# signals with full transform near 3816,-8238
text = open(os.path.join(rom, *catenary.SIGNAL_PLAYLIST.split('/')), encoding='utf-8', errors='replace').read()
print("=== signals near 3816,-8238 and NH")
for loc in re.finditer(r'<l id="\d+" tile="data/tiles/([^"]+)\.xml".*?</l>', text, re.S):
    origin = signs.mainland_origin(loc.group(1))
    if origin is None: continue
    for zone in re.finditer(r'cargo_zone_import_cargo_type="railroad_signal[^"]*".*?<spawn_transform([^/]*)/>', loc.group(0), re.S):
        v = dict(re.findall(r'(\d\d)="([^"]*)"', zone.group(1)))
        x, z = float(v.get("30", 0)) + origin[0], float(v.get("32", 0)) + origin[1]
        if math.hypot(x - 3816, z + 8238) < 60:
            print("  ", round(x, 1), round(z, 1), {k: v[k] for k in sorted(v) if k not in ("30", "31", "32")}, "y", v.get("31"))
print("=== supports near 3816,-8238")
for i, sp in enumerate(S):
    if math.hypot(sp['track'][0] - 3816, sp['track'][2] + 8238) < 60 and sp.get('pole'):
        print("  ", i, [round(v, 1) for v in sp['track']], "pole", [round(v, 1) for v in sp['pole']], "t", [round(v, 2) for v in sp['t']])
print("=== anchors: guy block distance to tracks (all), near NH and 6970,-9223")
net_tracks = signs.SegmentIndex()
for k, ls in links.items():
    for o in ls:
        if k < o and o in pos:
            net_tracks.add((pos[k][0], pos[k][2]), (pos[o][0], pos[o][2]), None)
bad = []
for i, sp in enumerate(S):
    a = sp.get('anchor')
    if not a or sp.get('dropped'): continue
    g = a['guy_ground']
    d = min([signs._point_segment_distance((g[0], g[2]), sa, sb) for sa, sb, _ in net_tracks.near(g[0], g[2], 10)] + [10])
    hs = []
    if d < 2.6 or math.hypot(sp['pole'][0] - 6970, sp['pole'][2] + 9223) < 40 or math.hypot(sp['pole'][0] - 1500, sp['pole'][2] + 3765) < 250:
        bad.append((i, round(d, 2), [round(v, 1) for v in sp['pole']], [round(v, 1) for v in g], sp.get('bracket'), sp.get('section_anchor'), sp.get('overlap_of')))
for b in bad: print("  ", b)
print("=== anchors on brackets / guy far from ground")
for i, sp in enumerate(S):
    a = sp.get('anchor')
    if not a or sp.get('dropped'): continue
    g = a['guy_ground']
    if sp.get('bracket') is not None or g[1] < sp['track'][1] - 3:
        print("  ", i, [round(v, 1) for v in sp['track']], "bracket", sp.get('bracket'), "guy y", round(g[1], 1))
