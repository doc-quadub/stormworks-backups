import json, sys, re, collections
ORIGINS = {"mega_island": (-8000, -12000), "arid_island": (-24000, -40000)}
q = json.load(open(sys.argv[1]))
def local(tile, w):
    m = re.fullmatch(r"(.+)_(\d+)_(\d+)", tile)
    if m is None or m.group(1) not in ORIGINS: return w
    ox, oz = ORIGINS[m.group(1)]
    return (w[0] - 1000 * int(m.group(2)) - ox, w[1] - 1000 * int(m.group(3)) - oz)
c = collections.Counter(); ex = collections.defaultdict(list)
for deg, l1, l2, junction, ins, w, tile, k in q["kinks"]:
    if deg <= 1.5: continue
    lx, lz = local(tile, w)
    fam = tile.split("_")[0]
    if junction: cls = "junction"
    elif max(abs(lx), abs(lz)) > 495: cls = "border"
    elif min(l1, l2) < 1.0: cls = "short piece"
    elif ins: cls = "inserted"
    else: cls = "vanilla node"
    c[(fam, cls)] += 1; ex[(fam, cls)].append((deg, l1, l2, w, tile, k))
for key, n in sorted(c.items()): print(key, n, sorted(ex[key], reverse=True)[:4])
g = collections.Counter()
for dg, l1, l2, dy, w, tile, k in q["grades"]:
    if dg < 1.0: continue
    lx, lz = local(tile, w)
    g[(tile.split("_")[0], "border" if max(abs(lx), abs(lz)) > 495 else "inner", "short" if min(l1, l2) < 5 else "long")] += 1
print(g)
