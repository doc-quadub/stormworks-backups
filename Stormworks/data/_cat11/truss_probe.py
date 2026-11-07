"""python3 truss_probe.py rom plan.json : for every truss girder support: vertical rays up from the track
axis (offsets along the track) against the truss-coloured triangles, and horizontal rays to the walls."""
import sys, os, re, math, json
sys.path.insert(0, ".")
import swmesh, signs
import catenary as C
rom = sys.argv[1]; plan = json.load(open(sys.argv[2]))
tris = {}
def load(tile):
    if tile in tris: return tris[tile]
    out = []; origin = signs.mainland_origin(tile)
    path = os.path.join(rom, "data", "tiles", tile + ".xml")
    if origin and os.path.exists(path):
        text = open(path, encoding="utf-8", errors="replace").read()
        for m in re.finditer(r"<mesh ([^>]*)>\s*<transform([^/]*)/>", text):
            attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
            if "parent_id" in attrs or "file_name" not in attrs: continue
            mp = os.path.join(rom, *attrs["file_name"].split("/"))
            if not os.path.exists(mp): continue
            mx = swmesh.parse_transform(m.group(2)); mesh = swmesh.read_mesh_full(mp)
            V, I = mesh.vertices, mesh.indices
            for i in range(0, len(I) - 2, 3):
                a, b, c = V[I[i]], V[I[i+1]], V[I[i+2]]
                if tuple(a[3:6]) != C.TRUSS_COLOR: continue
                pts = []
                for v in (a, b, c):
                    x, y, z = signs._transform((v[0], v[1], v[2]), mx); pts.append((x + origin[0], y, z + origin[1]))
                out.append(pts)
    tris[tile] = out; return out
def tile_of(x, z): return "mega_island_%d_%d" % (int(math.floor((x + 8500) / 1000)), int(math.floor((z + 12500) / 1000)))
def ray_up(x, z, T):
    hs = []
    for a, b, c in T:
        d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
        if abs(d) < 1e-12: continue
        l1 = ((x-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(z-a[2]))/d; l2 = ((b[0]-a[0])*(z-a[2]) - (x-a[0])*(b[2]-a[2]))/d
        if l1 < 0 or l2 < 0 or l1 + l2 > 1: continue
        hs.append(a[1] + l1*(b[1]-a[1]) + l2*(c[1]-a[1]))
    return sorted(hs)
S = plan["supports"]; P = plan["portals"]
for i, sp in enumerate(S):
    if sp["portal"] is None or P[sp["portal"]].get("kind") != "truss" or sp.get("dropped"): continue
    p = sp["track"]; t = sp["t"]; r = signs.right_of(tuple(t))
    T = [tr for tr in load(tile_of(p[0], p[2])) if min(abs(q[0]-p[0]) + abs(q[2]-p[2]) for q in tr) < 40]
    row = []
    for k in range(-12, 13):
        a = k * 0.5; x, z = p[0] + t[0] * a, p[2] + t[1] * a
        hs = [round(h - p[1], 2) for h in ray_up(x, z, T) if 6.5 < h - p[1] < 16]
        if hs: row.append((a, hs[:3]))
    print(i, [round(v) for v in p[::2]], "beam_y-rail", round(P[sp["portal"]]["beam_y"] - p[1], 2), "ups:", row)
