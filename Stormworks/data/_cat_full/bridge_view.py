"""Side view of a track stretch: python3 bridge_view.py <rom> x0,z0 x1,z1 rail_y out.png [side]"""
import math, sys
sys.path.insert(0, ".")
import render_world, zrender
rom = sys.argv[1]
(x0, z0), (x1, z1) = [tuple(map(float, a.split(","))) for a in sys.argv[2:4]]
rail_y = float(sys.argv[4]); out = sys.argv[5]; side = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
L = math.hypot(x1 - x0, z1 - z0)
d = ((x1 - x0) / max(L, 1e-6), (z1 - z0) / max(L, 1e-6)) if L > 1 else (1.0, 0.0)
n = (d[1] * side, -d[0] * side)
dist = max(60.0, L * 0.9 + 50)
T = render_world.tris_world("nowork", rom, cx, cz, L / 2 + 90, terrain=True)
eye = (cx + n[0] * dist, rail_y + dist * 0.25, cz + n[1] * dist)
zrender.render(T, eye, (cx, rail_y - 8, cz), out, fov=60, title=out)
print(out, len(T))
