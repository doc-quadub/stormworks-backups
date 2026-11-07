"""Map geometry of a mainland tile: 11 polygon layers (xyz, indexed), then 12 triangle-strip layers (xyz uv).
python3 maproads.py rom x,z ... : for each world point, the strip layers whose ribbon covers it or is near (<3 m)."""
import struct, os, sys, math
def load(rom, tile):
    d = open(os.path.join(rom, "data", "tiles", tile + "_map_geometry.bin"), "rb").read()
    off = 0
    for rec in range(11):
        n = struct.unpack_from("<H", d, off)[0]; off += 2 + 12 * n
        m = struct.unpack_from("<H", d, off)[0]; off += 2 + 2 * m
    layers = []
    while off + 2 <= len(d):
        n = struct.unpack_from("<H", d, off)[0]; off += 2
        layers.append([struct.unpack_from("<5f", d, off + 20 * i) for i in range(n)]); off += 20 * n
    return layers
def point_in(p, a, b, c):
    d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
    if abs(d) < 1e-9: return False
    l1 = ((p[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(p[1]-a[2]))/d; l2 = ((b[0]-a[0])*(p[1]-a[2]) - (p[0]-a[0])*(b[2]-a[2]))/d
    return l1 >= 0 and l2 >= 0 and l1 + l2 <= 1
if __name__ == "__main__":
    rom = sys.argv[1]
    for arg in sys.argv[2:]:
        x, z = map(float, arg.split(","))
        tx = int(math.floor((x + 8500) / 1000)); tz = int(math.floor((z + 12500) / 1000))
        tile = "mega_island_%d_%d" % (tx, tz)
        L = load(rom, tile); p = (x - (tx * 1000 - 8000), z - (tz * 1000 - 12000))
        out = []
        for li, v in enumerate(L):
            inside = any(point_in(p, v[i], v[i+1], v[i+2]) for i in range(len(v) - 2))
            near = min((math.hypot(q[0]-p[0], q[2]-p[1]) for q in v), default=None)
            if inside or (near is not None and near < 12): out.append((li, inside, round(near, 1)))
        print(arg, tile, out)

def ribbons(layer, max_width=16.0, max_step=40.0):
    """Quads (a0, a1, b1, b0) of a strip layer whose two sides are a ribbon (not a join between ribbons)."""
    quads = []
    for i in range(0, len(layer) - 3, 2):
        a0, a1, b0, b1 = layer[i], layer[i + 1], layer[i + 2], layer[i + 3]
        w0 = math.hypot(a0[0] - a1[0], a0[2] - a1[2]); w1 = math.hypot(b0[0] - b1[0], b0[2] - b1[2])
        st = math.hypot(a0[0] - b0[0], a0[2] - b0[2])
        if w0 < max_width and w1 < max_width and st < max_step and w0 > 0.5:
            quads.append((a0, a1, b1, b0, w0))
    return quads
