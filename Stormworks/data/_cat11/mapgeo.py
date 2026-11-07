"""python3 mapgeo.py file.bin [x z]: records of a *_map_geometry.bin (u16 vertex count, xyz floats, u16 index count, u16 indices)."""
import struct, sys
data = open(sys.argv[1], "rb").read()
pt = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else None
off = 0; rec = 0
while off + 2 <= len(data):
    n = struct.unpack_from("<H", data, off)[0]; off += 2
    verts = [struct.unpack_from("<3f", data, off + 12 * i) for i in range(n)]; off += 12 * n
    if off + 2 > len(data): print("trailing"); break
    m = struct.unpack_from("<H", data, off)[0]; off += 2
    idx = struct.unpack_from("<%dH" % m, data, off); off += 2 * m
    ys = sorted(set(round(v[1], 2) for v in verts))
    hit = 0
    if pt:
        for i in range(0, m - 2, 3):
            a, b, c = verts[idx[i]], verts[idx[i+1]], verts[idx[i+2]]
            d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
            if abs(d) < 1e-12: continue
            l1 = ((pt[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(pt[1]-a[2]))/d; l2 = ((b[0]-a[0])*(pt[1]-a[2]) - (pt[0]-a[0])*(b[2]-a[2]))/d
            if l1 >= 0 and l2 >= 0 and l1 + l2 <= 1: hit += 1
    print("rec", rec, "verts", n, "idx", m, "y", ys[:4], "hit", hit)
    rec += 1
print("end", off, len(data))
