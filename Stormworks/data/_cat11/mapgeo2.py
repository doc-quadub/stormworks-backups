"""python3 mapgeo2.py file.bin [x z]: 11 polygon records (xyz), then strip records (xyz uv)."""
import struct, sys
data = open(sys.argv[1], "rb").read()
pt = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else None
off = 0
def hit(verts, idx):
    h = 0
    for i in range(0, len(idx) - 2, 3):
        a, b, c = verts[idx[i]], verts[idx[i+1]], verts[idx[i+2]]
        d = (b[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(b[2]-a[2])
        if abs(d) < 1e-12: continue
        l1 = ((pt[0]-a[0])*(c[2]-a[2]) - (c[0]-a[0])*(pt[1]-a[2]))/d; l2 = ((b[0]-a[0])*(pt[1]-a[2]) - (pt[0]-a[0])*(b[2]-a[2]))/d
        if l1 >= 0 and l2 >= 0 and l1 + l2 <= 1: h += 1
    return h
for rec in range(11):
    n = struct.unpack_from("<H", data, off)[0]; off += 2
    verts = [struct.unpack_from("<3f", data, off + 12 * i) for i in range(n)]; off += 12 * n
    m = struct.unpack_from("<H", data, off)[0]; off += 2
    idx = struct.unpack_from("<%dH" % m, data, off); off += 2 * m
    print("poly", rec, n, m, "hit", hit(verts, idx) if pt else "")
rec = 0
while off + 4 <= len(data):
    tag = struct.unpack_from("<H", data, off)[0]; off += 2
    n = struct.unpack_from("<H", data, off)[0]; off += 2
    verts = [struct.unpack_from("<5f", data, off + 20 * i) for i in range(n)]; off += 20 * n
    if off + 2 > len(data): print("trunc"); break
    m = struct.unpack_from("<H", data, off)[0]; off += 2
    if off + 2 * m > len(data): print("bad idx", tag, n, m); break
    idx = struct.unpack_from("<%dH" % m, data, off); off += 2 * m
    if m and max(idx) >= n: print("idx out of range", tag, n, m); break
    us = sorted(set(round(v[3], 2) for v in verts))
    print("strip", rec, "tag", tag, "verts", n, "idx", m, "u", us[:3], us[-2:], "hit", hit(verts, idx) if pt else "")
    rec += 1
print("end", off, len(data))
