import glob, math, os, re, struct, sys
import numpy as np
mod = sys.argv[1]
problems = []
mesh_files = glob.glob(os.path.join(mod, 'meshes', '**', '*.mesh'), recursive=True)
phys_files = glob.glob(os.path.join(mod, 'meshes', '**', '*.phys'), recursive=True)
max_v = 0
for path in mesh_files:
    b = open(path, 'rb').read()
    if b[:4] != b'mesh':
        problems.append(('bad header', path)); continue
    vcount = struct.unpack_from('<H', b, 8)[0]
    max_v = max(max_v, vcount)
    off = 14
    v = np.frombuffer(b, dtype=np.dtype([('p', '<f4', 3), ('c', 'u1', 4), ('n', '<f4', 3)]), count=vcount, offset=off)
    off += 28 * vcount
    icount = struct.unpack_from('<I', b, off)[0]; off += 4
    idx = np.frombuffer(b, dtype='<u2', count=icount, offset=off); off += 2 * icount
    sub = struct.unpack_from('<H', b, off)[0]; off += 2
    if not np.isfinite(v['p']).all() or not np.isfinite(v['n']).all():
        problems.append(('non-finite vertex', path))
    if icount % 3 or (icount and idx.max() >= vcount):
        problems.append(('bad indices', path, icount, int(idx.max()) if icount else -1, vcount))
    for s in range(sub):
        start, count = struct.unpack_from('<II', b, off); off += 8
        off += 4
        bbox = struct.unpack_from('<6f', b, off); off += 24
        off += 2
        nlen = struct.unpack_from('<H', b, off)[0]; off += 2 + nlen + 12
        if start + count > icount or not all(math.isfinite(x) for x in bbox):
            problems.append(('bad submesh', path, start, count, icount))
    if off + 2 != len(b):
        problems.append(('trailing size', path, off + 2, len(b)))
    tri = idx.reshape(-1, 3)
    p = v['p'][tri]
    area = np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1)
    if (area < 1e-10).any():
        problems.append(('degenerate mesh triangles', path, int((area < 1e-10).sum())))
degenerate_phys = 0
max_part = 0
for path in phys_files:
    b = open(path, 'rb').read()
    if b[:4] != b'phys':
        problems.append(('bad phys header', path)); continue
    parts = struct.unpack_from('<H', b, 6)[0]
    off = 8
    for _ in range(parts):
        n = struct.unpack_from('<H', b, off)[0]; off += 2
        max_part = max(max_part, n)
        pts = np.frombuffer(b, dtype='<f4', count=3 * n, offset=off).reshape(-1, 3); off += 12 * n
        off += 2
        if n % 3 or not np.isfinite(pts).all():
            problems.append(('bad phys part', path, n))
        t = pts.reshape(-1, 3, 3)
        area = np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1)
        degenerate_phys += int((area < 1e-8).sum())
    if off != len(b):
        problems.append(('phys trailing size', path, off, len(b)))
# tile xmls: referenced files exist, track links consistent
missing = 0; bad_links = 0; tiles = 0
for path in glob.glob(os.path.join(mod, 'data', 'tiles', '*.xml')):
    tiles += 1
    text = open(path, encoding='utf-8').read()
    if text.count('<train_tracks') > 1:
        problems.append(('two train_tracks', path))
    for name in re.findall(r'file_name="(meshes/real_track/[^"]+)"', text):
        if not os.path.exists(os.path.join(mod, name)):
            missing += 1
    tracks = re.search(r'<train_tracks>(.*)</train_tracks>', text, re.S)
    if tracks:
        ids = set(re.findall(r'<track id="([^"]+)"', tracks.group(1)))
        for m in re.finditer(r'<track id="([^"]+)">(.*?)</track>', tracks.group(1), re.S):
            for l in re.findall(r'<link id="([^"]+)"', m.group(2)):
                if l not in ids:
                    bad_links += 1
                    problems.append(('link to unknown node', os.path.basename(path), m.group(1), l))
    for num in re.findall(r'\d\d="([^"]*)"', text):
        try:
            if not math.isfinite(float(num)): problems.append(('non-finite transform', path))
        except ValueError:
            problems.append(('bad number', path, num))
print('meshes', len(mesh_files), 'max vertices', max_v, 'phys', len(phys_files), 'max phys part', max_part,
      'degenerate phys triangles', degenerate_phys, 'tiles', tiles, 'missing refs', missing, 'links to unknown nodes', bad_links)
print('problems', len(problems))
ballast_degenerate = [p for p in problems if p[0] == 'degenerate mesh triangles' and '/ballast/' in p[1].replace(os.sep, '/')]
print('degenerate triangles in ballast copies (vanilla geometry, colour-only copies):', len(ballast_degenerate), sum(p[2] for p in ballast_degenerate))
for p in problems:
    if p not in ballast_degenerate: print('  ', p)
