import glob, os, re, sys
mod, rom = sys.argv[1], sys.argv[2]
diff = 0
for path in sorted(glob.glob(os.path.join(mod, 'data', 'tiles', '*.xml'))):
    name = os.path.basename(path)
    vanilla = open(os.path.join(rom, 'data', 'tiles', name), encoding='utf-8').read()
    ours = open(path, encoding='utf-8').read()
    strip = lambda t: re.sub(r'<train_tracks>.*?</train_tracks>|<train_tracks/>', '', t, flags=re.S)
    ours2 = strip(ours)
    ours2 = re.sub(r'<mesh id="real_track_[^"]*" file_name="[^"]*">\s*<transform/>\s*</mesh>\s*', '', ours2)
    ours2 = re.sub(r'<physics_mesh id="real_track_phys_[^"]*" file_name="[^"]*">\s*<transform/>\s*</physics_mesh>\s*', '', ours2)
    ours2 = re.sub(r'file_name="meshes/real_track/ballast/([^"]+)"', lambda m: 'file_name="meshes/' + m.group(1).replace('__', '/') + '"', ours2)
    v2 = strip(vanilla)
    norm = lambda t: re.sub(r'\s+', '', t).replace('<meshes/>', '<meshes></meshes>').replace('<physics_meshes/>', '<physics_meshes></physics_meshes>')
    if norm(ours2) != norm(v2):
        diff += 1
        a, b = norm(ours2), norm(v2)
        i = next((k for k in range(min(len(a), len(b))) if a[k] != b[k]), min(len(a), len(b)))
        print('DIFF', name, 'at', i, '| ours:', a[max(0, i - 60):i + 80], '| vanilla:', b[max(0, i - 60):i + 80])
    # train graph: every vanilla node present with the same element attributes
    vt = dict(re.findall(r'<track id="([^"]+)"([^>]*)>', vanilla))
    ot = dict(re.findall(r'<track id="([^"]+)"([^>]*)>', ours))
    for k, attrs in vt.items():
        if ot.get(k) != attrs:
            diff += 1; print('TRACK', name, k, repr(attrs), repr(ot.get(k)))
print('tiles with differences outside the intended changes:', diff)
