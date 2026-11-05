import json, sys, os
sys.path.insert(0, '..'); sys.path.insert(0, '.')
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from map_rails import graph, origin
rom = sys.argv[1]; data = json.load(open(sys.argv[2])); scan = json.load(open(sys.argv[3]))
jobs = [a.split('=') for a in sys.argv[4:]]          # name=x0,z0,x1,z1 or name=all
grey = []
for t in scan['tiles']:
    o = origin(t)
    if not o: continue
    pos, adj = graph(os.path.join(rom, 'data', 'tiles', t + '.xml'))
    for a in adj:
        for b in adj[a]:
            if a < b: grey.append(((pos[a][0]+o[0], pos[a][1]+o[1]), (pos[b][0]+o[0], pos[b][1]+o[1])))
red = [((p[0]/10, p[1]/10), (q[0]/10, q[1]/10)) for l in data['world'] for p, q in zip(l, l[1:])]
for name, box in jobs:
    fig, ax = plt.subplots(figsize=(12, 12), dpi=100)
    ax.add_collection(LineCollection(grey, colors='#888888', linewidths=0.6))
    ax.add_collection(LineCollection(red, colors='#e01010', linewidths=2.0 if box != 'all' else 1.0, alpha=0.55))
    ax.set_aspect('equal'); ax.autoscale()
    if box != 'all':
        b = [float(v) for v in box.split(',')]; ax.set_xlim(b[0], b[2]); ax.set_ylim(b[1], b[3])
    plt.savefig('prev/%s.png' % name, bbox_inches='tight'); plt.close(fig)
print('ok', len(grey), len(red))
