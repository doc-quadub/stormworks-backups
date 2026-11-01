"""Checks on a finished build (run after the tiles step).

  python3 verify.py <rom_dir> <work_dir> [first:last tile index, to split long runs]

* sleepers: every rail point (sampled every 0.6 m) has a sleeper under it within 0.45 m
* switches: every vanilla junction keeps exactly its vanilla links (the game reads the route
  choice from them)
* close nodes: no inserted node lies within smooth.MIN_NODE_GAP of a node it is not linked to
  (the game connects nearby nodes, trains would jump between tracks)
"""

import glob
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rails  # noqa: E402
import smooth  # noqa: E402
from swmesh import read_mesh_full, read_tracks  # noqa: E402


def load_state(work, tile):
    with open(os.path.join(work, "tiles", tile + ".json")) as f:
        return json.load(f)


def check_sleepers(state):
    graph = state["graph"]
    nodes = {k: ((v[0], v[1], v[2]), v[3]) for k, v in graph.items()}
    chains = rails.build_chains(nodes)
    points = [rails.chain_points(c, nodes, state["node_y"]) for c, _ in chains]
    sleepers = rails.place_sleepers(points)
    grid = {}
    for s in sleepers:
        grid.setdefault((int(s.center[0] // 4), int(s.center[2] // 4)), []).append(s)
    total = missing = 0
    where = []
    for pts in points:
        for pos, t, _, _ in rails.sample_chain(pts, 0.6, 0.3):
            u = rails.left_of(t)
            for side in (-1, 1):
                r = rails.add(pos, rails.mul(u, side * rails.RAIL_HALF_GAUGE))
                total += 1
                supported = False
                for ix in range(int(r[0] // 4) - 2, int(r[0] // 4) + 3):
                    for iz in range(int(r[2] // 4) - 2, int(r[2] // 4) + 3):
                        for s in grid.get((ix, iz), ()):
                            d = rails.sub(r, s.center)
                            if s.span[0] <= rails.dot(d, s.axis_u) <= s.span[1] and \
                                    abs(rails.dot(d, s.axis_t)) <= 0.45:
                                supported = True
                                break
                        if supported:
                            break
                    if supported:
                        break
                if not supported:
                    missing += 1
                    where.append((round(r[0], 1), round(r[2], 1)))
    return total, missing, where


def check_switches(rom_dir, tile, state):
    vanilla = read_tracks(os.path.join(rom_dir, "data", "tiles", tile + ".xml"))
    changed = []
    count = 0
    for k, (p, links) in vanilla.items():
        if not smooth.is_junction(vanilla, k):
            continue
        count += 1
        if set(state["graph"][k][3]) != set(o for o in links if o in vanilla):
            changed.append(k)
    return count, changed


def check_close_nodes(state):
    graph = state["graph"]
    cell = smooth.MIN_NODE_GAP
    grid = {}
    for k, v in graph.items():
        grid.setdefault((int(v[0] // cell), int(v[2] // cell)), []).append(k)
    close = []
    for k, v in graph.items():
        if not k.startswith(smooth.NEW_NODE_PREFIX):
            continue
        cx, cz = int(v[0] // cell), int(v[2] // cell)
        for ix in (cx - 1, cx, cx + 1):
            for iz in (cz - 1, cz, cz + 1):
                for o in grid.get((ix, iz), ()):
                    w = graph[o]
                    if o == k or o in v[3] or k in w[3]:
                        continue
                    if math.hypot(v[0] - w[0], v[2] - w[2]) < cell - 0.01:
                        close.append((k, o))
    return close


def main(rom_dir, work, part="0:9999"):
    tiles = sorted(os.path.basename(f)[:-5] for f in glob.glob(os.path.join(work, "tiles", "*.json")))
    first, last = (int(x) for x in part.split(":"))
    tiles = tiles[first:last]
    sleeper_total = sleeper_missing = 0
    switch_total = 0
    problems = []
    for tile in tiles:
        state = load_state(work, tile)
        total, missing, where = check_sleepers(state)
        sleeper_total += total
        sleeper_missing += missing
        if missing:
            problems.append(("sleepers", tile, missing, where[:3]))
        close = check_close_nodes(state)
        if close:
            problems.append(("new node close to an unlinked node", tile, close[:3]))
        count, changed = check_switches(rom_dir, tile, state)
        switch_total += count
        if changed:
            problems.append(("switch links changed", tile, changed[:5]))
    print("rail samples %d, without sleeper %d; switches %d" % (sleeper_total, sleeper_missing, switch_total))
    for p in problems[:40]:
        print(" ", p)
    print("done")


if __name__ == "__main__":
    main(*sys.argv[1:4])
