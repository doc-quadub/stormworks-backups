"""High resolution plan pictures of the overhead line.
python3 plan_pictures.py <rom> <work with scan.json, catenary.json> <out_dir>"""
import json, math, os, sys
sys.path.insert(0, ".")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import build_world, signs, catenary

rom, work, out = sys.argv[1:4]
os.makedirs(out, exist_ok=True)
scan = json.load(open(os.path.join(work, "scan.json")))
plan = json.load(open(os.path.join(work, "catenary.json")))
net = signs.build_network(scan["tiles"], lambda t: build_world.prepare_graph(rom, t, scan))
S, P = plan["supports"], plan["portals"]

track_segments = []
for k, ls in net.links.items():
    for o in ls:
        if k < o and o in net.nodes_world:
            a, b = net.nodes_world[k], net.nodes_world[o]
            track_segments.append(((a[0], a[2]), (b[0], b[2])))           # game Y = z, north up

PALETTE = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#8c564b", "#e377c2",
           "#bcbd22", "#7f7f7f", "#393b79", "#ad494a", "#637939", "#7b4173", "#e6550d", "#3182bd"]


def wire_point(sp):
    if sp.get("anchor"):
        tw = sp["anchor"]["towards"]
        return (sp["pole"][0] + tw[0] * catenary.CONTACT_END_OUT, sp["pole"][2] + tw[1] * catenary.CONTACT_END_OUT)
    r = (sp["t"][1], -sp["t"][0])
    st = sp.get("stagger", 0.0)
    return (sp["track"][0] + r[0] * st, sp["track"][2] + r[1] * st)


def draw(name, box=None, width_px=6000, detail=False, title=None):
    if box is None:
        xs = [p[0] for s in track_segments for p in s]; ys = [p[1] for s in track_segments for p in s]
        box = (min(xs) - 200, max(xs) + 200, min(ys) - 200, max(ys) + 200)
    x0, x1, y0, y1 = box
    aspect = (y1 - y0) / (x1 - x0)
    dpi = 200
    fig = plt.figure(figsize=(width_px / dpi, width_px * aspect / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor("white")
    scale = (x1 - x0) / width_px                   # metres per pixel
    track_w = max(0.6, min(4.0, 3.0 / scale)) if detail else 2.6
    ax.add_collection(LineCollection(track_segments, colors="#b9b9b9", linewidths=track_w * 1.8, zorder=1))
    ax.add_collection(LineCollection(track_segments, colors="#6e6e6e", linewidths=track_w * 0.5, zorder=2))
    wires = {}
    for span in plan["spans"]:
        a, b = S[span["from"]], S[span["to"]]
        wires.setdefault(span.get("run", 0), []).append((wire_point(a), wire_point(b)))
    for run, segs in wires.items():
        ax.add_collection(LineCollection(segs, colors=PALETTE[run % len(PALETTE)],
                                         linewidths=(1.8 if detail else 2.4), zorder=3))
    if detail:
        cant = []
        for sp in S:
            if sp.get("dropped") or sp["portal"] is not None or sp.get("pole") is None:
                continue
            cant.append(((sp["pole"][0], sp["pole"][2]), wire_point(sp)))
        ax.add_collection(LineCollection(cant, colors="#444444", linewidths=0.8, zorder=4))
        for sp in S:
            if sp.get("dropped") or sp["portal"] is not None or sp.get("pole") is None or sp.get("pole_owner") is not None:
                continue
            anchor = sp.get("anchor") is not None
            ax.plot(sp["pole"][0], sp["pole"][2], "s" if anchor else "o", ms=5 if anchor else 3.2,
                    color="#000000" if anchor else "#555555", zorder=5)
            if anchor:
                g = sp["anchor"]["guy_ground"]
                ax.plot([sp["pole"][0], g[0]], [sp["pole"][2], g[2]], ":", color="#000000", lw=0.8, zorder=4)
    else:
        for sp in S:
            if sp.get("anchor") and not sp.get("dropped"):
                ax.plot(sp["pole"][0], sp["pole"][2], "o", ms=2.4, color="black", zorder=5)
    for portal in P:
        if portal.get("unused"):
            continue
        a, b = portal["poles"]
        truss = portal.get("kind") == "truss"
        ax.plot([a[0], b[0]], [a[2], b[2]], "-", color="#8b5a2b" if truss else "#7a0000",
                lw=(3.0 if detail else 1.5), solid_capstyle="butt", zorder=6)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal"); ax.axis("off")
    if title:
        ax.text(0.012, 0.985, title, transform=ax.transAxes, va="top", ha="left", fontsize=14 if detail else 22,
                color="#222222", family="DejaVu Sans", zorder=10,
                bbox=dict(facecolor="white", edgecolor="#cccccc", boxstyle="round,pad=0.4"))
    fig.savefig(os.path.join(out, name + ".png"), dpi=dpi, facecolor="white")
    plt.close(fig)
    print(name, box)


draw("sawyer_overview", width_px=7000, title="SRO Catenary+ (Sawyer): anchor sections of the overhead line")

# stations and junctions: groups of turnouts, the busiest first
turnouts = [net.nodes_world[k] for k, ls in net.links.items() if len(ls) >= 3]
groups = []
for p in turnouts:
    for g in groups:
        if math.hypot(g["x"] - p[0], g["z"] - p[2]) < 250:
            g["pts"].append(p); g["x"] = sum(q[0] for q in g["pts"]) / len(g["pts"]); g["z"] = sum(q[2] for q in g["pts"]) / len(g["pts"]); break
    else:
        groups.append({"x": p[0], "z": p[2], "pts": [p]})
groups.sort(key=lambda g: -len(g["pts"]))
def fitted_box(cx, cz, reach, pad=30.0, min_aspect=0.42, max_aspect=1.0):
    """Box round the tracks within `reach` of (cx, cz), padded, with a sensible aspect."""
    pts = [p for seg in track_segments for p in seg if math.hypot(p[0] - cx, p[1] - cz) < reach]
    x0, x1 = min(p[0] for p in pts) - pad, max(p[0] for p in pts) + pad
    y0, y1 = min(p[1] for p in pts) - pad, max(p[1] for p in pts) + pad
    w, h = x1 - x0, y1 - y0
    if h / w < min_aspect:
        grow = (w * min_aspect - h) / 2; y0 -= grow; y1 += grow
    elif h / w > max_aspect:
        grow = (h / max_aspect - w) / 2; x0 -= grow; x1 += grow
    y1 += (y1 - y0) * 0.06                        # room for the title
    return (x0, x1, y0, y1)


for i, g in enumerate(groups[:6]):
    xs = [q[0] for q in g["pts"]]; zs = [q[2] for q in g["pts"]]
    cx, cz = (min(xs) + max(xs)) / 2, (min(zs) + max(zs)) / 2
    reach = max(220.0, max(max(xs) - min(xs), max(zs) - min(zs)) / 2 + 140)
    draw("station_%d" % (i + 1), fitted_box(cx, cz, reach), width_px=5000, detail=True,
         title="X %.0f  Y %.0f" % (cx, cz))
draw("north_harbour", fitted_box(1480, -3766, 330), width_px=6000, detail=True, title="North Harbour  X 1480  Y -3766")
draw("truss_bridge", fitted_box(3880, -9150, 230), width_px=4000, detail=True, title="Truss bridge  X 3880  Y -9150")
