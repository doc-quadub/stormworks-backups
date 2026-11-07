"""Plan view: python3 plot_region.py catenary.json diag.json x0,x1 z0,z1 out.png"""
import json, sys, math
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
C = json.load(open(sys.argv[1])); G = json.load(open(sys.argv[2]))
x0, x1 = map(float, sys.argv[3].split(",")); z0, z1 = map(float, sys.argv[4].split(","))
pos, links = G["pos"], G["links"]
inside = lambda p, m=40: x0 - m <= p[0] <= x1 + m and z0 - m <= p[2] <= z1 + m
fig, ax = plt.subplots(figsize=(18, 18 * (z1 - z0) / max(1, x1 - x0) + 1))
for k, ls in links.items():
    for o in ls:
        if k < o and o in pos and inside(pos[k]):
            ax.plot([pos[k][0], pos[o][0]], [pos[k][2], pos[o][2]], "-", color="0.75", lw=1)
S = C["supports"]; P = C["portals"]
colors = plt.cm.tab10.colors
def wire_xy(sp):
    r = (sp["t"][1], -sp["t"][0]); st = sp.get("stagger", 0.0)
    if sp.get("anchor"):
        tw = sp["anchor"]["towards"]; return (sp["pole"][0] + tw[0] * 4.8, sp["pole"][2] + tw[1] * 4.8)
    return (sp["track"][0] + r[0] * st, sp["track"][2] + r[1] * st)
for span in C["spans"]:
    a, b = S[span["from"]], S[span["to"]]
    if not inside(a["track"]): continue
    pa, pb = wire_xy(a), wire_xy(b)
    col = colors[span.get("run", 0) % 10]
    ax.plot([pa[0], pb[0]], [pa[1], pb[1]], "-", color=col, lw=4 if span.get("middle_anchor") else 1.6)
    if span.get("insulators"):
        m = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2); ax.plot(*m, "k*", ms=12)
for sp in S:
    if sp.get("dropped") or not inside(sp["track"]): continue
    if sp["portal"] is None:
        pole = sp["pole"]
        mk = "rs" if sp.get("anchor") else ("c^" if sp.get("bracket") is not None else "bo")
        if sp.get("pole_owner") is None: ax.plot(pole[0], pole[2], mk, ms=5)
        ax.plot([pole[0], wire_xy(sp)[0]], [pole[2], wire_xy(sp)[1]], "-", color="0.4", lw=0.6)
        if sp.get("anchor"):
            g = sp["anchor"]["guy_ground"]; ax.plot([pole[0], g[0]], [pole[2], g[2]], "r:", lw=1)
    if sp.get("overlap_of") is not None and not sp.get("anchor"):
        w = wire_xy(sp); ax.plot(w[0], w[1], "o", mfc="none", mec="k", ms=7)
for p in P:
    if inside(p["centre"]):
        a, b = p["poles"]; ax.plot([a[0], b[0]], [a[2], b[2]], "-", color="brown" if p.get("kind") == "truss" else "r", lw=3)
ax.set_xlim(x0, x1); ax.set_ylim(z0, z1); ax.set_aspect("equal"); ax.grid(True)
plt.savefig(sys.argv[5], dpi=60, bbox_inches="tight"); print(sys.argv[5])
