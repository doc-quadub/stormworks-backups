"""Tiny z-buffer rasterizer for previewing quads/triangles (numpy)."""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def render(quads, eye, target, path, fov=55, width=1400, height=800, title="",
           light=(0.4, 0.8, 0.3), sky=(0.70, 0.80, 0.90)):
    eye = np.array(eye, float); target = np.array(target, float)
    f = target - eye; f /= np.linalg.norm(f)
    # Game world: x east, y up, z north (left-handed). Facing north, east is on the right.
    r = np.cross([0, 1, 0], f); r /= np.linalg.norm(r)
    u = np.cross(f, r)
    focal = (height / 2) / math.tan(math.radians(fov) / 2)
    L = np.array(light) / np.linalg.norm(light)
    img = np.empty((height, width, 3)); img[:] = sky
    zbuf = np.full((height, width), np.inf)
    tris = []
    for q in quads:
        p = q[:4]
        tris.append((p[0], p[1], p[2], q[4], q[5]))
        if p[3] != p[2]:
            tris.append((p[0], p[2], p[3], q[4], q[5]))
    for a, b, c, n, col in tris:
        P = np.array([a, b, c], float)
        n = np.array(n, float)
        if np.dot(n, eye - P[0]) <= 0:
            continue
        rel = P - eye
        z = rel @ f
        if np.any(z < 0.2):
            continue
        sx = width / 2 + focal * (rel @ r) / z
        sy = height / 2 - focal * (rel @ u) / z
        x0, x1 = int(max(0, math.floor(sx.min()))), int(min(width - 1, math.ceil(sx.max())))
        y0, y1 = int(max(0, math.floor(sy.min()))), int(min(height - 1, math.ceil(sy.max())))
        if x0 > x1 or y0 > y1:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        d = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0])
        if abs(d) < 1e-9:
            continue
        w1 = ((xs - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (ys - sy[0])) / d
        w2 = ((sx[1] - sx[0]) * (ys - sy[0]) - (xs - sx[0]) * (sy[1] - sy[0])) / d
        w0 = 1 - w1 - w2
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        invz = w0 / z[0] + w1 / z[1] + w2 / z[2]
        depth = 1 / invz
        zb = zbuf[y0:y1 + 1, x0:x1 + 1]
        m = inside & (depth < zb)
        k = 0.4 + 0.6 * max(0.0, float(n @ L))
        zb[m] = depth[m]
        img[y0:y1 + 1, x0:x1 + 1][m] = np.array(col[:3]) / 255 * k
    fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1]); ax.imshow(img); ax.axis("off")
    if title:
        ax.text(10, 25, title, fontsize=12, color="black")
    fig.savefig(path); plt.close(fig)
