"""Read/write Stormworks .mesh files and parse tile XML train tracks.

.mesh layout (little endian), as observed in game files:
    char[4]  "mesh"
    u16      7
    u16      1
    u16      vertex_count
    u16      0x13          (vertex format flags)
    u16      0
    vertex[vertex_count]   28 bytes each: float3 position, u8[4] rgba, float3 normal
    u32      index_count
    u16[index_count]       triangle list, front face winding: (b-a)x(c-a) points AGAINST the normal
    u16      submesh_count
    submesh[submesh_count]:
        u32 index_start, u32 index_count, u16 0, u16 shader,
        float3 bounds_min, float3 bounds_max,
        u16 0, u16 name_length, char[name_length] name,
        float3 (1, 1, 1)
    u16      0
"""

import re
import struct
from dataclasses import dataclass, field


@dataclass
class Submesh:
    index_start: int
    index_count: int
    shader: int
    bounds_min: tuple
    bounds_max: tuple
    name: bytes = b""
    unknown_u16: int = 0
    scale: tuple = (1.0, 1.0, 1.0)


@dataclass
class Mesh:
    # vertex: (x, y, z, r, g, b, a, nx, ny, nz)
    vertices: list = field(default_factory=list)
    indices: list = field(default_factory=list)
    submeshes: list = field(default_factory=list)
    trailer: bytes = b"\x00\x00"


def read_mesh_full(path):
    data = open(path, "rb").read()
    assert data[:4] == b"mesh", "not a mesh file"
    vertex_count = struct.unpack_from("<H", data, 8)[0]
    offset = 14
    mesh = Mesh()
    for _ in range(vertex_count):
        x, y, z = struct.unpack_from("<3f", data, offset)
        r, g, b, a = struct.unpack_from("<4B", data, offset + 12)
        nx, ny, nz = struct.unpack_from("<3f", data, offset + 16)
        mesh.vertices.append((x, y, z, r, g, b, a, nx, ny, nz))
        offset += 28
    index_count = struct.unpack_from("<I", data, offset)[0]
    offset += 4
    mesh.indices = list(struct.unpack_from("<%dH" % index_count, data, offset))
    offset += 2 * index_count
    submesh_count = struct.unpack_from("<H", data, offset)[0]
    offset += 2
    for _ in range(submesh_count):
        start, count, pad, shader = struct.unpack_from("<IIHH", data, offset)
        offset += 12
        bmin = struct.unpack_from("<3f", data, offset)
        bmax = struct.unpack_from("<3f", data, offset + 12)
        offset += 24
        unknown, name_len = struct.unpack_from("<HH", data, offset)
        offset += 4
        name = data[offset:offset + name_len]
        offset += name_len
        scale = struct.unpack_from("<3f", data, offset)
        offset += 12
        mesh.submeshes.append(Submesh(start, count, shader, bmin, bmax, name, unknown, scale))
    mesh.trailer = data[offset:]
    return mesh


def write_mesh(path, mesh):
    assert len(mesh.vertices) < 65536, "too many vertices for one mesh: %d" % len(mesh.vertices)
    out = bytearray()
    out += b"mesh" + struct.pack("<HHHHH", 7, 1, len(mesh.vertices), 0x13, 0)
    for (x, y, z, r, g, b, a, nx, ny, nz) in mesh.vertices:
        out += struct.pack("<3f4B3f", x, y, z, r, g, b, a, nx, ny, nz)
    out += struct.pack("<I", len(mesh.indices))
    out += struct.pack("<%dH" % len(mesh.indices), *mesh.indices)
    out += struct.pack("<H", len(mesh.submeshes))
    for sm in mesh.submeshes:
        out += struct.pack("<IIHH", sm.index_start, sm.index_count, 0, sm.shader)
        out += struct.pack("<3f", *sm.bounds_min) + struct.pack("<3f", *sm.bounds_max)
        out += struct.pack("<HH", sm.unknown_u16, len(sm.name)) + sm.name
        out += struct.pack("<3f", *sm.scale)
    out += mesh.trailer
    with open(path, "wb") as f:
        f.write(out)


PHYS_MAX_VERTICES_PER_PART = 65535 // 3 * 3


def write_phys(path, triangles):
    """Write a Stormworks .phys collision file (checked against 1996 game files):
        char[4] "phys", u16 2, u16 part_count,
        part: u16 vertex_count (triangle list, 3 vertices per triangle), float3[vertex_count], u16 0
    Winding as in .mesh: (b-a)x(c-a) points into the solid."""
    flat = [p for triangle in triangles for p in triangle]
    parts = [flat[i:i + PHYS_MAX_VERTICES_PER_PART]
             for i in range(0, len(flat), PHYS_MAX_VERTICES_PER_PART)]
    out = bytearray(b"phys" + struct.pack("<HH", 2, len(parts)))
    for part in parts:
        out += struct.pack("<H", len(part))
        for p in part:
            out += struct.pack("<3f", *p)
        out += struct.pack("<H", 0)
    with open(path, "wb") as f:
        f.write(out)


def read_phys(path):
    """Return a list of parts, each a list of (x, y, z) vertices (3 per triangle)."""
    data = open(path, "rb").read()
    assert data[:4] == b"phys", "not a phys file"
    _version, part_count = struct.unpack_from("<HH", data, 4)
    offset = 8
    parts = []
    for _ in range(part_count):
        n = struct.unpack_from("<H", data, offset)[0]
        values = struct.unpack_from("<%df" % (3 * n), data, offset + 2)
        parts.append([values[i:i + 3] for i in range(0, 3 * n, 3)])
        offset += 2 + 12 * n + 2
    return parts


# Backwards-compatible helper used by earlier analysis scripts.
def read_mesh(path):
    m = read_mesh_full(path)
    return m.vertices, m.indices, b""


def parse_transform(attr_text):
    """Return the 4x4 row-major matrix from <transform 00=".." 30=".."/> attributes."""
    values = dict(re.findall(r'(\d\d)="([-\d.eE+]+)"', attr_text))
    matrix = [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]
    for key, val in values.items():
        matrix[int(key[0])][int(key[1])] = float(val)
    return matrix


def read_tracks(path):
    """Return {node_id: ((x, y, z), [linked ids])} from a tile XML."""
    text = open(path, encoding="utf-8").read()
    nodes = {}
    # isolated nodes are written as <links/>
    pattern = r'<track id="([^"]+)">\s*<transform([^/]*)/>\s*(?:<links>(.*?)</links>|<links/>)'
    for m in re.finditer(pattern, text, re.S):
        matrix = parse_transform(m.group(2))
        pos = (matrix[3][0], matrix[3][1], matrix[3][2])
        links = re.findall(r'<link id="([^"]+)"', m.group(3) or "")
        nodes[m.group(1)] = (pos, links)
    return nodes
