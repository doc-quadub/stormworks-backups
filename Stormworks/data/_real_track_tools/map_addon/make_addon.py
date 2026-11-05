"""Build the Rail Map addon from rails.json (map_rails.py output).
python3 make_addon.py <rails.json> <out_dir>"""
import hashlib, json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
src, out = sys.argv[1:3]
data = json.load(open(src))


def enc(line):
    return '"' + ",".join("%d,%d" % (x, z) for x, z in line) + '"'


def lua_list(lines, indent):
    pad = " " * indent
    return "{\n" + "".join(pad + "    " + enc(l) + ",\n" for l in lines) + pad + "}"


mega = [l for l in data["world"] if l[0][1] > -180000]       # Sawyer lies north of z = -18 km
arid = [l for l in data["world"] if l[0][1] <= -180000]
world = "{\n    mega = " + lua_list(mega, 4) + ",\n    arid = " + lua_list(arid, 4) + ",\n}"
tiles = "{\n" + "".join('    ["%s"] = %s,\n' % (t, lua_list(ls, 4)) for t, ls in sorted(data["tiles"].items()) if ls) + "}"
version = hashlib.md5(json.dumps(data, sort_keys=True).encode()).hexdigest()[:12]
lua = open(os.path.join(HERE, "railmap_template.lua")).read()
lua = lua.replace("@@DATA_VERSION@@", version).replace("@@WORLD@@", world).replace("@@TILES@@", tiles)
assert "@@" not in lua
os.makedirs(out, exist_ok=True)
open(os.path.join(out, "script.lua"), "w", newline="\n").write(lua)
open(os.path.join(out, "playlist.xml"), "w", newline="\n").write(
    '<?xml version="1.0" encoding="UTF-8"?>\n<playlist file_store="4" name="Rail Map">\n'
    '\t<locations>\n\t\t<locations/>\n\t</locations>\n</playlist>\n\n')
print("script.lua", len(lua), "bytes; world lines", len(mega), "+", len(arid), "; tiles", len(data["tiles"]), "; version", version)
