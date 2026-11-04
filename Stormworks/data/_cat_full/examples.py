import json, math, sys
plan = json.load(open(sys.argv[1])); S = plan["supports"]; P = plan["portals"]
def show(tag, sp):
    print("%-12s track (%.1f, %.1f, %.1f) t (%.2f, %.2f) pole %s" % (tag, sp["track"][0], sp["track"][2], sp["track"][1], sp["t"][0], sp["t"][1],
          None if sp.get("pole") is None else "(%.1f, %.1f)" % (sp["pole"][0], sp["pole"][2])))
done = set()
for sp in S:
    if sp.get("insulating_mid") and "ins" not in done: show("insulating", sp); done.add("ins")
for sp in S:
    if sp.get("section_anchor") is None and sp.get("overlap_gap") is None and sp.get("overlap_of") is not None and sp.get("insulate") is False and len([1 for k in done if k.startswith("plain")]) < 3:
        show("plain", sp); done.add("plain%d" % len(done))
for i, p in enumerate(P):
    if p.get("kind") == "truss": print("truss       centre (%.1f, %.1f, %.1f)" % (p["centre"][0], p["centre"][2], p["rail_y"]))
n = 0
for sp in S:
    if sp.get("bracket") is not None and not sp.get("dropped") and n < 4 and sp.get("pole_owner") is None:
        show("bracket", sp); n += 1
m = 0
for span in plan["spans"]:
    if span.get("middle_anchor") and m < 2:
        show("middle", S[span["from"]]); m += 1
