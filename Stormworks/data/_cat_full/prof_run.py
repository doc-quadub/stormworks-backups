"""Run diag_full.py under cProfile; print the top functions after the run or at LIMIT seconds."""
import cProfile, os, pstats, sys, threading
LIMIT = float(os.environ.get("LIMIT", "150"))
prof = cProfile.Profile()

def dump():
    prof.disable()
    pstats.Stats(prof).sort_stats("cumulative").print_stats(25)
    sys.stdout.flush()
    os._exit(0)

threading.Timer(LIMIT, dump).start()
sys.argv = ["diag_full.py"] + sys.argv[1:]
prof.enable()
exec(compile(open("diag_full.py").read(), "diag_full.py", "exec"))
dump()
