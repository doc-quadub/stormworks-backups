import random, math, sys
random.seed(12345)
ins, outs = [], []
def add(op, a, b, res):
    ins.append("%s %r %r" % (op, a, b)); outs.append(res)
vals = []
for _ in range(60000):
    k = random.random()
    if k < 0.3: v = random.uniform(-1000, 1000)
    elif k < 0.5: v = random.uniform(-30000, 30000)
    elif k < 0.6: v = random.uniform(-1, 1) * 10 ** random.randint(-12, 12)
    elif k < 0.7: v = round(random.uniform(-600, 600), random.randint(0, 3))
    elif k < 0.8: v = float(random.randint(-2000, 2000)) / 2
    else: v = random.uniform(-10, 10)
    vals.append(v)
for i in range(0, len(vals) - 1, 2):
    a, b = vals[i], vals[i + 1]
    add("hypot", a, b, repr(math.hypot(a, b)))
    if b != 0:
        add("floordiv", a, b, repr(a // b)); add("mod", a, b, repr(a % b))
    add("round", a, 0, str(round(a)))
    add("round1", a, 1, repr(round(a, 1)))
    add("repr", a, 0, repr(a))
    add("repr", a * 1e-7, 0, repr(a * 1e-7))
    add("pow", abs(a) / 70.0, 2.0, repr((abs(a) / 70.0) ** 2.0))
    add("f6", a, 0, "%.6f" % a)
for a in (0.5, 1.5, 2.5, -0.5, -2.5, 0.25, 0.35, 2.675, 1e16, 1e15, 123456789012345678.0, 0.0001, 9.999e-05, 1e-4):
    add("round", a, 0, str(round(a))); add("round1", a, 2, repr(round(a, 2))); add("repr", a, 0, repr(a))
open("in.txt", "w").write("\n".join(ins) + "\n"); open("expected.txt", "w").write("\n".join(outs) + "\n")
print(len(ins))
