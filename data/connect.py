import matplotlib.pyplot as plt
import numpy as np

hops   = [60, 80, 100, 120]
time_s = [260.663, 70.605, 96.170, 99.266]
wps    = [547, 428, 920, 993]
nodes  = [15684, 4462, 5767, 5950]

x = np.arange(len(hops))
w = 0.25

fig = plt.figure(figsize=(8, 4.5))

b1 = plt.bar(x - w, time_s,  width=w, label="Runtime (s)")
b2 = plt.bar(x,     wps,     width=w, label="Waypoints")
b3 = plt.bar(x + w, nodes,   width=w, label="Nodes")

def annotate(bars):
    for bar in bars:
        h = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., h,
                 f"{h:.3g}", ha='center', va='bottom', fontsize=8)

annotate(b1); annotate(b2); annotate(b3)

plt.xticks(x, [str(h) for h in hops])
plt.xlabel("CONNECT_HOPS")
plt.title("KD-RRT Performance vs CONNECT_HOPS (goal bias = 0.12)")
plt.legend()
plt.tight_layout()
plt.savefig("connect_hops.png", dpi=200)
print("Saved connect_hops.png")
