import matplotlib.pyplot as plt
import numpy as np

primitive_counts = [6, 12, 18]

execution_time = [31.55, 99.266, 254.512]  
waypoints      = [566, 993, 577]
nodes          = [1883, 5950, 15256]

x = np.arange(len(primitive_counts))  
width = 0.25  

fig, ax = plt.subplots(figsize=(8, 5))

b1 = ax.bar(x - width, execution_time, width, label='Execution Time (s)')
b2 = ax.bar(x, waypoints, width, label='Waypoints')
b3 = ax.bar(x + width, nodes, width, label='Nodes')

ax.set_xlabel('Number of Motion Primitives (k)', fontsize=12)
ax.set_ylabel('Value', fontsize=12)
ax.set_title('Effect of Motion Primitive Count on Planning Performance', fontsize=14, weight='bold')
ax.set_xticks(x)
ax.set_xticklabels(primitive_counts)
ax.legend()
ax.grid(axis='y', linestyle='--', alpha=0.6)

def autolabel(bars):
    for bar in bars:
        height = bar.get_height()
        ax.annotate(f'{height:.0f}',
                    xy=(bar.get_x() + bar.get_width()/2, height),
                    xytext=(0, 3),  
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=9)

autolabel(b1)
autolabel(b2)
autolabel(b3)

plt.tight_layout()
plt.show()
