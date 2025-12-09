import matplotlib.pyplot as plt
import numpy as np

bias_vals   = [0.05, 0.12, 0.2, 0.3]
times       = [196.806, 99.266, 427.143, 173.845]   
waypoints   = [736, 993, 657, 521]
nodes       = [11820, 5950, 29121, 10424]

x = np.arange(len(bias_vals))
width = 0.25

fig, ax = plt.subplots(figsize=(8, 5))

bars1 = ax.bar(x - width, times, width, label='Execution Time (s)', color='#1f77b4')
bars2 = ax.bar(x, waypoints, width, label='Waypoints', color='#ff7f0e')
bars3 = ax.bar(x + width, nodes, width, label='Nodes', color='#2ca02c')

ax.set_xlabel('Goal Bias', fontsize=12)
ax.set_ylabel('Metric Value', fontsize=12)
ax.set_title('Effect of Goal Bias on KD-RRT Performance', fontsize=14)
ax.set_xticks(x)
ax.set_xticklabels([str(b) for b in bias_vals])
ax.legend()
ax.grid(alpha=0.3, linestyle='--', axis='y')

def autolabel(bars, fmt='{:.0f}', offset=2):
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, height + offset,
                fmt.format(height), ha='center', va='bottom', fontsize=9)

autolabel(bars1, fmt='{:.1f}', offset=5)  
autolabel(bars2, fmt='{:.0f}', offset=20)  
autolabel(bars3, fmt='{:.0f}', offset=400)

plt.tight_layout()
plt.savefig('goal_bias.png', dpi=300)
plt.show()
