import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
df = pd.read_csv('benchmark_summary_20260215_161510.csv')

# Calculate mean of the 2 iterations for each map and planner
mean_data = df.groupby(['Map', 'Planner'])['Tasks_Finished'].mean().reset_index()

# Pivot to get default and implemented4 side by side
pivot_data = mean_data.pivot(index='Map', columns='Planner', values='Tasks_Finished')

# Calculate improvement percentage
pivot_data['Improvement'] = (pivot_data['implemented4'] - pivot_data['default']) / pivot_data['default'] * 100

# Sort by the order in the image (from top to bottom)
map_order = ['paris_1_256_250', 'brc202d_500', 'random_32_32_20_100', 'sortation_large_2000', 'warehouse_large_5000']
pivot_data = pivot_data.reindex(map_order)

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={'width_ratios': [3, 1]})

# Left plot: Task completed
y_pos = np.arange(len(map_order))
bar_height = 0.35

# Plot bars
bars1 = ax1.barh(y_pos - bar_height/2, pivot_data['default'], bar_height, 
                 label='Default', color='#E85D35')
bars2 = ax1.barh(y_pos + bar_height/2, pivot_data['implemented4'], bar_height, 
                 label='Implemented4', color='#1F5B8A')

# Add value labels on bars
for i, (bar1, bar2) in enumerate(zip(bars1, bars2)):
    width1 = bar1.get_width()
    width2 = bar2.get_width()
    ax1.text(width1/2, bar1.get_y() + bar1.get_height()/2, f'{int(width1)}',
             ha='center', va='center', color='white', fontweight='bold', fontsize=11)
    ax1.text(width2/2, bar2.get_y() + bar2.get_height()/2, f'{int(width2)}',
             ha='center', va='center', color='white', fontweight='bold', fontsize=11)

ax1.set_yticks(y_pos)
ax1.set_yticklabels(map_order)
ax1.set_xlabel('Task Completati', fontsize=12)
ax1.set_title('Task completati per istanza in 500 sec', fontsize=16, fontweight='bold')
ax1.legend(loc='lower right', fontsize=11)
ax1.grid(axis='x', alpha=0.3)

# Right plot: Improvement
bars3 = ax2.barh(y_pos, pivot_data['Improvement'], color='#1F5B8A')

# Add percentage labels on bars
for i, bar in enumerate(bars3):
    width = bar.get_width()
    ax2.text(width/2, bar.get_y() + bar.get_height()/2, f'{width:.2f}%',
             ha='center', va='center', color='white', fontweight='bold', fontsize=11)

ax2.set_yticks(y_pos)
ax2.set_yticklabels([])  # No labels on y-axis for the right plot
ax2.set_xlabel('')
ax2.set_title('Miglioramento', fontsize=16, fontweight='bold')
ax2.grid(axis='x', alpha=0.3)
ax2.set_xlim(0, 50)

# Adjust layout
plt.tight_layout()

# Save the figure
plt.savefig('benchmark_comparison_graph2.png', dpi=300, bbox_inches='tight')
print("Graph saved as benchmark_comparison_graph.png")

# Display the figure
plt.show()
