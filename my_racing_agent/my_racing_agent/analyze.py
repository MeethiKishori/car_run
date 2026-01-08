import pandas as pd
import matplotlib.pyplot as plt
import os

# ==========================
# 1. Load CSV
# ==========================

# Path relative to this script
script_dir = os.path.dirname(os.path.realpath(__file__))
csv_file = os.path.join(script_dir, 'pid.csv') #change .csv accordingly

# Load CSV
df = pd.read_csv(csv_file)

# Ensure timestamp is float

df['timestamp'] = df['timestamp'].astype(float)
time_sec = df['timestamp'] - df['timestamp'].iloc[0]  # relative time

# ==========================
# 2. Plot Cross-Track Error (CTE)
# ==========================
plt.figure(figsize=(10,4))
plt.plot(time_sec, df['cte'], label='CTE', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Cross-Track Error (m)')
plt.title('CTE over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ==========================
# 3. Plot Linear and Angular Speed
# ==========================
plt.figure(figsize=(10,4))
plt.plot(time_sec, df['linear_speed'], label='Linear Speed (m/s)', color='green')
plt.plot(time_sec, df['angular_speed'], label='Angular Speed (rad/s)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Speed')
plt.title('Linear & Angular Speeds over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

"""
# ==========================
# 4. Plot Robot Trajectory
# ==========================
plt.figure(figsize=(6,6))
plt.plot(df['x'], df['y'], label='Trajectory', color='purple')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ==========================
# 5. Highlight Safety Events on CTE
# ==========================
plt.figure(figsize=(10,4))
plt.plot(time_sec, df['cte'], label='CTE', color='blue')
danger_times = time_sec[df['safety_flag'] != 0]
for t in danger_times:
    plt.axvline(x=t, color='red', linestyle='--', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Cross-Track Error (m)')
plt.title('CTE with Safety Events')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ==========================
# 6. Summary Metrics
# ==========================
mean_cte = df['cte'].abs().mean()
max_cte = df['cte'].abs().max()
num_safety = df[df['safety_flag'] != 0].shape[0]

print("=== Performance Summary ===")
print(f"Mean CTE: {mean_cte:.3f} m")
print(f"Max CTE: {max_cte:.3f} m")
print(f"Number of Safety Events: {num_safety}")

"""