import csv
import math
import os

# --- PHYSICAL PARAMETERS (From JSON) ---
L_tug = 1.886       # Tug wheelbase (m)
c_tug = -0.1        # Hitch offset behind tug rear axle (m)
L_ac = 8.781        # Aircraft wheelbase (m)

# --- SIMULATION SETTINGS ---
dt = 0.1            # Time step (10 Hz)
total_time = 50.0   # Increased time so we can see the full right turn finish
v_tug = 2.0         # Slightly faster tug velocity (m/s)

# --- INITIAL STATES (Matching your plot) ---
# To match your graph, the plane starts pointing UP (North, +Y direction)
# which is a yaw of 90 degrees (pi / 2 radians).
initial_yaw = math.pi / 2.0

# Aircraft starts at origin pointing North
ac_x, ac_y, ac_yaw = 0.0, 0.0, initial_yaw

# Tug is positioned exactly in front of the aircraft along the Y axis
tug_x, tug_y, tug_yaw = 0.0, 8.881, initial_yaw

script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, 'example_trajectory.csv')

print("Calculating 90-degree right turn trajectory...")

with open(csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'tug_x', 'tug_y', 'tug_yaw', 'ac_x', 'ac_y', 'ac_yaw'])

    for step in range(int(total_time / dt)):
        t = step * dt

        # --- 1. DEFINE THE 90-DEGREE RIGHT TURN MANEUVER ---
        if t < 5.0:
            # Drive straight North for the first 5 seconds
            steer_angle = 0.0
        elif tug_yaw > 0.0:
            # Steer right until the tug is pointing exactly East (0 radians)
            steer_angle = -0.45
        else:
            # Once it hits 0 radians, lock the steering straight to drive East
            steer_angle = 0.0
            tug_yaw = 0.0 # Clamp to prevent overshooting

        # --- 2. TUG KINEMATICS (Bicycle Model) ---
        tug_x += v_tug * math.cos(tug_yaw) * dt
        tug_y += v_tug * math.sin(tug_yaw) * dt
        
        # Only calculate yaw change if we are actually steering
        if steer_angle != 0.0:
            tug_yaw += (v_tug / L_tug) * math.tan(steer_angle) * dt

        # --- 3. AIRCRAFT KINEMATICS (Trailer Following) ---
        hitch_x = tug_x + c_tug * math.cos(tug_yaw)
        hitch_y = tug_y + c_tug * math.sin(tug_yaw)

        ac_yaw = math.atan2(hitch_y - ac_y, hitch_x - ac_x)
        ac_x = hitch_x - L_ac * math.cos(ac_yaw)
        ac_y = hitch_y - L_ac * math.sin(ac_yaw)

        writer.writerow([
            round(t, 2),
            round(tug_x, 4), round(tug_y, 4), round(tug_yaw, 4),
            round(ac_x, 4), round(ac_y, 4), round(ac_yaw, 4)
        ])

print(f"Success! Trajectory saved to: {csv_path}")