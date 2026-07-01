import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import csv
import math
import os
import json
from ament_index_python.packages import get_package_share_directory

class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trajectory_player')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pkg_dir = get_package_share_directory('aircraft_design')

        # --- 1. PHYSICAL DIMENSIONS ---
        # Distance from aircraft center (base_link) to the nose gear hitch
        self.center_to_nose = 16.5 
        # Length of the towbar (distance from tug center to nose gear)
        self.towbar_length = 5.0

        # --- 2. LOAD CSV ---
        csv_path = os.path.join(self.pkg_dir, 'data', 'example_trajectory.csv')
        self.trajectory_data = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.trajectory_data.append(row)

        self.current_idx = 0
        self.timer = self.create_timer(0.1, self.playback_callback)

    def playback_callback(self):
        if self.current_idx >= len(self.trajectory_data):
            return

        row = self.trajectory_data[self.current_idx]
        now = self.get_clock().now().to_msg()
        
        # Aircraft State from CSV
        ax, ay, ayaw = float(row['ac_x']), float(row['ac_y']), float(row['ac_yaw'])
        # Tug Heading from CSV (or match aircraft if pushing straight)
        tyaw = float(row.get('tug_yaw', ayaw))

        # --- 3. HITCH LOGIC ---
        # Find the Nose Gear Position
        nx = ax + (self.center_to_nose * math.cos(ayaw))
        ny = ay + (self.center_to_nose * math.sin(ayaw))

        # Position the Tug at the end of the towbar
        # (Assuming tug is PUSHING, it sits 'towbar_length' away from the nose)
        tx = nx + (self.towbar_length * math.cos(tyaw))
        ty = ny + (self.towbar_length * math.sin(tyaw))

        # --- 4. BROADCAST ---
        # Aircraft Model
        self.broadcast_tf('world', 'aircraft/base_link', ax, ay, ayaw, now)
        # Tug Model (Connected to the hitch point)
        self.broadcast_tf('world', 'tug/base_link', tx, ty, tyaw, now)

        self.current_idx += 1

    def broadcast_tf(self, parent, child, x, y, yaw, stamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    rclpy.spin(node)
    rclpy.shutdown()