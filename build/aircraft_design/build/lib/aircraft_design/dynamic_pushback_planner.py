# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import math
# import numpy as np
# import csv
# import os

# from .physics_engine.load_parameters import load_parameters

# def normalize_angle(angle):
#     while angle > math.pi: angle -= 2.0 * math.pi
#     while angle < -math.pi: angle += 2.0 * math.pi
#     return angle

# def quaternion_from_euler(yaw):
#     # Returns (x, y, z, w) for a given yaw
#     return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

# class DynamicPushbackPlanner(Node):
#     def __init__(self):
#         super().__init__('dynamic_pushback_planner')
        
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         # --- PATH TO YOUR OLD SIMULATION FILE ---
#         # Ensure the filename matches exactly!
#         script_dir = os.path.dirname(os.path.abspath(__file__))
#         self.csv_path = os.path.join(script_dir, '..', 'data', 'kinematic_Right Turn_MAC_16.5_Nose_Heavy.csv')
        
#         self.recorded_data = []
#         try:
#             with open(self.csv_path, 'r') as f:
#                 # Note: Your CSV uses 'Time', 'X', 'Y', 'Yaw', 'Hitch_Angle'
#                 reader = csv.DictReader(f)
#                 for row in reader:
#                     self.recorded_data.append(row)
#             self.get_logger().info(f"SUCCESS: Loaded {len(self.recorded_data)} moments from CSV.")
#         except Exception as e:
#             self.get_logger().error(f"Failed to load CSV: {e}")
#             return

#         # Load Physics (needed for hitch 'c' and aircraft 'L' dimensions)
#         self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        
#         # Match the CSV time step (0.04s)
#         self.timer_period = 0.04 
#         self.current_index = 0
        
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)
#         self.get_logger().info("Simulation Playback Started. Watch RViz for the Right Turn.")

#     def timer_callback(self):
#         if self.current_index >= len(self.recorded_data):
#             self.get_logger().info("Playback Complete.")
#             self.timer.cancel()
#             return

#         # 1. Extract recorded Tug state
#         row = self.recorded_data[self.current_index]
#         tx = float(row['X'])
#         ty = float(row['Y'])
#         tyaw = float(row['Yaw'])
#         hitch = float(row['Hitch_Angle'])

#         # 2. Calculate Aircraft Position using hitch kinematics
#         # ac_yaw = tug_yaw + hitch_angle
#         ayaw = normalize_angle(tyaw + hitch)
#         # Hitch joint position
#         hx = tx + self.p.vehicle.c * math.cos(tyaw)
#         hy = ty + self.p.vehicle.c * math.sin(tyaw)
#         # Aircraft center (MLG) position
#         ax = hx - self.p.aircraft.L * math.cos(ayaw)
#         ay = hy - self.p.aircraft.L * math.sin(ayaw)

#         # 3. Publish TFs to RViz
#         now = self.get_clock().now().to_msg()
#         self.publish_tf('tug/base_link', tx, ty, tyaw, now)
#         self.publish_tf('aircraft/base_link', ax, ay, ayaw, now)

#         self.current_index += 1

#     def publish_tf(self, frame_id, x, y, yaw, timestamp):
#         t = TransformStamped()
#         t.header.stamp = timestamp
#         t.header.frame_id = 'world'
#         t.child_frame_id = frame_id
#         t.transform.translation.x = float(x)
#         t.transform.translation.y = float(y)
#         t.transform.translation.z = 0.0
        
#         qx, qy, qz, qw = quaternion_from_euler(yaw)
#         t.transform.rotation.x = qx
#         t.transform.rotation.y = qy
#         t.transform.rotation.z = qz
#         t.transform.rotation.w = qw
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamicPushbackPlanner()
#     rclpy.spin(node)
#     rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped, PoseStamped
# from nav_msgs.msg import Path
# from tf2_ros import TransformBroadcaster
# import math
# import numpy as np
# import csv
# import os

# from .physics_engine.load_parameters import load_parameters

# def normalize_angle(angle):
#     while angle > math.pi: angle -= 2.0 * math.pi
#     while angle < -math.pi: angle += 2.0 * math.pi
#     return angle

# def quaternion_from_euler(yaw):
#     return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

# class DynamicPushbackPlanner(Node):
#     def __init__(self):
#         super().__init__('dynamic_pushback_planner')
        
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         --- PATH PUBLISHERS (To match your 2D Plot) ---
#         self.pubs = {
#             'drive_nose': self.create_publisher(Path, '/path/drive_nose', 10),
#             'trace_main': self.create_publisher(Path, '/path/trace_main', 10),
#             'left_wing': self.create_publisher(Path, '/path/track_left_wing', 10),
#             'right_wing': self.create_publisher(Path, '/path/track_right_wing', 10),
#             'tail': self.create_publisher(Path, '/path/track_tail', 10)
#         }
#         self.path_msgs = {k: Path() for k in self.pubs.keys()}
#         for p in self.path_msgs.values(): p.header.frame_id = 'world'
        
#         --- LOAD CSV DATA ---
#         script_dir = os.path.dirname(os.path.abspath(__file__))
#         self.csv_path = os.path.join(script_dir, '..', 'data', 'kinematic_Right Turn_MAC_16.5_Nose_Heavy.csv')
        
#         self.recorded_data = []
#         try:
#             with open(self.csv_path, 'r') as f:
#                 reader = csv.DictReader(f)
#                 for row in reader:
#                     self.recorded_data.append(row)
#             self.get_logger().info(f"SUCCESS: Loaded {len(self.recorded_data)} moments from CSV.")
#         except Exception as e:
#             self.get_logger().error(f"Failed to load CSV: {e}")
#             return

#         Load Physics Parameters
#         self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        
#         Provide fallback dimensions for wings/tail if not in JSON

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import csv
import os

from .physics_engine.load_parameters import load_parameters

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quaternion_from_euler(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class DynamicPushbackPlanner(Node):
    def __init__(self):
        super().__init__('dynamic_pushback_planner')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- PATH PUBLISHERS (To match your 2D Plot) ---
        self.pubs = {
            'drive_nose': self.create_publisher(Path, '/path/drive_nose', 10),
            'trace_main': self.create_publisher(Path, '/path/trace_main', 10),
        }
        self.wing_span = getattr(self.p.aircraft, 'wing_span', 24.57) # Example ATR42 span
        self.tail_dist = getattr(self.p.aircraft, 'tail_dist', 10.0)  # Distance behind main gear
        
        self.timer_period = 0.04 
        self.current_index = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Simulation Playback Started. Watch RViz for the Right Turn.")

    def timer_callback(self):
        if self.current_index >= len(self.recorded_data):
            self.get_logger().info("Playback Complete.")
            self.timer.cancel()
            return

        row = self.recorded_data[self.current_index]
        tx, ty, tyaw = float(row['X']), float(row['Y']), float(row['Yaw'])
        hitch = float(row['Hitch_Angle'])

        # --- KINEMATICS ---
        ayaw = normalize_angle(tyaw + hitch)
        
        # Hitch joint (Nose Gear)
        hx = tx + self.p.vehicle.c * math.cos(tyaw)
        hy = ty + self.p.vehicle.c * math.sin(tyaw)
        
        # Aircraft center (Main Gear)
        ax = hx - self.p.aircraft.L * math.cos(ayaw)
        ay = hy - self.p.aircraft.L * math.sin(ayaw)

        now = self.get_clock().now().to_msg()

        # --- UPDATE PATHS ---
        # 1. Drive (Nose Gear)
        self.update_path('drive_nose', hx, hy, ayaw, now)
        # 2. Trace (Main Gear)
        self.update_path('trace_main', ax, ay, ayaw, now)
        
        # 3. Wings
        lwx = ax - (self.wing_span/2 * math.sin(ayaw))
        lwy = ay + (self.wing_span/2 * math.cos(ayaw))
        self.update_path('left_wing', lwx, lwy, ayaw, now)
        
        rwx = ax + (self.wing_span/2 * math.sin(ayaw))
        rwy = ay - (self.wing_span/2 * math.cos(ayaw))
        self.update_path('right_wing', rwx, rwy, ayaw, now)

        # 4. Tail
        tax = ax - (self.tail_dist * math.cos(ayaw))
        tay = ay - (self.tail_dist * math.sin(ayaw))
        self.update_path('tail', tax, tay, ayaw, now)

        # --- PUBLISH TFs ---
        self.publish_tf('tug/base_link', tx, ty, tyaw, now)
        self.publish_tf('aircraft/base_link', ax, ay, ayaw, now)

        self.current_index += 1

    def update_path(self, key, x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'world'
        pose.pose.position.x, pose.pose.position.y = x, y
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        
        self.path_msgs[key].poses.append(pose)
        self.path_msgs[key].header.stamp = stamp
        self.pubs[key].publish(self.path_msgs[key])

    def publish_tf(self, frame_id, x, y, yaw, timestamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = timestamp, 'world', frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), 0.0
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPushbackPlanner()
    rclpy.spin(node)
    rclpy.shutdown()