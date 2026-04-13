import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import csv
import os
from ament_index_python.packages import get_package_share_directory

# Import your parameters and inputs
from .physics_engine.load_parameters import load_parameters
from .physics_engine.simulation_inputs import get_maneuver_scenarios

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quaternion_from_euler(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class Dynamic2PushbackPlanner(Node):
    def __init__(self):
        super().__init__('dynamic2_pushback_planner')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pkg_dir = get_package_share_directory('aircraft_design')
        
        # --- 1. LOAD SCENARIO ---
        scenarios = get_maneuver_scenarios()
        self.scenario_name = "Right Turn"
        self.scenario = scenarios[self.scenario_name]
        
        # --- 2. LOAD PHYSICS PARAMETERS ---
        self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        
        self.tug_wb = getattr(self.p.vehicle, 'wheelbase', 3.0) 
        self.hitch_length = getattr(self.p.vehicle, 'c', 5.0)  
        self.ac_L = getattr(self.p.aircraft, 'L', 16.5)        
        self.wing_span = getattr(self.p.aircraft, 'wing_span', 24.57)
        self.tail_dist = getattr(self.p.aircraft, 'tail_dist', 10.0)

        self.tug_mass = getattr(self.p.vehicle, 'mass', 5000.0)
        self.ac_mass = getattr(self.p.aircraft, 'mass', 20000.0)
        self.total_mass = self.tug_mass + self.ac_mass

        # --- YOUR PERFECT CALIBRATION OFFSETS ---
        self.local_x = 2.5
        self.local_y = -0.5

        # --- 3. PHYSICS STATE ---
        self.tx, self.ty, self.tyaw = 0.0, 0.0, 0.0  
        self.ac_yaw = 0.0                            
        self.current_velocity = 0.0  

        # --- NEW: DATA RECORDER ---
        self.history = [] # This will store our rows

        # --- 4. PATH PUBLISHERS ---
        self.pubs = {
            'drive_nose': self.create_publisher(Path, '/path/drive_nose', 10),
            'trace_main': self.create_publisher(Path, '/path/trace_main', 10),
            'left_wing': self.create_publisher(Path, '/path/track_left_wing', 10),
            'right_wing': self.create_publisher(Path, '/path/track_right_wing', 10),
            'tail': self.create_publisher(Path, '/path/track_tail', 10)
        }
        self.path_msgs = {k: Path() for k in self.pubs.keys()}
        for p in self.path_msgs.values(): p.header.frame_id = 'world'

        # --- 5. TIMER ---
        self.dt = 0.04  
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.dt, self.physics_step)
        self.get_logger().info(f"Starting DYNAMIC Scenario: {self.scenario_name}")

    def physics_step(self):
        target_v = 0.0
        current_steer = 0.0
        active_segment = False
        
        time_accumulator = 0.0
        for segment in self.scenario['path']:
            time_accumulator += segment['duration']
            if self.elapsed_time <= time_accumulator:
                target_v = segment['velocity']
                current_steer = segment['steering_angle']
                active_segment = True
                break
                
        if not active_segment:
            self.get_logger().info("Scenario Complete! Saving CSV...")
            self.save_to_csv()
            self.timer.cancel()
            return

        # --- DYNAMICS (F = ma) ---
        force = self.scenario['propulsive_force']
        acceleration = force / self.total_mass
        
        if abs(self.current_velocity) < abs(target_v):
            self.current_velocity += acceleration * self.dt
        else:
            self.current_velocity = target_v

        # --- KINEMATICS ---
        self.tx += self.current_velocity * math.cos(self.tyaw) * self.dt
        self.ty += self.current_velocity * math.sin(self.tyaw) * self.dt
        self.tyaw += (self.current_velocity / self.tug_wb) * math.tan(current_steer) * self.dt

        hitch_angle = self.tyaw - self.ac_yaw
        self.ac_yaw += (self.current_velocity / self.ac_L) * math.sin(hitch_angle) * self.dt

        # --- APPLYING YOUR CALIBRATION TO THE DYNAMICS ---
        # 1. Physics angles (raw)
        raw_tyaw = self.tyaw
        raw_ayaw = self.ac_yaw

        # 2. Visual angles (180 degree flip)
        visual_tyaw = raw_tyaw - (math.pi / 2)
        visual_ayaw = raw_ayaw - (math.pi / 2)

        # 3. Compute dynamic nose point from tug physics
        hx = self.tx + (self.hitch_length * math.cos(raw_tyaw))
        hy = self.ty + (self.hitch_length * math.sin(raw_tyaw))
        
        # 4. Compute aircraft center using your perfect offsets!
        ax = hx + (self.local_x * math.cos(visual_ayaw) - self.local_y * math.sin(visual_ayaw))
        ay = hy + (self.local_x * math.sin(visual_ayaw) + self.local_y * math.cos(visual_ayaw))

        # --- RECORD THE DATA ---
        self.history.append({
            'time': round(self.elapsed_time, 2),
            'tug_x': round(self.tx, 4),
            'tug_y': round(self.ty, 4),
            'tug_yaw': round(self.tyaw, 4),
            'ac_x': round(ax, 4),
            'ac_y': round(ay, 4),
            'ac_yaw': round(self.ac_yaw, 4)
        })

        now = self.get_clock().now().to_msg()

        # Update paths (using raw angles so the map lines draw properly)
        self.update_path('drive_nose', hx, hy, raw_ayaw, now)
        self.update_path('trace_main', ax, ay, raw_ayaw, now)
        
        lwx = ax - (self.wing_span/2 * math.sin(raw_ayaw))
        lwy = ay + (self.wing_span/2 * math.cos(raw_ayaw))
        self.update_path('left_wing', lwx, lwy, raw_ayaw, now)
        
        rwx = ax + (self.wing_span/2 * math.sin(raw_ayaw))
        rwy = ay - (self.wing_span/2 * math.cos(raw_ayaw))
        self.update_path('right_wing', rwx, rwy, raw_ayaw, now)

        tax = ax - (self.tail_dist * math.cos(raw_ayaw))
        tay = ay - (self.tail_dist * math.sin(raw_ayaw))
        self.update_path('tail', tax, tay, raw_ayaw, now)

        # 5. PUBLISH HEIGHT FIX: Tug and Aircraft both exactly at Z=1.0
        self.publish_tf('tug/base_link', self.tx, self.ty, 1.0, visual_tyaw, now)
        self.publish_tf('aircraft/base_link', ax, ay, 1.0, visual_ayaw, now)

        self.elapsed_time += self.dt

    def save_to_csv(self):
        """Saves the recorded history to a CSV file."""
        file_name = f"generated_dynamic_{self.scenario_name.replace(' ', '_')}.csv"
        workspace_src_data = os.path.expanduser('~/flight_ws/src/aircraft_design/data')
        
        if not os.path.exists(workspace_src_data):
            os.makedirs(workspace_src_data)

        csv_path = os.path.join(workspace_src_data, file_name)
        keys = ['time', 'tug_x', 'tug_y', 'tug_yaw', 'ac_x', 'ac_y', 'ac_yaw']
        
        try:
            with open(csv_path, 'w', newline='') as output_file:
                dict_writer = csv.DictWriter(output_file, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.history)
            self.get_logger().info(f"SUCCESS: Generated trajectory saved to {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def update_path(self, key, x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'world'
        pose.pose.position.x, pose.pose.position.y = x, y
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        
        self.path_msgs[key].poses.append(pose)
        self.pubs[key].publish(self.path_msgs[key])

    # ADDED 'z' parameter to handle your 1.0 height calibration
    def publish_tf(self, frame_id, x, y, z, yaw, timestamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = timestamp, 'world', frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Dynamic2PushbackPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()