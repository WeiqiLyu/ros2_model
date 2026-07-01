import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import math
import numpy as np

# Import your parameters and inputs
from .physics_engine.load_parameters import load_parameters
from .physics_engine.simulation_inputs import get_maneuver_scenarios


def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quaternion_from_euler(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class Dynamic1PushbackPlanner(Node):
    def __init__(self):
        super().__init__('dynamic1_pushback_planner')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- 1. LOAD SCENARIO FROM simulation_inputs.py ---
        scenarios = get_maneuver_scenarios()
        self.scenario_name = "Right Turn"
        self.scenario = scenarios[self.scenario_name]
        
        # --- 2. LOAD PHYSICS PARAMETERS ---
        self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        
        # Extract dimensions
        self.tug_wb = getattr(self.p.vehicle, 'wheelbase', 3.0) 
        self.hitch_length = getattr(self.p.vehicle, 'c', 5.0)  # Tug center to hitch
        self.ac_L = getattr(self.p.aircraft, 'L', 16.5)        # Nose gear to Main gear
        self.wing_span = getattr(self.p.aircraft, 'wing_span', 24.57)
        self.tail_dist = getattr(self.p.aircraft, 'tail_dist', 10.0)

        # --- 3. PHYSICS STATE ---
        self.tx, self.ty, self.tyaw = 0.0, 0.0, 0.0  # Tug Position
        self.ac_yaw = 0.0                            # Aircraft Heading

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

        # --- 5. CLOCK AND TIMER ---
        self.dt = 0.04  # 25Hz Simulation Rate
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.dt, self.physics_step)
        self.get_logger().info(f"Starting Scenario: {self.scenario_name}")

    def physics_step(self):
        # --- PHASE CHECKER ---
        current_v = 0.0
        current_steer = 0.0
        active_segment = False
        
        time_accumulator = 0.0
        # Iterate through the array from simulation_inputs.py
        for segment in self.scenario['path']:
            time_accumulator += segment['duration']
            if self.elapsed_time <= time_accumulator:
                current_v = segment['velocity']
                current_steer = segment['steering_angle']
                active_segment = True
                break
                
        if not active_segment:
            self.get_logger().info("Scenario Complete!")
            self.timer.cancel()
            return

        # --- KINEMATICS ENGINE ---
        # 1. Tug Movement
        self.tx += current_v * math.cos(self.tyaw) * self.dt
        self.ty += current_v * math.sin(self.tyaw) * self.dt
        self.tyaw += (current_v / self.tug_wb) * math.tan(current_steer) * self.dt

        # 2. Aircraft Movement (Trailer hitch logic)
        hitch_angle = self.tyaw - self.ac_yaw
        self.ac_yaw += (current_v / self.ac_L) * math.sin(hitch_angle) * self.dt

        # --- GEOMETRY CALCULATION ---
        # Nose Gear (Hitch point)
        hx = self.tx + (self.hitch_length * math.cos(self.tyaw))
        hy = self.ty + (self.hitch_length * math.sin(self.tyaw))
        # Main Gear (Aircraft Center)
        ax = hx - (self.ac_L * math.cos(self.ac_yaw))
        ay = hy - (self.ac_L * math.sin(self.ac_yaw))

        now = self.get_clock().now().to_msg()

        # --- UPDATE ALL PATHS ---
        self.update_path('drive_nose', hx, hy, self.ac_yaw, now)
        self.update_path('trace_main', ax, ay, self.ac_yaw, now)
        
        lwx = ax - (self.wing_span/2 * math.sin(self.ac_yaw))
        lwy = ay + (self.wing_span/2 * math.cos(self.ac_yaw))
        self.update_path('left_wing', lwx, lwy, self.ac_yaw, now)
        
        rwx = ax + (self.wing_span/2 * math.sin(self.ac_yaw))
        rwy = ay - (self.wing_span/2 * math.cos(self.ac_yaw))
        self.update_path('right_wing', rwx, rwy, self.ac_yaw, now)

        tax = ax - (self.tail_dist * math.cos(self.ac_yaw))
        tay = ay - (self.tail_dist * math.sin(self.ac_yaw))
        self.update_path('tail', tax, tay, self.ac_yaw, now)

        # --- PUBLISH TFS ---
        self.publish_tf('tug/base_link', self.tx, self.ty, self.tyaw, now)
        self.publish_tf('aircraft/base_link', ax, ay, self.ac_yaw, now)

        # Increment clock
        self.elapsed_time += self.dt

    def update_path(self, key, x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'world'
        pose.pose.position.x, pose.pose.position.y = x, y
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        
        self.path_msgs[key].poses.append(pose)
        self.pubs[key].publish(self.path_msgs[key])

    def publish_tf(self, frame_id, x, y, yaw, timestamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = timestamp, 'world', frame_id
        t.transform.translation.x, t.transform.translation.y = x, y
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Dynamic1PushbackPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()