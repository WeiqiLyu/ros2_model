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

from .physics_engine.load_parameters import load_parameters
from .physics_engine.simulation_inputs import get_maneuver_scenarios

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def quaternion_from_euler(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class HangarDynamicPlanner(Node):
    def __init__(self):
        super().__init__('hangar_dynamic_planner')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pkg_dir = get_package_share_directory('aircraft_design')
        
        # --- 1. LOAD SCENARIO ---
        scenarios = get_maneuver_scenarios()
        self.scenario_name = "Right Turn" 
        self.scenario = scenarios[self.scenario_name]
        
        # --- 2. LOAD PHYSICS PARAMETERS ---
        self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        self.tug_wb = getattr(self.p.vehicle, 'wheelbase', 3.0) 
        self.tug_mass = getattr(self.p.vehicle, 'mass', 5000.0)

        # Force towbar distance to zero to keep kinematics tied to the hitch point
        self.c = 0.0  

        self.L_a = getattr(self.p.aircraft, 'L', 16.5)        
        self.L_wing = getattr(self.p.aircraft, 'L_wing', 24.57)
        self.L_tail = getattr(self.p.aircraft, 'L_tail', 8.0)
        self.L_plane = getattr(self.p.aircraft, 'L_plane', 24.0)
        self.L_nose = getattr(self.p.aircraft, 'L_nose', 2.0) 
        self.le_mac = getattr(self.p.aircraft, 'le_mac', 0.0)
        self.datum = getattr(self.p.aircraft, 'datum', 0.0)
        self.mac = getattr(self.p.aircraft, 'mac', 0.0)
        self.ac_mass = getattr(self.p.aircraft, 'mass', 20000.0)

        self.total_mass = self.tug_mass + self.ac_mass

        
        # --- 3. ULTIMATE VISUAL CALIBRATION HUB ---
    
        # A. TUG CALIBRATION
        # If the tug swings out of the green line during a turn, 
        
        self.tug_visual_x = -1.5 
        self.tug_visual_y = 1.0
        
        self.tug_visual_yaw = -(math.pi / 2) 
        
        # B. AIRCRAFT CALIBRATION
        self.ac_visual_x = 0.0
        self.ac_visual_y = 0.0
        self.ac_visual_yaw = -(math.pi / 2)
        
        # C. TAIL TRACE CALIBRATION
        self.tail_distance_from_nose = 21.5 
        

        # --- 4. STARTING POSITION ---
        self.tx = -609.913                 
        self.ty = -649.950                 
        self.tyaw = np.deg2rad(30.000)     
        self.ac_yaw = self.tyaw            
        self.current_velocity = 0.0  
        self.history = [] 

        # --- 5. PATH PUBLISHERS ---
        self.pubs = {
            'drive_nose': self.create_publisher(Path, '/path/drive_nose', 10),
            'trace_main': self.create_publisher(Path, '/path/trace_main', 10),
            'left_wing': self.create_publisher(Path, '/path/track_left_wing', 10),
            'right_wing': self.create_publisher(Path, '/path/track_right_wing', 10),
            'tail': self.create_publisher(Path, '/path/track_tail', 10)
        }
        self.path_msgs = {k: Path() for k in self.pubs.keys()}
        for p in self.path_msgs.values(): p.header.frame_id = 'world'

        self.dt = 0.04  
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.dt, self.physics_step)

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
            self.save_to_csv()
            self.timer.cancel()
            return

        # --- DYNAMICS ---
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
        self.ac_yaw += (self.current_velocity / self.L_a) * math.sin(hitch_angle) * self.dt

        # --- TRACE MATH (Draws the perfect lines) ---
        nx = self.tx - self.c * math.cos(self.tyaw)
        ny = self.ty - self.c * math.sin(self.tyaw)
        
        cx = nx - self.L_a * math.cos(self.ac_yaw)
        cy = ny - self.L_a * math.sin(self.ac_yaw)

        wing_lon_offset = self.le_mac - self.datum - self.L_nose + (self.mac / 2)
        wing_dist = self.L_a - wing_lon_offset

        lwx = nx - wing_dist * math.cos(self.ac_yaw) - (self.L_wing / 2) * math.sin(self.ac_yaw)
        lwy = ny - wing_dist * math.sin(self.ac_yaw) + (self.L_wing / 2) * math.cos(self.ac_yaw)
        
        rwx = nx - wing_dist * math.cos(self.ac_yaw) + (self.L_wing / 2) * math.sin(self.ac_yaw)
        rwy = ny - wing_dist * math.sin(self.ac_yaw) - (self.L_wing / 2) * math.cos(self.ac_yaw)

        tax = nx - self.tail_distance_from_nose * math.cos(self.ac_yaw)
        tay = ny - self.tail_distance_from_nose * math.sin(self.ac_yaw)

        self.history.append({
            'time': round(self.elapsed_time, 2),
            'tug_x': round(self.tx, 4), 'tug_y': round(self.ty, 4), 'tug_yaw': round(self.tyaw, 4),
            'ac_x': round(cx, 4), 'ac_y': round(cy, 4), 'ac_yaw': round(self.ac_yaw, 4)
        })

        now = self.get_clock().now().to_msg()
        self.update_path('drive_nose', nx, ny, self.ac_yaw, now)
        self.update_path('trace_main', cx, cy, self.ac_yaw, now)
        self.update_path('left_wing', lwx, lwy, self.ac_yaw, now)
        self.update_path('right_wing', rwx, rwy, self.ac_yaw, now)
        self.update_path('tail', tax, tay, self.ac_yaw, now)

       
        #  VISUAL PUBLISHING (Applies manual slide offsets) 
        
        v_tyaw = self.tyaw + self.tug_visual_yaw
        v_ayaw = self.ac_yaw + self.ac_visual_yaw

        # Shifts the Tug mesh away from its built-in orbital origin
        v_tx = self.tx + (self.tug_visual_x * math.cos(v_tyaw) - self.tug_visual_y * math.sin(v_tyaw))
        v_ty = self.ty + (self.tug_visual_x * math.sin(v_tyaw) + self.tug_visual_y * math.cos(v_tyaw))

        # Shifts the Aircraft mesh 
        v_ax = nx + (self.ac_visual_x * math.cos(v_ayaw) - self.ac_visual_y * math.sin(v_ayaw))
        v_ay = ny + (self.ac_visual_x * math.sin(v_ayaw) + self.ac_visual_y * math.cos(v_ayaw))

        self.publish_tf('tug/base_link', v_tx, v_ty, 1.0, v_tyaw, now)
        self.publish_tf('aircraft/base_link', v_ax, v_ay, 1.0, v_ayaw, now)

        self.elapsed_time += self.dt

    def save_to_csv(self):
        file_name = f"hangar_dynamic_{self.scenario_name.replace(' ', '_')}.csv"
        # Write into src/ (not the install/ copy) so results are version-controlled
        # and survive the next colcon build, regardless of workspace location/name.
        ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(self.pkg_dir))))
        workspace_src_data = os.path.join(ws_root, 'src', 'aircraft_design', 'data')
        if not os.path.exists(workspace_src_data):
            os.makedirs(workspace_src_data)
        csv_path = os.path.join(workspace_src_data, file_name)
        keys = ['time', 'tug_x', 'tug_y', 'tug_yaw', 'ac_x', 'ac_y', 'ac_yaw']
        try:
            with open(csv_path, 'w', newline='') as output_file:
                dict_writer = csv.DictWriter(output_file, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.history)
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

    def publish_tf(self, frame_id, x, y, z, yaw, timestamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = timestamp, 'world', frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        qx, qy, qz, qw = quaternion_from_euler(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = HangarDynamicPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()