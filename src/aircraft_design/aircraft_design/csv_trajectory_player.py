#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import math
import os

from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)] # x, y, z, w

class CSVTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('csv_trajectory_player')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/pushback_markers', 10)
        
        # ICD Geometry Constants
        self.c = 3.315
        self.L2 = 8.781
        self.L_landing = 4.10
        
        # Load New CSV File
        filename = 'trajectories.csv'
        self.csv_path = os.path.expanduser(f'~/flight_ws/src/aircraft_design/data/{filename}')
        self.recorded_data = []
        
        # Using utf-8-sig and stripping headers prevents Excel formatting crashes
        with open(self.csv_path, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [name.strip() for name in reader.fieldnames]
            
            for row in reader:
                dyn_theta = float(row['dyn_theta'])
                dyn_hitch = float(row['dyn_hitch'])
                ref_theta = float(row['ref_theta'])
                ref_hitch = float(row['ref_hitch'])
                
                self.recorded_data.append({
                    # Active Vehicle Pose (Kinematic / Dynamic tracking)
                    'x': float(row['dyn_x']),
                    'y': float(row['dyn_y']),
                    'theta': dyn_theta,
                    'theta_a': dyn_theta - dyn_hitch,
                    'beta': dyn_hitch,
                    
                    # Global Reference Path
                    'ref_x': float(row['ref_x']),
                    'ref_y': float(row['ref_y']),
                    'ref_theta': ref_theta,
                    'ref_theta_a': ref_theta - ref_hitch
                })

        self.markers = MarkerArray()
        self.generate_static_markers()
        
        self.idx = 0
        self.create_timer(0.05, self.timer_callback) # 20 Hz playback
        self.create_timer(1.0, self.publish_markers) # 1 Hz marker keep-alive

    def create_line_marker(self, ns, m_id, r, g, b, is_dotted=False):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = m_id
        # LINE_LIST uses pairs of points to draw disconnected segments (dots/dashes)
        m.type = Marker.LINE_LIST if is_dotted else Marker.LINE_STRIP
        m.action = Marker.ADD
        # Thicker line scaling (0.25m to 0.35m) ensures visibility from high camera zooms
        m.scale.x = 0.35 if is_dotted else 0.25
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95
        return m
        
    def create_arrow_marker(self, ns, m_id, r, g, b, pose_data):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 3.0; m.scale.y = 0.8; m.scale.z = 0.8
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        
        # Calculate nose wheel (hitch) position exactly as required by table
        hx = pose_data['x'] + self.c * math.cos(pose_data['theta'])
        hy = pose_data['y'] + self.c * math.sin(pose_data['theta'])
        
        m.pose.position.x = hx
        m.pose.position.y = hy
        m.pose.position.z = 0.5 # Elevated above floor
        q = quaternion_from_euler(pose_data['theta_a'])
        m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
        return m

    def generate_static_markers(self):
        # 1. Initialize Short-Horizon Planned Trajectory Lines (Blue / Light Blue) - IDs 0, 1, 2
        m_rear = self.create_line_marker("planned_rear_axle", 0, 0.0, 0.2, 1.0, is_dotted=True)
        m_left = self.create_line_marker("planned_mlg_left", 1, 0.4, 0.8, 1.0, is_dotted=True)
        m_right = self.create_line_marker("planned_mlg_right", 2, 0.4, 0.8, 1.0, is_dotted=True)

        # 2. Global Reference Trajectory Lines (Green / Light Green) - IDs 10, 11, 12
        m_ref_rear = self.create_line_marker("ref_rear_axle", 10, 0.0, 0.85, 0.1)
        m_ref_left = self.create_line_marker("ref_mlg_left", 11, 0.5, 1.0, 0.5)
        m_ref_right = self.create_line_marker("ref_mlg_right", 12, 0.5, 1.0, 0.5)

        # Populate Global Reference Trajectory across entire CSV dataset (elevated to Z=0.20m)
        for pt in self.recorded_data:
            m_ref_rear.points.append(Point(x=pt['ref_x'], y=pt['ref_y'], z=0.20))
            
            ref_xh = pt['ref_x'] + self.c * math.cos(pt['ref_theta'])
            ref_yh = pt['ref_y'] + self.c * math.sin(pt['ref_theta'])
            ref_xb = ref_xh - self.L2 * math.cos(pt['ref_theta_a'])
            ref_yb = ref_yh - self.L2 * math.sin(pt['ref_theta_a'])
            
            ref_xl = ref_xb - (self.L_landing/2) * math.sin(pt['ref_theta_a'])
            ref_yl = ref_yb + (self.L_landing/2) * math.cos(pt['ref_theta_a'])
            m_ref_left.points.append(Point(x=ref_xl, y=ref_yl, z=0.20))
            
            ref_xr = ref_xb + (self.L_landing/2) * math.sin(pt['ref_theta_a'])
            ref_yr = ref_yb - (self.L_landing/2) * math.cos(pt['ref_theta_a'])
            m_ref_right.points.append(Point(x=ref_xr, y=ref_yr, z=0.20))

        # 3. Start and End Poses for Nose Wheel (Yellow / Red Arrows) - IDs 20, 21
        m_start = self.create_arrow_marker("start_pose", 20, 1.0, 1.0, 0.0, self.recorded_data[0])
        m_end = self.create_arrow_marker("end_pose", 21, 1.0, 0.0, 0.0, self.recorded_data[-1])

        # Register all 8 verified markers into the array
        self.markers.markers.extend([
            m_rear, m_left, m_right, 
            m_ref_rear, m_ref_left, m_ref_right, 
            m_start, m_end
        ])

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = now
        self.marker_pub.publish(self.markers)

    def timer_callback(self):
        if self.idx >= len(self.recorded_data):
            self.idx = 0 # Loop playback
            
        pt = self.recorded_data[self.idx]
        now = self.get_clock().now().to_msg()
        
        # 1. Broadcast Tractor TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'tractor_base_link'
        t.transform.translation.x = pt['x']
        t.transform.translation.y = pt['y']
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(pt['theta'])
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Tow Joint Angle
        js = JointState()
        js.header.stamp = now
        js.name = ['tow_joint']
        js.position = [pt['beta']]
        self.joint_pub.publish(js)

        # 3. Dynamic Short-Horizon Planned Trajectory (Dotted line every 10 CSV lines)
        # Look ahead by 150 rows (~7.5 seconds of future path)
        lookahead_rows = 150
        end_idx = min(self.idx + lookahead_rows, len(self.recorded_data) - 1)
        
        m_rear, m_left, m_right = self.markers.markers[0], self.markers.markers[1], self.markers.markers[2]
        m_rear.points.clear(); m_left.points.clear(); m_right.points.clear()
        
        # Step by 10 rows: drawing a 4-row segment then leaving a 6-row gap creates distinct dots/dashes
        for i in range(0, end_idx, 10):
            if i + 4 < len(self.recorded_data):
                p1 = self.recorded_data[i]
                p2 = self.recorded_data[i + 4]
                
                # Elevated to Z = 0.30m so planned path floats clearly over both floor and green reference path
                m_rear.points.append(Point(x=p1['x'], y=p1['y'], z=0.30))
                m_rear.points.append(Point(x=p2['x'], y=p2['y'], z=0.30))
                
                for p_data, m_l, m_r in [(p1, m_left, m_right), (p2, m_left, m_right)]:
                    xh = p_data['x'] + self.c * math.cos(p_data['theta'])
                    yh = p_data['y'] + self.c * math.sin(p_data['theta'])
                    xb = xh - self.L2 * math.cos(p_data['theta_a'])
                    yb = yh - self.L2 * math.sin(p_data['theta_a'])
                    
                    xl = xb - (self.L_landing/2) * math.sin(p_data['theta_a'])
                    yl = yb + (self.L_landing/2) * math.cos(p_data['theta_a'])
                    m_l.points.append(Point(x=xl, y=yl, z=0.30))
                    
                    xr = xb + (self.L_landing/2) * math.sin(p_data['theta_a'])
                    yr = yb - (self.L_landing/2) * math.cos(p_data['theta_a'])
                    m_r.points.append(Point(x=xr, y=yr, z=0.30))

        # Publish updated short-horizon dotted lines smoothly at 20 Hz
        self.marker_pub.publish(self.markers)
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = CSVTrajectoryPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()