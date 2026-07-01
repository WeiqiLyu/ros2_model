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
        
        # Load CSV
        filename = 'towing_path_halle_turn_expand_new.csv'
        self.csv_path = os.path.expanduser(f'~/flight_ws/src/aircraft_design/data/{filename}')
        self.recorded_data = []
        
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.recorded_data.append({
                    'x': float(row['x_r_m']),
                    'y': float(row['y_r_m']),
                    'theta': math.radians(float(row['theta_r_deg'])),
                    'theta_a': math.radians(float(row['theta_a_deg'])),
                    'beta': math.radians(float(row['beta_deg']))
                })

        self.markers = MarkerArray()
        self.generate_markers()
        
        self.idx = 0
        self.create_timer(0.05, self.timer_callback) # 20 Hz playback
        self.create_timer(1.0, self.publish_markers) # Keep markers alive

    def generate_markers(self):
        def create_line_marker(ns, m_id, r, g, b):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = ns
            m.id = m_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
            return m
            
        def create_arrow_marker(ns, m_id, r, g, b, pose_data):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = ns
            m.id = m_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 2.0; m.scale.y = 0.5; m.scale.z = 0.5
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
            
            hx = pose_data['x'] + self.c * math.cos(pose_data['theta'])
            hy = pose_data['y'] + self.c * math.sin(pose_data['theta'])
            
            m.pose.position.x = hx
            m.pose.position.y = hy
            m.pose.position.z = 0.0
            q = quaternion_from_euler(pose_data['theta_a'])
            m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
            return m

        # 1. Planned Trajectory Lines (Blue / Light Blue)
        m_rear = create_line_marker("planned_rear_axle", 0, 0.0, 0.0, 1.0)
        m_left = create_line_marker("planned_mlg_left", 1, 0.5, 0.8, 1.0)
        m_right = create_line_marker("planned_mlg_right", 2, 0.5, 0.8, 1.0)

        # 2. Reference Trajectory Lines (Green / Light Green)
        m_ref_rear = create_line_marker("ref_rear_axle", 10, 0.0, 1.0, 0.0)
        m_ref_left = create_line_marker("ref_mlg_left", 11, 0.5, 1.0, 0.5)
        m_ref_right = create_line_marker("ref_mlg_right", 12, 0.5, 1.0, 0.5)

        for pt in self.recorded_data:
            # --- PLANNED POSITIONS ---
            m_rear.points.append(Point(x=pt['x'], y=pt['y'], z=0.0))
            
            xh = pt['x'] + self.c * math.cos(pt['theta'])
            yh = pt['y'] + self.c * math.sin(pt['theta'])
            xb = xh - self.L2 * math.cos(pt['theta_a'])
            yb = yh - self.L2 * math.sin(pt['theta_a'])
            
            xl = xb - (self.L_landing/2) * math.sin(pt['theta_a'])
            yl = yb + (self.L_landing/2) * math.cos(pt['theta_a'])
            m_left.points.append(Point(x=xl, y=yl, z=0.0))
            
            xr = xb + (self.L_landing/2) * math.sin(pt['theta_a'])
            yr = yb - (self.L_landing/2) * math.cos(pt['theta_a'])
            m_right.points.append(Point(x=xr, y=yr, z=0.0))

            # --- REFERENCE POSITIONS (Mocked with a 0.5m offset to show tracking difference) ---
            offset = 0.5
            m_ref_rear.points.append(Point(x=pt['x'] + offset, y=pt['y'] + offset, z=0.0))
            m_ref_left.points.append(Point(x=xl + offset, y=yl + offset, z=0.0))
            m_ref_right.points.append(Point(x=xr + offset, y=yr + offset, z=0.0))

        # 3. Start and End Arrows (Yellow / Red)
        m_start = create_arrow_marker("start_pose", 20, 1.0, 1.0, 0.0, self.recorded_data[0])
        m_end = create_arrow_marker("end_pose", 21, 1.0, 0.0, 0.0, self.recorded_data[-1])

        # Publish all 8 markers exactly as specified in the table
        self.markers.markers.extend([
            m_rear, m_left, m_right, 
            m_ref_rear, m_ref_left, m_ref_right, 
            m_start, m_end
        ])

    def publish_markers(self):
        for m in self.markers.markers:
            m.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.markers)

    def timer_callback(self):
        if self.idx >= len(self.recorded_data):
            self.idx = 0 # Loop
            
        pt = self.recorded_data[self.idx]
        now = self.get_clock().now().to_msg()
        
        # 1. Update Tractor Pose (TF)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'tractor_base_link'
        t.transform.translation.x = pt['x']
        t.transform.translation.y = pt['y']
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(pt['theta'])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Update Hitch Angle (JointState)
        js = JointState()
        js.header.stamp = now
        js.name = ['tow_joint']
        js.position = [pt['beta']]
        self.joint_pub.publish(js)
        
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = CSVTrajectoryPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()