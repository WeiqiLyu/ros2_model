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

class TugTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('tug_trajectory_player')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/pushback_markers', 10)
        
        # Load active CSV file
        filename = 'trajectories.csv'
        self.csv_path = os.path.expanduser(f'~/flight_ws/src/aircraft_design/data/{filename}')
        self.recorded_data = []
        
        with open(self.csv_path, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [name.strip() for name in reader.fieldnames]
            
            for row in reader:
                self.recorded_data.append({
                    # Dynamic columns (Driven tug behavior at center of rear wheels)
                    'x': float(row['dyn_x']),
                    'y': float(row['dyn_y']),
                    'theta': float(row['dyn_theta']),
                    'beta': float(row['dyn_hitch']),
                    
                    # Kinematic columns (Global reference baseline at center of rear wheels)
                    'kin_x': float(row['kin_x']),
                    'kin_y': float(row['kin_y']),
                    'kin_theta': float(row['kin_theta'])
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
        m.type = Marker.LINE_LIST if is_dotted else Marker.LINE_STRIP
        m.action = Marker.ADD
        # 0.50m thickness for dotted line guarantees bold, unmistakable visibility!
        m.scale.x = 0.50 if is_dotted else 0.25
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        return m
        
    def create_arrow_marker(self, ns, m_id, r, g, b, pose_data):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 2.5; m.scale.y = 0.7; m.scale.z = 0.7
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        
        m.pose.position.x = pose_data['x']
        m.pose.position.y = pose_data['y']
        m.pose.position.z = 0.4 # Elevated above floor
        q = quaternion_from_euler(pose_data['theta'])
        m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
        return m

    def generate_static_markers(self):
        # 1. Planned Dynamic Trajectory (0.50m THICK DOTTED DARK BLUE for Unified Tug Center)
        m_planned = self.create_line_marker("planned_trajectory", 0, 0.0, 0.15, 0.85, is_dotted=True) # Dark Blue

        # 2. Global Kinematic Reference Trajectory (0.25m SOLID GREEN for Unified Tug Center)
        m_ref = self.create_line_marker("ref_trajectory", 10, 0.0, 0.70, 0.15) # Solid Green

        # Populate Global Kinematic path across entire CSV (elevated to Z=0.20m)
        for pt in self.recorded_data:
            m_ref.points.append(Point(x=pt['kin_x'], y=pt['kin_y'], z=0.20))

        # 3. Start and End Arrows (Yellow / Red)
        m_start = self.create_arrow_marker("start_pose", 20, 1.0, 0.90, 0.0, self.recorded_data[0])
        m_end = self.create_arrow_marker("end_pose", 21, 0.95, 0.10, 0.10, self.recorded_data[-1])

        self.markers.markers.extend([m_planned, m_ref, m_start, m_end])

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = now
        self.marker_pub.publish(self.markers)

    def timer_callback(self):
        if self.idx >= len(self.recorded_data):
            self.idx = 0
            
        pt = self.recorded_data[self.idx]
        now = self.get_clock().now().to_msg()
        
        # 1. Broadcast Tug TF strictly to 'tug_base_link'
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'tug_base_link'
        t.transform.translation.x = pt['x']
        t.transform.translation.y = pt['y']
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(pt['theta'])
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Hitch Angle
        js = JointState()
        js.header.stamp = now
        js.name = ['tow_joint']
        js.position = [pt['beta']]
        self.joint_pub.publish(js)

        # 3. DYNAMIC SHORT HORIZON (Exactly 30 rows ahead from current point!)
        lookahead_rows = 30
        end_idx = min(self.idx + lookahead_rows, len(self.recorded_data) - 1)
        
        m_planned = self.markers.markers[0]
        m_planned.points.clear()
        
        # Step by 3 rows: drawing 1 row then skipping 2 creates 10 bold, distinct dots across exactly 30 rows!
        for i in range(self.idx, end_idx, 3):
            if i + 1 < len(self.recorded_data):
                p1 = self.recorded_data[i]
                p2 = self.recorded_data[i + 1]
                
                # Elevated to Z = 0.30m so thick dotted path floats clearly over solid green reference path
                m_planned.points.append(Point(x=p1['x'], y=p1['y'], z=0.30))
                m_planned.points.append(Point(x=p2['x'], y=p2['y'], z=0.30))

        self.marker_pub.publish(self.markers)
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = TugTrajectoryPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()