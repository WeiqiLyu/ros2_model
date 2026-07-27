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
        
        # ==========================================================
        # ICD Geometry Constants (Matches URDF perfectly)
        # ==========================================================
        self.c = 0.023
        self.L2 = 8.146          
        self.L_landing = 4.0309   
        
        # Load Active CSV File
        filename = 'trajectories.csv'
        self.csv_path = os.path.expanduser(f'~/flight_ws/src/aircraft_design/data/{filename}')
        self.recorded_data = []
        
        with open(self.csv_path, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [name.strip() for name in reader.fieldnames]
            
            for row in reader:
                dyn_theta = float(row['dyn_theta'])
                dyn_hitch = float(row['dyn_hitch'])
                kin_theta = float(row['kin_theta'])
                kin_hitch = float(row['kin_hitch'])
                
                self.recorded_data.append({
                    'x': float(row['dyn_x']),
                    'y': float(row['dyn_y']),
                    'theta': dyn_theta,
                    'theta_a': dyn_theta - dyn_hitch,
                    'beta': dyn_hitch,
                    
                    'kin_x': float(row['kin_x']),
                    'kin_y': float(row['kin_y']),
                    'kin_theta': kin_theta,
                    'kin_theta_a': kin_theta - kin_hitch
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
        m.scale.x = 3.0; m.scale.y = 0.8; m.scale.z = 0.8
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        
        hx = pose_data['x'] + self.c * math.cos(pose_data['theta'])
        hy = pose_data['y'] + self.c * math.sin(pose_data['theta'])
        
        m.pose.position.x = hx
        m.pose.position.y = hy
        m.pose.position.z = 0.5
        q = quaternion_from_euler(pose_data['theta_a'])
        m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
        return m

    def generate_static_markers(self):
        m_rear = self.create_line_marker("planned_rear_axle", 0, 0.0, 0.10, 0.95, is_dotted=True) 
        m_left = self.create_line_marker("planned_mlg_left", 1, 0.0, 0.75, 1.00, is_dotted=True) 
        m_right = self.create_line_marker("planned_mlg_right", 2, 0.0, 0.75, 1.00, is_dotted=True) 

        m_ref_rear = self.create_line_marker("ref_rear_axle", 10, 0.0, 0.55, 0.15) 
        m_ref_left = self.create_line_marker("ref_mlg_left", 11, 0.25, 0.95, 0.25) 
        m_ref_right = self.create_line_marker("ref_mlg_right", 12, 0.25, 0.95, 0.25) 

        for pt in self.recorded_data:
            m_ref_rear.points.append(Point(x=pt['kin_x'], y=pt['kin_y'], z=0.20))
            
            kin_xh = pt['kin_x'] + self.c * math.cos(pt['kin_theta'])
            kin_yh = pt['kin_y'] + self.c * math.sin(pt['kin_theta'])
            kin_xb = kin_xh - self.L2 * math.cos(pt['kin_theta_a'])
            kin_yb = kin_yh - self.L2 * math.sin(pt['kin_theta_a'])
            
            kin_xl = kin_xb - (self.L_landing/2) * math.sin(pt['kin_theta_a'])
            kin_yl = kin_yb + (self.L_landing/2) * math.cos(pt['kin_theta_a'])
            m_ref_left.points.append(Point(x=kin_xl, y=kin_yl, z=0.20))
            
            kin_xr = kin_xb + (self.L_landing/2) * math.sin(pt['kin_theta_a'])
            kin_yr = kin_yb - (self.L_landing/2) * math.cos(pt['kin_theta_a'])
            m_ref_right.points.append(Point(x=kin_xr, y=kin_yr, z=0.20))

        m_start = self.create_arrow_marker("start_pose", 20, 1.0, 0.90, 0.0, self.recorded_data[0])
        m_end = self.create_arrow_marker("end_pose", 21, 0.95, 0.10, 0.10, self.recorded_data[-1])

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
            self.idx = 0 
            
        pt = self.recorded_data[self.idx]
        now = self.get_clock().now().to_msg()
        
        # ==========================================================
        # 1. BROADCAST TRACTOR POSE (map -> tractor_base_link)
        # ==========================================================
        t_tractor = TransformStamped()
        t_tractor.header.stamp = now
        t_tractor.header.frame_id = 'map'
        t_tractor.child_frame_id = 'tractor_base_link'
        t_tractor.transform.translation.x = pt['x']
        t_tractor.transform.translation.y = pt['y']
        t_tractor.transform.translation.z = 0.0
        q_tractor = quaternion_from_euler(pt['theta'])
        t_tractor.transform.rotation.x = q_tractor[0]
        t_tractor.transform.rotation.y = q_tractor[1]
        t_tractor.transform.rotation.z = q_tractor[2]
        t_tractor.transform.rotation.w = q_tractor[3]
        
        self.tf_broadcaster.sendTransform(t_tractor)
        
        # ==========================================================
        # 2. PUBLISH TOW JOINT STATE
        # The robot_state_publisher will read this and automatically 
        # broadcast the TF for tractor_hitch_link -> aircraft_nose_wheel_link
        # ==========================================================
        js = JointState()
        js.header.stamp = now
        js.name = ['tow_joint']
        # Depending on your CSV coordinate logic, this might need to be positive pt['beta'].
        # -pt['beta'] is used here to match standard tractor-trailer kinematic folding.
        js.position = [-pt['beta']] 
        self.joint_pub.publish(js)

        # ==========================================================
        # 3. UPDATE DYNAMIC PLANNED TRAJECTORY
        # ==========================================================
        lookahead_rows = 30
        end_idx = min(self.idx + lookahead_rows, len(self.recorded_data) - 1)
        
        m_rear, m_left, m_right = self.markers.markers[0], self.markers.markers[1], self.markers.markers[2]
        m_rear.points.clear(); m_left.points.clear(); m_right.points.clear()
        
        for i in range(self.idx, end_idx, 3):
            if i + 1 < len(self.recorded_data):
                p1 = self.recorded_data[i]
                p2 = self.recorded_data[i + 1]
                
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