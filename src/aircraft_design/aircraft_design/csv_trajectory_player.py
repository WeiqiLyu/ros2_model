import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import csv
import os

def quaternion_from_euler(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class CSVTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('csv_trajectory_player')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        filename = 'towing_path_halle_turn_expand_new.csv'
        self.csv_path = os.path.expanduser(f'~/flight_ws/src/aircraft_design/data/{filename}')
        
        self.recorded_data = []
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.recorded_data.append(row)

        # YOUR EXACT OFFSETS
        self.local_x = 3.0
        self.local_y = -0.7
        
        self.wing_span = 24.57 

        self.pubs = {
            'nose': self.create_publisher(Path, '/path/drive_nose', 10),
            'main': self.create_publisher(Path, '/path/trace_main', 10),
            'left': self.create_publisher(Path, '/path/track_left_wing', 10),
            'right': self.create_publisher(Path, '/path/track_right_wing', 10)
        }
        self.paths = {k: Path() for k in self.pubs.keys()}
        for p in self.paths.values(): p.header.frame_id = 'world'

        self.idx = 0
        self.create_timer(0.04, self.timer_callback)

    def timer_callback(self):
        if self.idx >= len(self.recorded_data):
            return

        row = self.recorded_data[self.idx]
        
        # Raw CSV positions
        tx = float(row['x_r_m'])
        ty = float(row['y_r_m'])
        hx = float(row['nose_x_m'])
        hy = float(row['nose_y_m'])
        
        # 1. Get RAW angles
        raw_tyaw = np.deg2rad(float(row['theta_r_deg']))
        raw_ayaw = np.deg2rad(float(row['theta_a_deg']))

        # --- THE 180 DEGREE FLIP ---
        # Instead of adding 90 degrees (+ math.pi/2), we SUBTRACT 90 degrees (- math.pi/2).
        # This aligns the 3D model AND flips it 180 degrees so the tail moves first!
        visual_tyaw = raw_tyaw - (math.pi / 2)
        visual_ayaw = raw_ayaw - (math.pi / 2)

        # 3. Apply your exact 4.0 and -0.5 offsets using the FLIPPED visual angle!
        ax = hx + (self.local_x * math.cos(visual_ayaw) - self.local_y * math.sin(visual_ayaw))
        ay = hy + (self.local_x * math.sin(visual_ayaw) + self.local_y * math.cos(visual_ayaw))

        now = self.get_clock().now().to_msg()

        # Update paths for visual lines (using raw angles so the lines draw correctly on the map)
        self.update_path('nose', hx, hy, raw_ayaw, now)
        self.update_path('main', ax, ay, raw_ayaw, now)
        
        lwx = ax - (self.wing_span/2 * math.sin(raw_ayaw))
        lwy = ay + (self.wing_span/2 * math.cos(raw_ayaw))
        self.update_path('left', lwx, lwy, raw_ayaw, now)
        
        rwx = ax + (self.wing_span/2 * math.sin(raw_ayaw))
        rwy = ay - (self.wing_span/2 * math.cos(raw_ayaw))
        self.update_path('right', rwx, rwy, raw_ayaw, now)

        # 4. PUBLISH HEIGHT FIX: Tug and Aircraft both exactly at Z=0.0
        self.publish_tf('tug/base_link', tx, ty, 1.0, visual_tyaw, now)
        self.publish_tf('aircraft/base_link', ax, ay, 1.0, visual_ayaw, now)

        self.idx += 1

    def update_path(self, key, x, y, yaw, stamp):
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'world'
        pose.pose.position.x, pose.pose.position.y = x, y
        self.paths[key].poses.append(pose)
        self.pubs[key].publish(self.paths[key])

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
    node = CSVTrajectoryPlayer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()