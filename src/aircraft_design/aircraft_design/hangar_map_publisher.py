import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml
import cv2
import numpy as np
import os

class HangarMapPublisher(Node):
    def __init__(self):
        super().__init__('hangar_map_publisher')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/hangar_map', qos)
        
        # Correct Path to the Maps folder
        self.map_dir = os.path.expanduser('/home/student/flight_ws/src/aircraft_design/maps')
        self.yaml_path = os.path.join(self.map_dir, 'halle_straight_map.yaml')
        self.load_and_publish_map()

    def load_and_publish_map(self):
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f"YAML NOT FOUND AT: {self.yaml_path}")
            return
        with open(self.yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        img_path = os.path.join(self.map_dir, map_data['image'])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"IMAGE NOT FOUND AT: {img_path}")
            return

        img = cv2.flip(img, 0)
        grid = np.zeros_like(img, dtype=np.int8)
        grid[img > 230] = 0   # White
        grid[img < 50] = 100  # Black
        grid[(img >= 50) & (img <= 230)] = -1 # Grey
        
        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.info.resolution = float(map_data['resolution'])
        msg.info.width, msg.info.height = img.shape[1], img.shape[0]
        msg.info.origin.position.x, msg.info.origin.position.y = float(map_data['origin'][0]), float(map_data['origin'][1])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.map_pub.publish(msg)
        self.get_logger().info("Hangar Map Published Successfully!")

def main():
    rclpy.init()
    rclpy.spin(HangarMapPublisher())