import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
import numpy as np
import cv2
import csv
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from .physics_engine.load_parameters import load_parameters

class SweptPathMapper(Node):
    def __init__(self):
        super().__init__('swept_path_mapper')
        
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/swept_path_map', qos)
        
        self.p = load_parameters("TF6_parameters.json", "ATR42_300_parameters.json")
        self.margin = 3.0 
        
        self.tug_W = 3.0 + (self.margin * 2)
        self.tug_L = 6.0 + (self.margin * 2)
        self.ac_L = getattr(self.p.aircraft, 'L', 16.5) 
        self.wing_span = getattr(self.p.aircraft, 'wing_span', 24.57) + (self.margin * 2)
        self.wing_chord = 3.0 + (self.margin * 2)
        self.fuselage_W = 3.0 + (self.margin * 2)
        self.nose_to_tail = 25.0 + (self.margin * 2)
        
        self.resolution = 0.2  
        self.map_width_m = 400.0   
        self.map_height_m = 400.0  
        self.origin_x = -200.0     
        self.origin_y = -200.0     
        
        self.grid_w = int(self.map_width_m / self.resolution)
        self.grid_h = int(self.map_height_m / self.resolution)
        
        self.grid = np.full((self.grid_h, self.grid_w), 100, dtype=np.int8)
        
        workspace_src_data = os.path.expanduser('~/flight_ws/src/aircraft_design/data')
        self.csv_path = os.path.join(workspace_src_data, 'generated_dynamic_Right_Turn.csv')
        
        # Define where to save the map files (we will create a 'maps' folder)
        self.map_save_dir = os.path.expanduser('~/flight_ws/src/aircraft_design/maps')
        if not os.path.exists(self.map_save_dir):
            os.makedirs(self.map_save_dir)
            
        self.generate_map()

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return [gx, gy]

    def draw_rotated_rect(self, cx, cy, yaw, length, width):
        corners = np.array([
            [-length/2, -width/2],
            [ length/2, -width/2],
            [ length/2,  width/2],
            [-length/2,  width/2]
        ])
        
        R = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw),  math.cos(yaw)]
        ])
        
        pixels = []
        for point in corners:
            world_pt = R.dot(point) + np.array([cx, cy])
            pixels.append(self.world_to_grid(world_pt[0], world_pt[1]))
            
        pts = np.array(pixels, np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(self.grid, [pts], 0) 

    def generate_map(self):
        self.get_logger().info("Generating Occupancy Grid...")
        
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    tx, ty, tyaw = float(row['tug_x']), float(row['tug_y']), float(row['tug_yaw'])
                    ax, ay, ayaw = float(row['ac_x']), float(row['ac_y']), float(row['ac_yaw'])
                    
                    self.draw_rotated_rect(tx, ty, tyaw, self.tug_L, self.tug_W)
                    
                    fuse_cx = ax + (self.ac_L / 2) * math.cos(ayaw)
                    fuse_cy = ay + (self.ac_L / 2) * math.sin(ayaw)
                    self.draw_rotated_rect(fuse_cx, fuse_cy, ayaw, self.nose_to_tail, self.fuselage_W)
                    
                    self.draw_rotated_rect(ax, ay, ayaw, self.wing_chord, self.wing_span)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV or generate map: {e}")
            return

        self.get_logger().info("Map generated. Publishing to RViz and saving to disk...")
        self.publish_map()
        self.save_to_disk()

    def save_to_disk(self):
        """Converts the ROS grid into a PNG image and YAML metadata file."""
        # 1. Convert ROS Grid values to standard Image Grayscale values (0-255)
        img = np.zeros_like(self.grid, dtype=np.uint8)
        img[self.grid == 100] = 0    # Obstacles -> Black
        img[self.grid == 0] = 255    # Path -> White
        img[self.grid == -1] = 205   # Unknown -> Grey
        
        # ROS origin is bottom-left, but images draw from top-left. We flip it.
        img = cv2.flip(img, 0)
        
        # 2. Save PNG
        png_path = os.path.join(self.map_save_dir, 'swept_path_map.png')
        cv2.imwrite(png_path, img)
        
        # 3. Save YAML Metadata
        yaml_path = os.path.join(self.map_save_dir, 'swept_path_map.yaml')
        yaml_data = {
            'image': 'swept_path_map.png',
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False)
            
        self.get_logger().info(f"SUCCESS: Map saved to {self.map_save_dir}")

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        msg.data = self.grid.flatten().tolist()
        
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SweptPathMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()