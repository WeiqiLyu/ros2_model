import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Reduced queue size to 1 to prevent RViz from getting backlogged and flickering
        self.pubs = {
            'tug/base_link': self.create_publisher(Path, '/tug/path', 1),
            'aircraft/nose_wheel_link': self.create_publisher(Path, '/aircraft/nose_wheel_path', 1),
            'aircraft/left_main_gear_link': self.create_publisher(Path, '/aircraft/left_gear_path', 1),
            'aircraft/right_main_gear_link': self.create_publisher(Path, '/aircraft/right_gear_path', 1)
        }
        
        self.paths = {frame: Path() for frame in self.pubs.keys()}
        for path in self.paths.values():
            path.header.frame_id = 'world'
            
        # Running at 20Hz (0.05s) to perfectly catch the 10Hz TF updates without stuttering
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        for frame_id in self.pubs.keys():
            try:
                # Time() gets the absolute most recent transform instantly
                trans = self.tf_buffer.lookup_transform('world', frame_id, rclpy.time.Time())
                
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                
                current_path = self.paths[frame_id]
                
                if len(current_path.poses) == 0:
                    self.add_pose(current_path, x, y, now)
                else:
                    last_pose = current_path.poses[-1].pose.position
                    distance = math.sqrt((x - last_pose.x)**2 + (y - last_pose.y)**2)
                    
                    if distance > 2.0:
                        # Teleportation detected (animation looped) -> Wipe the slate clean
                        current_path.poses.clear()
                        self.add_pose(current_path, x, y, now)
                    elif distance > 0.05:
                        # Moved a tiny bit -> Draw the next point
                        self.add_pose(current_path, x, y, now)
                        
                current_path.header.stamp = now
                self.pubs[frame_id].publish(current_path)
                    
            except Exception:
                pass # Silently wait if TF tree is booting up

    def add_pose(self, path, x, y, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        path.poses.append(pose)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()