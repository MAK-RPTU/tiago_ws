#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import datetime
import os

class AmclPoseSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_pose_subscriber')
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(script_dir, "data_logger", "robot_pose.txt")

        # Subscribe to the amcl_pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10  # QoS profile depth
        )
        
        self.get_logger().info('Subscribed to /amcl_pose')

    def pose_callback(self, msg):
        # Extract position and orientation
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Format the data
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        pose_data = (
            # f"[{timestamp}]\n"
            f"Position: x={position.x}, y={position.y}, z={position.z}\n"
            f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}\n"
        )

        # Append the data to a file (without overwriting)
        with open(self.file_path, "a") as file:
            file.write(pose_data)

        # Log the data in the terminal
        self.get_logger().info(f'Pose saved: {pose_data.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
