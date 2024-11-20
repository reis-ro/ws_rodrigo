# parallel_parking_service.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class ParallelParkingService(Node):
    def __init__(self):
        super().__init__('parallel_parking_service')
        self.srv = self.create_service(Trigger, 'generate_parking_path', self.generate_path_callback)
        self.get_logger().info('Parallel parking service ready.')

    def generate_path_callback(self, request, response):
        # Here we will use the logic you already have for generating the path
        path = self.generate_parking_path()
        self.publish_path(path)
        response.success = True
        response.message = "Path generated and published"
        return response

    def generate_parking_path(self):
        # Simulated path generation, you can include your full logic here
        path = Path()
        path.header.frame_id = 'map'

        # Add poses to the path (you should use your actual discretization logic here)
        for i in range(10):
            pose = PoseStamped()
            pose.pose.position.x = i * 0.5  # Example positions
            pose.pose.position.y = i * 0.2  # Example positions
            path.poses.append(pose)

        return path

    def publish_path(self, path):
        # Create a publisher to publish the generated path
        path_publisher = self.create_publisher(Path, '/generated_parking_path', 10)
        path.header.stamp = self.get_clock().now().to_msg()
        path_publisher.publish(path)
        self.get_logger().info('Publishing generated parking path.')
        

def main(args=None):
    rclpy.init(args=args)
    node = ParallelParkingService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
