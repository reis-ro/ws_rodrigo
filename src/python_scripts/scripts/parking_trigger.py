import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class ParkingTriggerNode(Node):
    def __init__(self):
        super().__init__('parking_trigger_node')

        # Subscription to parking trigger topic
        self.parking_trigger_sub = self.create_subscription(
            Bool,
            '/parking_trigger',
            self.trigger_callback,
            10
        )

        # Publisher for goal pose (Nav2 action)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Parking Trigger Node has been started.')

    def trigger_callback(self, msg):
        if msg.data:  # If the message is TRUE
            self.get_logger().info('Parking Triggered!')

            # Define the parking spot goal
            parking_goal = PoseStamped()
            parking_goal.header.frame_id = 'map'  # Assuming using the 'map' frame
            parking_goal.header.stamp = self.get_clock().now().to_msg()

            # Set the parking spot location (adjust these values based on your parking spot)
            parking_goal.pose.position.x = 190.0  # Example X position
            parking_goal.pose.position.y = -273.0  # Example Y position
            parking_goal.pose.orientation.z = -0.7  # Orientation (facing forward)
            parking_goal.pose.orientation.w = 0.7  # Orientation (facing forward)

            # Publish the goal to the Nav2 stack
            self.goal_pub.publish(parking_goal)

            self.get_logger().info('Parking goal sent to Nav2.')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingTriggerNode()

    # Spin the node to keep it active and listen for messages
    rclpy.spin(node)

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

