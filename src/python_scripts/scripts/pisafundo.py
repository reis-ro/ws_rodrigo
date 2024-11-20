import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl  # Import the message type

class ThrottleMultiplierNode(Node):
    def __init__(self):
        super().__init__('throttle_multiplier_node')
        
        # Create a subscriber to /carla/ego_vehicle/vehicle_control_cmd_hue
        self.subscription = self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd_hue',
            self.cmd_hue_callback,
            10  # QoS profile depth
        )
        
        # Create a publisher to /carla/ego_vehicle/vehicle_control_cmd
        self.publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            10
        )

    def cmd_hue_callback(self, msg):
        # Multiply the throttle value by 10
        msg.throttle *= 10
        
        # Publish the modified message
        self.publisher.publish(msg)
        self.get_logger().info('Throttle: {}'.format(msg.throttle))

def main(args=None):
    rclpy.init(args=args)

    throttle_multiplier_node = ThrottleMultiplierNode()

    rclpy.spin(throttle_multiplier_node)

    throttle_multiplier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
