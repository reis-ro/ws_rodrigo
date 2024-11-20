import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl

# Vehicle and parking parameters
length = 4.88
wheel_base = 2.85
overh = (length - wheel_base) / 2
width = 1.910
beta_max = 0.6
R_min = 5
Ri_min = np.sqrt(R_min**2 - wheel_base**2) - (width / 2)
Re_min = np.sqrt((Ri_min + width)**2 + (wheel_base + overh)**2)
L_min = overh + np.sqrt(Re_min**2 - Ri_min**2)
flag = 0

class ParallelParkingNode(Node):
    def __init__(self):
        super().__init__('parallel_parking')

        # Define parking space availability (could come from a perception sensor in future)
        L_available = 7

        if L_available >= L_min:
            # Subscribe to odometry (CARLA provides odometry data)
            self.subscription = self.create_subscription(
                Odometry,
                '/carla/ego_vehicle/odometry',  # CARLA odometry topic
                self.veh_mission,
                10
            )

            # Publisher for vehicle control (steering, throttle, brake)
            self.control_publisher = self.create_publisher(
                CarlaEgoVehicleControl,
                '/carla/ego_vehicle/vehicle_control_cmd',  # CARLA control topic
                10
            )
        else:
            self.get_logger().info("Not enough space to park!")

    def park_points(self, ri, current, goal, w):
        r_prime = ri + w / 2
        c1 = np.array([goal[0], goal[1] + r_prime])
        x_c1 = c1[0]
        y_c1 = c1[1]
        x_i = current[0]
        y_i = current[1]
        y_s = y_i
        y_c2 = y_s - r_prime
        y_t = (y_c1 + y_c2) / 2
        x_t = x_c1 + np.sqrt(r_prime**2 - (y_t - y_c1)**2)
        x_s = 2 * x_t - x_c1
        x_c2 = x_s

        c2 = np.array([x_c2, y_c2])
        i = np.array([x_i, y_i])
        s = np.array([x_s, y_s])
        pt = np.array([x_t, y_t])
        return r_prime, c1, c2, i, s, pt

    def calc_distance(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def veh_control(self, initial_pos, start_pos, transition_pos, goal_pos, radius, update_pos):
        global flag

        if update_pos[0] > start_pos[0]:
            vel = -12
            steer = 0
        elif start_pos[0] >= update_pos[0] > transition_pos[0]:
            vel = -104
            steer = -0.49
        elif transition_pos[0] >= update_pos[0] > goal_pos[0] - 0.5 and flag == 0:
            steer = 0.49
            vel = -104
        elif update_pos[0] <= goal_pos[0] - 0.5:
            steer = 0
            vel = 20
            flag = 1
        elif flag == 1:
            steer = 0
            vel = -5

        return vel, steer

    def veh_mission(self, msg):
        # Get current vehicle position from odometry
        current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        # Define goal and launch coordinates
        first_obstacle = np.array([20 - (length / 2 + 7), 20 - (width / 2 + 0.5)])
        goal = np.array([first_obstacle[0], first_obstacle[1] - width / 2])
        launch_coord = np.array([20, 20])

        # Calculate parking points
        r_star, center1, center2, initial, start, transition = self.park_points(Ri_min, launch_coord, goal, width)

        # Calculate velocity and steering commands
        vel_cmd, steer_cmd = self.veh_control(initial, start, transition, goal, r_star, current_pos)

        # Publish vehicle control commands
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = max(0.0, vel_cmd / 100.0)
        control_msg.brake = max(0.0, -vel_cmd / 100.0)
        control_msg.steer = steer_cmd

        self.control_publisher.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParkingNode()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

