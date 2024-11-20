import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
from nav_msgs.msg import Odometry, Path
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import time
import carla

def get_vehicle():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    car_id = 135 
    vehicle = world.get_actor(car_id)  # Replace <vehicle_id> with your vehicle's actor ID
    return vehicle

def get_vehicle_attributes(vehicle):
    bounding_box = vehicle.bounding_box.extent
    length, width = bounding_box.x*2, bounding_box.y*2 # Multiply by 2 for full width
    # Get physics control to retrieve dynamic attributes
    physics_control = vehicle.get_physics_control()

    # Extract the wheelbase (distance between front and rear axles)
    wheels = physics_control.wheels
    front_left_wheel = wheels[0]
    front_right_wheel = wheels[1]
    rear_left_wheel = wheels[2]
    wheel_base = front_left_wheel.position.distance(rear_left_wheel.position)/100
    # Maximum steering angle (in radians) from both front wheels
    # max_steering_angle_left = front_left_wheel.max_steer_angle * np.pi / 180
    # max_steering_angle_right = front_right_wheel.max_steer_angle * np.pi / 180
    max_steering_angle_left = 70 * np.pi / 180
    max_steering_angle_right = 47.95 * np.pi / 180
    # Calculate the turning radius using the Ackermann steering geometry
    # Use the average of the two front steering angles or the more extreme value (inner wheel)
    avg_steering_angle = (max_steering_angle_left + max_steering_angle_right) / 2
    
    min_turning_radius = wheel_base / np.sin(avg_steering_angle)
    # Aproximação: overhang dianteiro = overhang traseiro
    overhang = (length-wheel_base)/2
    print(wheel_base, min_turning_radius, width, length, overhang)
    return wheel_base, min_turning_radius, avg_steering_angle, width, length, overhang

def estimate_heading(prev_pos, current_pos):
    """
    Estimate the heading (orientation angle theta) based on two consecutive positions.
    """
    delta_x = current_pos[0] - prev_pos[0]
    delta_y = current_pos[1] - prev_pos[1]
    
    # Use atan2 to compute the heading angle
    theta = np.arctan2(delta_y, delta_x)
    return theta

def transform_local_to_global(local_point, vehicle_pos, theta):
    """
    Transforms a point from the vehicle's local frame to the global frame using
    estimated heading (theta) and vehicle position in the global frame.
    """
    # Rotation matrix based on the estimated heading theta
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Perform the transformation
    global_point = np.dot(rotation_matrix, local_point) + vehicle_pos
    return global_point


# Vehicle parameters
vehicle = get_vehicle()
wheel_base, R_min, beta, width, length, overh = get_vehicle_attributes(vehicle)
Ri = wheel_base/np.tan(beta) - width/2
Re = np.sqrt(np.power(Ri+width, 2) + np.power(wheel_base+overh, 2))
L_min = overh + np.sqrt(Re**2 - Ri**2)

# Spot parameters
spot_width, spot_length = 2.3, 7.0
spot_center = np.array([189.2, -254])
goal = np.array([spot_center[0]+0.7, spot_center[1]+spot_length/2-overh])

# "Arbitrary" - to some extent
# launch_coord = np.array([goal[0]+R_min, goal[1]-R_min*np.sqrt(3)-overh])
launch_coord = np.array([goal[0]+width*1.3, spot_center[1]-spot_length/2-length/2])

print(launch_coord)
transform = carla.Transform(carla.Location(launch_coord[0], -launch_coord[1], 0.2), carla.Rotation(0, 90, 0))
if vehicle.get_transform().location.distance(transform.location) > 0.5:
    vehicle.set_transform(transform)
vehicle.set_target_velocity(carla.Vector3D(0.,0.,0.))

flag = 0 # Estacionamento finalizado
case = 0 # Debugar
k = 0

def discretize_arc(center, start_point, end_point, R, num_points):
    """
    Discretize the arc between two points on a circle.
    :param center: Center of the circle as a numpy array [x, y].
    :param start_point: Starting point on the arc as a numpy array [x, y].
    :param end_point: Ending point on the arc as a numpy array [x, y].
    :param R: Radius of the circle.
    :param num_points: Number of points to discretize the arc into.
    :return: A list of points along the arc.
    """
    path_points = []
    
    # Calculate the start and end angles
    start_angle = np.arctan2(start_point[1] - center[1], start_point[0] - center[0])
    end_angle = np.arctan2(end_point[1] - center[1], end_point[0] - center[0])
    print(start_angle*180/np.pi, end_angle*180/np.pi)

    # Adjust the end angle if it's smaller than the start angle (to keep it continuous)
    if end_angle < start_angle:
        end_angle += 2 * np.pi

    # Discretize the arc by angle
    angles = np.linspace(start_angle, end_angle, num_points)

    for theta in angles:
        x = center[0] + R * np.cos(theta)
        y = center[1] + R * np.sin(theta)
        path_points.append(np.array([x, y]))

    return path_points



class ParallelParkingNode(Node):
    def __init__(self):
        super().__init__('parallel_parking')
        self.parking_trigger = False
        self.reverse = self.steer = self.vel = self.brake = None
        self.parking_sub = self.create_subscription(Bool, '/start_parking', self.trigger_callback, 10)
        while vehicle.get_transform().location.distance(transform.location) > 0.2:
            pass
        self.prev_vehicle_pos = self.current_vehicle_pos = launch_coord

        self._park_points(R_min, goal, launch_coord)
        self._init_plot()

        if spot_length >= L_min:
            # Subscribe to odometry (CARLA provides odometry data)
            # self.subscription = self.create_subscription(
            #     Odometry,
            #     '/carla/ego_vehicle/odometry',  # CARLA odometry topic
            #     self.veh_mission,
            #     10
            # )

            # Publisher for vehicle control (steering, throttle, brake)
            # self.control_publisher = self.create_publisher(
            #     CarlaEgoVehicleControl,
            #     '/carla/ego_vehicle/vehicle_control_cmd',  # CARLA control topic
            #     10
            # )

            self.path_publisher = self.create_publisher(Path, '/Path', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.publish_path)

            self.path = Path()
            self.path.header.frame_id = 'map'
            for point in zip(self.discretized_circle_x, self.discretized_circle_y):
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.orientation.w = 1.0
                self.path.poses.append(pose)
        else:
            self.get_logger().info("Not enough space to park!")

    def publish_path(self):
        # Update timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Publish the path
        self.path_publisher.publish(self.path)
        self.get_logger().info('Publishing path to /plan')

    def trigger_callback(self, msg):
        # Start parking if True is received
        self.parking_trigger = msg.data

    def _init_plot(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Enable interactive mode
        self.line, = self.ax.plot([], [], 'bo-', label='Car Path')  # Line representing car path
        self.line2, = self.ax.plot([], [], 'ro-', label='Rear Path')  # Line representing car path
        self.line3, = self.ax.plot([], [], 'go-', label='Discretized Path')  # Line representing car path
        self.target, = self.ax.plot([], [], 'rx', label='Goal')  # Point representing goal
        self.circle1 = None
        self.circle2 = None
        self.transition = None
        self.ax.invert_xaxis()  # If x-axis increases to the left
        self.ax.invert_yaxis()  # If y-axis increases downwards
        self.ax.set_aspect('equal', 'box')
        self.ax.legend()

        self.x_data, self.y_data = [], []  # To store the car's positions
        self.rear_trajectory_x, self.rear_trajectory_y = [], [] 
        
        self.update_plot()

    def _park_points(self, R, goal, launch_coord):
        # self.c1 = np.array([launch_coord[0]-R, 3*goal[1]-2*np.sqrt(R**2-((launch_coord[0]-goal[0])/2)**2)])
        self.c2 = np.array([goal[0]+R, goal[1]])

        xt = (launch_coord[0]+goal[0])/2
        yt = self.c2[1] - np.sqrt(R**2-(xt-self.c2[0])**2)
        self.transition_pt = np.array([xt, yt])

        self.c1 = np.array([launch_coord[0]-R, 2*self.transition_pt[1]-self.c2[1]])
        self.get_logger().info(f'C1: {self.c1}, C2: {self.c2}, Transition: {self.transition_pt}')

        # Number of points to discretize each arc
        num_points = 200

        # Discretize the two arcs
        arc1 = discretize_arc(self.c1, launch_coord, self.transition_pt, R, num_points)
        arc2 = discretize_arc(self.c2, goal, self.transition_pt, R, num_points)

        # arc1 = discretize_arc(c1, launch_coord+np.array([0, overh]), transition_pt, R, num_points)
        # arc2 = discretize_arc(c2, goal, transition_pt, R, num_points)

        # # Combine the two arcs into a full path
        full_path = arc1 + arc2
        self.discretized_circle_x, self.discretized_circle_y = [x[0] for x in full_path], [x[1] for x in full_path]
        # plt.scatter([x[0] for x in full_path], [x[1] for x in full_path])
        # plt.plot(goal[0], goal[1], 'rx')
        # plt.plot(launch_coord[0], launch_coord[1]+overh, 'gx')
        # plt.show()


    def update_plot(self):
        """Update the Matplotlib plot with the car's latest position."""
        # center1, center2, transition = self.c1, self.c2, self.transition

        self.circle1 = plt.Circle((self.c1[0], self.c1[1]), R_min, color='g', fill=False, linestyle='--', label='Circle 1')
        self.ax.add_artist(self.circle1)

        self.circle2 = plt.Circle((self.c2[0], self.c2[1]), R_min, color='b', fill=False, linestyle='--', label='Circle 2')
        self.ax.add_artist(self.circle2)

        self.transition, = self.ax.plot(self.transition_pt[0], self.transition_pt[1], 'go', label='Transition Point')

        # Plot goal position (just to ensure it's visualized)
        self.target.set_xdata([goal[0]])
        self.target.set_ydata([goal[1]])
        self.line.set_xdata(self.x_data)  # Update the car's path on x-axis
        self.line.set_ydata(self.y_data)  # Update the car's path on y-axis
        self.line2.set_xdata(self.rear_trajectory_x)  # Update the car's path on x-axis
        self.line2.set_ydata(self.rear_trajectory_y)  # Update the car's path on y-axis
        self.line3.set_xdata(self.discretized_circle_x)  # Update the car's path on x-axis
        self.line3.set_ydata(self.discretized_circle_y)  # Update the car's path on y-axis

        # Redraw the updated plot
        self.ax.relim()  # Recompute the limits of the plot
        self.ax.autoscale_view()  # Automatically adjust the view
        plt.title(f"Case: {case}")
        plt.draw()  # Draw the updated plot
        plt.pause(0.001)  # Pause briefly to allow the plot to update without blocking
        plt.savefig('trajetoria.png')
    
    def veh_control(self, initial_pos, goal_pos, update_pos):
        global flag, k
        steering = 0.7  # Constant steering angle
        base_velocity = 0.3  # Base velocity for regular movement
        slow_velocity = 0.2  # Slower velocity for precise maneuvering near key points
        goal_threshold = 0.4  # Distance threshold for stopping near the goal
        transition_threshold = 0.5  # Distance threshold for triggering transitions
        

        # Directional control to track whether the vehicle has passed the transition point
        past_transition = bool(update_pos[1] > self.transition_pt[1]-0.3)  # If vehicle's y is below the transition point's y

        # Calculate distances to transition point and goal
        dist_to_transition = np.linalg.norm(update_pos - self.transition_pt)
        dist_to_goal = abs(update_pos[1] - goal_pos[1])

        # Adjust velocity dynamically based on proximity to important points
        def dynamic_velocity(distance, threshold, high_speed, low_speed):
            """ Reduce speed when near the target to allow for precise control. """
            if distance < threshold:
                return low_speed, 0.0
            return high_speed, 0.0

        # ** Case 1: Move backward along the first circular path until passing the transition point **
        if not past_transition:
            case = 1
            self.vel, self.brake = dynamic_velocity(dist_to_transition, transition_threshold, base_velocity, slow_velocity)
            self.steer = steering  # Constant steering for the first circular path
            self.reverse = True  # Move in reverse

        # ** Case 2: After passing the transition point, switch steering for the second circular path **
        elif past_transition and dist_to_goal > goal_threshold:
            case = 2
            if k < 10: 
                self.vel = 0.0
                self.brake = 1.0
                self.steer = 0.0
                self.reverse = True
                k += 1
            else:
                self.vel, self.brake = dynamic_velocity(dist_to_goal, goal_threshold, base_velocity, slow_velocity)
                self.steer = -steering  # Switch to opposite steering for the second curve
                self.reverse = True  # Continue reversing

        # ** Case 3: Final adjustment and stopping near the parking goal **
        elif dist_to_goal <= goal_threshold:
            case = 3
            self.vel = 0.0  # Stop the vehicle
            self.steer = 0.0  # No steering needed once parked
            self.reverse = False
            self.brake = 1.0  # Apply the brake to stop completely
            flag = 1  # Mark parking as complete

        else:
            case = 0  # Default case (no action)
        self.get_logger().info(f'Case: {case}, Flag: {flag}')


    def veh_mission(self, msg):
        # if self.parking_trigger:
        if True:
            # Get current vehicle position from odometry
            current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

            # Eventually get the estimate from calculation
            # theta = estimate_heading(self.prev_vehicle_pos, self.current_vehicle_pos) 
            theta = -vehicle.get_transform().rotation.yaw * np.pi/180
            self.rear_axle = transform_local_to_global([-overh, 0], self.current_vehicle_pos, theta)

            # Calculate parking points
            self.veh_control(launch_coord, goal, self.rear_axle)
            
            self.get_logger().info(f'Current Pos: {current_pos}')
            self.get_logger().info(f'vel_cmd, steer_cmd, reverse_cmd, flag: {self.vel}, {self.steer}, {self.reverse}')
            # self.get_logger().info(f'Theta: {theta}, Rear Axle:, {self.rear_axle}')

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = self.vel # Always positive velocity
            control_msg.steer = self.steer
            control_msg.reverse = bool(self.reverse) # Set reverse mode
            control_msg.brake = self.brake

            # self.control_publisher.publish(control_msg)

            # Update plot
            self.rear_trajectory_x.append(self.rear_axle[0])
            self.rear_trajectory_y.append(self.rear_axle[1])
            self.x_data.append(current_pos[0])
            self.y_data.append(current_pos[1])
            self.update_plot()
            
            # Fix position for heading estimation
            self.prev_vehicle_pos = self.current_vehicle_pos
            self.current_vehicle_pos = current_pos
            
            pass

def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParkingNode()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



