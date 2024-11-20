import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import matplotlib.pyplot as plt
import numpy as np
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
    max_steering_angle_left = front_left_wheel.max_steer_angle * np.pi / 180
    max_steering_angle_right = front_right_wheel.max_steer_angle * np.pi / 180

    # Calculate the turning radius using the Ackermann steering geometry
    # Use the average of the two front steering angles or the more extreme value (inner wheel)
    avg_steering_angle = (max_steering_angle_left + max_steering_angle_right) / 2

    min_turning_radius = wheel_base / np.sin(max_steering_angle_left)
    # Aproximação: overhang dianteiro = overhang traseiro
    overhang = (length-wheel_base)/2
    print(wheel_base, min_turning_radius, width, length, overhang)
    return wheel_base, min_turning_radius, width, length, overhang


# Vehicle parameters
vehicle = get_vehicle()
wheel_base, R_min, width, length, overh = get_vehicle_attributes(vehicle)
Rc = R_min - width/2
Rb = np.sqrt(np.power(R_min+width/2, 2) + np.power(length-overh, 2))
L_min = overh + np.sqrt(Rb**2 - Rc**2)

# Spot parameters
L_available = 9.0

goal = np.array([190.2, -252.0])
launch_coord = np.array([goal[0]+R_min/np.sqrt(3), goal[1]-R_min*np.sqrt(3)])
print(launch_coord)
transform = carla.Transform(carla.Location(launch_coord[0], -launch_coord[1], 0.2), carla.Rotation(0, 90, 0))
if vehicle.get_transform().location.distance(transform.location) > 0.5:
    vehicle.set_transform(transform)
vehicle.set_target_velocity(carla.Vector3D(0.,0.,0.))

flag = 0 # Estacionamento finalizado
case = 0 # Debugar

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

class ParallelParkingNode(Node):
    def __init__(self):
        super().__init__('parallel_parking')
        self.parking_trigger = False
        self.reverse = self.steer = self.vel = self.brake = None
        self.parking_sub = self.create_subscription(Bool, '/start_parking', self.trigger_callback, 10)
        self.prev_vehicle_pos = self.current_vehicle_pos = launch_coord

        self.init_plot()

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

    def init_plot(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Enable interactive mode
        self.line, = self.ax.plot([], [], 'bo-', label='Car Path')  # Line representing car path
        self.line2, = self.ax.plot([], [], 'go-', label='Rear Path')  # Line representing car path
        self.target, = self.ax.plot([], [], 'rx', label='Goal')  # Point representing goal
        self.circle1 = None
        self.circle2 = None
        self.transition = None
        # self.ax.set_xlim(-, 10)
        # self.ax.set_ylim(-10, 10)
        self.ax.invert_xaxis()  # If x-axis increases to the left
        self.ax.invert_yaxis()  # If y-axis increases downwards
        self.ax.set_aspect('equal', 'box')
        self.ax.legend()

        self.x_data, self.y_data = [], []  # To store the car's positions
        self.rear_trajectory_x, self.rear_trajectory_y = [], [] 
        self.update_plot()

    def update_plot(self):
        """Update the Matplotlib plot with the car's latest position."""
        # for pos in zip([189.2+2.3/2]*2, [-254+4.5, -254-4.5]):
        #     self.target, = self.ax.plot([pos[0]], [pos[1]], 'gx')  # Point representing goal
        center1, center2, transition = self.park_points(R_min, goal)

        # Plot circle 1 (center1)
        self.circle1 = plt.Circle((center1[0], center1[1]), R_min, color='g', fill=False, linestyle='--', label='Circle 1')
        self.ax.add_artist(self.circle1)

        # Plot circle 2 (center2)
        self.circle2 = plt.Circle((center2[0], center2[1]), R_min, color='b', fill=False, linestyle='--', label='Circle 2')
        self.ax.add_artist(self.circle2)

        # Calculate the offset center of mass path (approximate)
        R_com1 = R_min + (wheel_base / 2)  # Offset path for center of mass
        R_com2 = R_min + (wheel_base / 2)  # Same for the second circle
        print("R_com, wheel_base", R_com1, wheel_base)
        
        # Plot the estimated CoM path (approximate as larger circles)
        # self.com_circle1 = plt.Circle((center1[0], center1[1]), R_com1, color='orange', fill=False, linestyle='--', label='CoM Path 1')
        # self.ax.add_artist(self.com_circle1)
        
        # self.com_circle2 = plt.Circle((center2[0], center2[1]), R_com2, color='orange', fill=False, linestyle='--', label='CoM Path 2')
        # self.ax.add_artist(self.com_circle2)

        # Plot transition point
        self.transition, = self.ax.plot(transition[0], transition[1], 'go', label='Transition Point')

        # Plot goal position (just to ensure it's visualized)
        self.target.set_xdata([goal[0]])
        self.target.set_ydata([goal[1]])
        self.line.set_xdata(self.x_data)  # Update the car's path on x-axis
        self.line.set_ydata(self.y_data)  # Update the car's path on y-axis
        self.line2.set_xdata(self.rear_trajectory_x)  # Update the car's path on x-axis
        self.line2.set_ydata(self.rear_trajectory_y)  # Update the car's path on y-axis

        # Redraw the updated plot
        self.ax.relim()  # Recompute the limits of the plot
        self.ax.autoscale_view()  # Automatically adjust the view
        plt.title(f"Case: {case}")
        plt.draw()  # Draw the updated plot
        plt.pause(0.001)  # Pause briefly to allow the plot to update without blocking
        plt.savefig('trajetoria.png')
    
    
    def trigger_callback(self, msg):
        # Start parking if True is received
        self.parking_trigger = msg.data

    def park_points(self, R, goal):
        c1 = np.array([goal[0], goal[1]-R*np.sqrt(3)])

        c2 = np.array([goal[0]+R, goal[1]])
        pt = np.array([goal[0]+R/2, goal[1]-R/2*np.sqrt(3)])

        # Return the radii and centers
        # self.plot_points(R, c1, c2, pt, goal)
        return c1, c2, pt


    def calc_distance(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def veh_control(self, initial_pos, transition_pos, goal_pos, update_pos):
        global flag, case
        self.reverse = False

        theta = estimate_heading(self.prev_vehicle_pos, self.current_vehicle_pos)
        self.global_point = transform_local_to_global([overh, 0], self.current_vehicle_pos, theta)
        self.rear_trajectory_x.append(self.global_point[0])
        self.rear_trajectory_y.append(self.global_point[1])

        # if update_pos[1] < launch_coord[1]:  # Move backward until start_pos
        #     case = 1
        #     self.vel = 0.1  # A small positive velocity
        #     self.steer = 0.0
        #     self.reverse = True  # Enable reverse motion
        #     self.brake = 0.0
        # elif launch_coord[1] <= update_pos[1] < transition_pos[1] or transition_pos[0] < update_pos[0] :  # First turn
        #     case = 2
        #     self.vel = 0.4  # Adjust velocity for reverse turning
        #     self.steer = 0.8  # Steering for reverse
        #     self.reverse = True  # Continue reversing
        #     self.brake = 0.0
        # elif flag == 0 and transition_pos[0] >= update_pos[0] > goal_pos[0]:  # Second turn
        #     case = 3
        #     self.steer = -0.8
        #     self.vel = 0.4  # Same velocity during this phase
        #     self.reverse = True  # Continue reversing
        #     self.brake = 0.0
        # elif update_pos[1] > goal_pos[1]:  # Finish parking, move forward
        #     case = 4
        #     self.steer = 0.0
        #     self.vel = 0.1  # Move forward
        #     self.reverse = False  # Disable reverse motion
        #     self.brake = 0.0
        #     self.flag = 1  # Mark flag to indicate parking is complete
        # elif flag == 1:  # After parking is complete, make adjustments
        #     case = 5
        #     self.steer = 0.0
        #     self.vel = 0.0  # Adjust forward slightly
        #     self.reverse = True  # Optional: fine-tune backward if needed
        #     self.brake = 1.0
        vec1 = update_pos - transition_pos
        vec2 = update_pos - goal_pos
        if update_pos[1] < launch_coord[1]:  # Move backward until start_pos
            case = 1
            self.vel = 0.1  # A small positive velocity
            self.steer = 0.0
            self.reverse = True  # Enable reverse motion
            self.brake = 0.0
        elif launch_coord[1] <= update_pos[1] and abs(vec1[1]/vec1[0]) >= 1.5:  # First turn
            case = 2
            self.vel = 0.4  # Adjust velocity for reverse turning
            self.steer = 1.0  # Steering for reverse
            self.reverse = True  # Continue reversing
            self.brake = 0.0
        elif flag == 0 and abs(vec1[1]/vec1[0]) < 1.5 and vec2[0] >= -0.2:  # Second turn
            case = 3
            self.steer = -1.0
            self.vel = 0.4  # Same velocity during this phase
            self.reverse = True  # Continue reversing
            self.brake = 0.0
        elif update_pos[1] > transition_pos[1] and vec2[0] < -0.2:  # Finish parking, move forward
            case = 4
            self.steer = 0.0
            self.vel = 0.1  # Move forward
            self.reverse = False  # Disable reverse motion
            self.brake = 0.0
            self.flag = 0  # Mark flag to indicate parking is complete
        elif flag == 1:  # After parking is complete, make adjustments
            case = 5
            self.steer = 0.0
            self.vel = 0.0  # Adjust forward slightly
            self.reverse = True  # Optional: fine-tune backward if needed
            self.brake = 1.0
        else:
            case = 8
        print("vec2", vec2)
        # print('cos', abs(vec[1]/vec[0]))

    def veh_mission(self, msg):
        global flag, case

        # if self.parking_trigger:
        if True:
            # Get current vehicle position from odometry
            current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

            # Calculate parking points
            center1, center2, transition = self.park_points(R_min, goal)
            # Calculate velocity and steering commands
            self.veh_control(launch_coord, transition, goal, current_pos)
            
            self.get_logger().info(f'Current Pos: {current_pos}')
            self.get_logger().info(f'Case: {case}')
            # self.get_logger().info(f'r_star, center1, center2, initial, start, transition: {r_star}, {center1}, {center2}, {initial}, {start}, {transition}')
            self.get_logger().info(f'vel_cmd, steer_cmd, reverse_cmd, flag: {self.vel}, {self.steer}, {self.reverse}, {flag}')

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = self.vel # Always positive velocity
            control_msg.steer = self.steer
            control_msg.reverse = self.reverse # Set reverse mode
            control_msg.brake = self.brake

            self.control_publisher.publish(control_msg)

            self.x_data.append(current_pos[0])
            self.y_data.append(current_pos[1])
            self.prev_vehicle_pos = self.current_vehicle_pos
            self.current_vehicle_pos = current_pos
            self.update_plot()
            pass

def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParkingNode()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



