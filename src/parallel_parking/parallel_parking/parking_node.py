# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# import numpy as np
# from nav_msgs.msg import Odometry
# from carla_msgs.msg import CarlaEgoVehicleControl
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import numpy as np
# import time
# import carla

# def get_vehicle():
#     client = carla.Client('localhost', 2000)
#     world = client.get_world()
#     car_id = 135
#     vehicle = world.get_actor(car_id)  # Replace <vehicle_id> with your vehicle's actor ID
#     return vehicle

# def get_vehicle_attributes(vehicle):
#     bounding_box = vehicle.bounding_box.extent
#     length, width = bounding_box.x*2, bounding_box.y*2 # Multiply by 2 for full width
#     # Get physics control to retrieve dynamic attributes
#     physics_control = vehicle.get_physics_control()

#     # Extract the wheelbase (distance between front and rear axles)
#     wheels = physics_control.wheels
#     front_left_wheel = wheels[0]
#     front_right_wheel = wheels[1]
#     rear_left_wheel = wheels[2]
#     wheel_base = front_left_wheel.position.distance(rear_left_wheel.position)/100
#     # Maximum steering angle (in radians) from both front wheels
#     # max_steering_angle_left = front_left_wheel.max_steer_angle * np.pi / 180
#     # max_steering_angle_right = front_right_wheel.max_steer_angle * np.pi / 180
#     max_steering_angle_left = 70 * np.pi / 180
#     max_steering_angle_right = 47.95 * np.pi / 180
#     # Calculate the turning radius using the Ackermann steering geometry
#     # Use the average of the two front steering angles or the more extreme value (inner wheel)
#     avg_steering_angle = (max_steering_angle_left + max_steering_angle_right) / 2
    
#     min_turning_radius = wheel_base / np.sin(avg_steering_angle)
#     # Aproximação: overhang dianteiro = overhang traseiro
#     overhang = (length-wheel_base)/2
#     print(wheel_base, min_turning_radius, width, length, overhang)
#     return wheel_base, min_turning_radius, avg_steering_angle, width, length, overhang

# def estimate_heading(prev_pos, current_pos):
#     """
#     Estimate the heading (orientation angle theta) based on two consecutive positions.
#     """
#     delta_x = current_pos[0] - prev_pos[0]
#     delta_y = current_pos[1] - prev_pos[1]
    
#     # Use atan2 to compute the heading angle
#     theta = np.arctan2(delta_y, delta_x)
#     return theta

# def transform_local_to_global(local_point, vehicle_pos, theta):
#     """
#     Transforms a point from the vehicle's local frame to the global frame using
#     estimated heading (theta) and vehicle position in the global frame.
#     """
#     # Rotation matrix based on the estimated heading theta
#     rotation_matrix = np.array([
#         [np.cos(theta), -np.sin(theta)],
#         [np.sin(theta), np.cos(theta)]
#     ])
    
#     # Perform the transformation
#     global_point = np.dot(rotation_matrix, local_point) + vehicle_pos
#     return global_point


# # Vehicle parameters
# vehicle = get_vehicle()
# wheel_base, R_min, beta, width, length, overh = get_vehicle_attributes(vehicle)
# Ri = wheel_base/np.tan(beta) - width/2
# Re = np.sqrt(np.power(Ri+width, 2) + np.power(wheel_base+overh, 2))
# L_min = overh + np.sqrt(Re**2 - Ri**2)

# # Spot parameters
# spot_width, spot_length = 2.3, 7.0
# spot_center = np.array([189.2, -254])
# goal = np.array([spot_center[0], spot_center[1]+spot_length/2-overh])

# # "Arbitrary" - to some extent
# # launch_coord = np.array([goal[0]+R_min, goal[1]-R_min*np.sqrt(3)-overh])
# launch_coord = np.array([spot_center[0]+width*1.3, spot_center[1]-spot_length/2-overh+wheel_base/2])


# print(launch_coord)
# transform = carla.Transform(carla.Location(launch_coord[0]+0.6, -launch_coord[1]+wheel_base/2-16, 0.2), carla.Rotation(0, 90, 0))
# if vehicle.get_transform().location.distance(transform.location) > 0.5:
#     vehicle.set_transform(transform)
# vehicle.set_target_velocity(carla.Vector3D(0.,0.,0.))

# flag = 0 # Estacionamento finalizado
# case = 0 # Debugar



# class ParallelParkingNode(Node):
#     def __init__(self):
#         super().__init__('parallel_parking')
#         self.parking_trigger = False
#         self.reverse = self.steer = self.vel = self.brake = None
#         self.parking_sub = self.create_subscription(Bool, '/start_parking', self.trigger_callback, 10)
#         time.sleep(5)
#         self.prev_vehicle_pos = self.current_vehicle_pos = np.array([launch_coord[0], launch_coord[1]-wheel_base/2])

#         self._park_points(R_min, goal, launch_coord)
#         self._init_plot()

#         if spot_length >= L_min:
#             # Subscribe to odometry (CARLA provides odometry data)
#             self.subscription = self.create_subscription(
#                 Odometry,
#                 '/carla/ego_vehicle/odometry',  # CARLA odometry topic
#                 self.veh_mission,
#                 10
#             )

#             # Publisher for vehicle control (steering, throttle, brake)
#             self.control_publisher = self.create_publisher(
#                 CarlaEgoVehicleControl,
#                 '/carla/ego_vehicle/vehicle_control_cmd',  # CARLA control topic
#                 10
#             )
#         else:
#             self.get_logger().info("Not enough space to park!")

#     def trigger_callback(self, msg):
#         # Start parking if True is received
#         self.parking_trigger = msg.data

#     def _init_plot(self):
#         self.fig, self.ax = plt.subplots()
#         plt.ion()  # Enable interactive mode
#         self.line, = self.ax.plot([], [], 'bo-', label='Car Path')  # Line representing car path
#         self.line2, = self.ax.plot([], [], 'ro-', label='Rear Path')  # Line representing car path
#         self.target, = self.ax.plot([], [], 'rx', label='Goal')  # Point representing goal
#         self.circle1 = None
#         self.circle2 = None
#         self.transition = None
#         self.ax.invert_xaxis()  # If x-axis increases to the left
#         self.ax.invert_yaxis()  # If y-axis increases downwards
#         self.ax.set_aspect('equal', 'box')
#         self.ax.legend()

#         self.x_data, self.y_data = [], []  # To store the car's positions
#         self.rear_trajectory_x, self.rear_trajectory_y = [], [] 
#         self.update_plot()

#     def _park_points(self, R, goal, launch_coord):
#         # self.c1 = np.array([launch_coord[0]-R, 3*goal[1]-2*np.sqrt(R**2-((launch_coord[0]-goal[0])/2)**2)])
#         self.c2 = np.array([goal[0]+R, goal[1]])

#         xt = (launch_coord[0]+goal[0])/2
#         yt = self.c2[1] - np.sqrt(R**2-(xt-self.c2[0])**2)
#         self.transition_pt = np.array([xt, yt])

#         self.c1 = np.array([launch_coord[0]-R, 2*self.transition_pt[1]-self.c2[1]])
#         self.get_logger().info(f'C1: {self.c1}, C2: {self.c2}, Transition: {self.transition_pt}')


#     def update_plot(self):
#         """Update the Matplotlib plot with the car's latest position."""
#         # center1, center2, transition = self.c1, self.c2, self.transition

#         self.circle1 = plt.Circle((self.c1[0], self.c1[1]), R_min, color='g', fill=False, linestyle='--', label='Circle 1')
#         self.ax.add_artist(self.circle1)

#         self.circle2 = plt.Circle((self.c2[0], self.c2[1]), R_min, color='b', fill=False, linestyle='--', label='Circle 2')
#         self.ax.add_artist(self.circle2)

#         self.transition, = self.ax.plot(self.transition_pt[0], self.transition_pt[1], 'go', label='Transition Point')

#         # Plot goal position (just to ensure it's visualized)
#         self.target.set_xdata([goal[0]])
#         self.target.set_ydata([goal[1]])
#         self.line.set_xdata(self.x_data)  # Update the car's path on x-axis
#         self.line.set_ydata(self.y_data)  # Update the car's path on y-axis
#         self.line2.set_xdata(self.rear_trajectory_x)  # Update the car's path on x-axis
#         self.line2.set_ydata(self.rear_trajectory_y)  # Update the car's path on y-axis

#         # Redraw the updated plot
#         self.ax.relim()  # Recompute the limits of the plot
#         self.ax.autoscale_view()  # Automatically adjust the view
#         plt.title(f"Case: {case}")
#         plt.draw()  # Draw the updated plot
#         plt.pause(0.001)  # Pause briefly to allow the plot to update without blocking
#         plt.savefig('trajetoria.png')
    
#     def veh_control(self, initial_pos, goal_pos, update_pos):
#         global flag
#         self.reverse = False
#         angle = np.arcsin(wheel_base/R_min)*180/np.pi
#         # angle = 70
#         steering = 0.7
#         # if update_pos[1] < launch_coord[1]:  # Move backward until start_pos
#         #     case = 1
#         #     self.vel = 0.1  # A small positive velocity
#         #     self.steer = 0.0
#         #     self.reverse = True  # Enable reverse motion
#         #     self.brake = 0.0
#         if np.linalg.norm(update_pos-self.transition_pt) > 0.2 and self.transition_pt[1] >= update_pos[1]:
#             case = 2
#             self.vel = 0.3  # A small positive velocity
#             self.steer = steering
#             self.reverse = True  # Enable reverse motion
#             self.brake = 0.0
#         elif flag == 0 and self.transition_pt[1] - 0.1 <= update_pos[1] < goal_pos[1] - 0.1:
#             case = 3
#             self.vel = 0.3  # Adjust velocity for reverse turning
#             self.steer = -steering  # Steering for reverse
#             self.reverse = True  # Continue reversing
#             self.brake = 0.0
#         elif flag == 0 and update_pos[1] >= goal_pos[1] - 0.1:
#             case = 4
#             self.steer = 0.0
#             self.vel = 0.2  # Move forward
#             self.reverse = True  # Disable reverse motion
#             self.brake = 0.0
#             flag = 1  # Mark flag to indicate parking is complete
#         elif flag == 1:  # After parking is complete, make adjustments
#             case = 5
#             self.steer = 0.0
#             self.vel = 0.0  # Adjust forward slightly
#             self.reverse = True  # Optional: fine-tune backward if needed
#             self.brake = 1.0
#         else:
#             case = 8
#         self.get_logger().info(f'Case: {case}, Flag: {flag}')


#     def veh_mission(self, msg):
#         # if self.parking_trigger:
#         if True:
#             # Get current vehicle position from odometry
#             current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

#             # Eventually get the estimate from calculation
#             # theta = estimate_heading(self.prev_vehicle_pos, self.current_vehicle_pos) 
#             theta = -vehicle.get_transform().rotation.yaw * np.pi/180
#             self.rear_axle = transform_local_to_global([-wheel_base/2, 0], self.current_vehicle_pos, theta)

#             # Calculate parking points
#             self.veh_control(launch_coord, goal, self.rear_axle)
            
#             self.get_logger().info(f'Current Pos: {current_pos}')
#             self.get_logger().info(f'vel_cmd, steer_cmd, reverse_cmd, flag: {self.vel}, {self.steer}, {self.reverse}')
#             # self.get_logger().info(f'Theta: {theta}, Rear Axle:, {self.rear_axle}')

#             control_msg = CarlaEgoVehicleControl()
#             control_msg.throttle = self.vel # Always positive velocity
#             control_msg.steer = self.steer
#             control_msg.reverse = self.reverse # Set reverse mode
#             control_msg.brake = self.brake

#             self.control_publisher.publish(control_msg)

#             # Update plot
#             self.rear_trajectory_x.append(self.rear_axle[0])
#             self.rear_trajectory_y.append(self.rear_axle[1])
#             self.x_data.append(current_pos[0])
#             self.y_data.append(current_pos[1])
#             self.update_plot()
            
#             # Fix position for heading estimation
#             self.prev_vehicle_pos = self.current_vehicle_pos
#             self.current_vehicle_pos = current_pos
            
#             pass

# def main(args=None):
#     rclpy.init(args=args)
#     parking_node = ParallelParkingNode()
#     rclpy.spin(parking_node)
#     parking_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time
import numpy as np
from nav_msgs.msg import Odometry
# from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import PoseStamped, PoseArray
import matplotlib.pyplot as plt
import numpy as np
import time
from scipy.spatial import distance
# import carla

class ParallelParkingNode(Node):
    def __init__(self):
        super().__init__('parallel_parking')
        self.cones = []
        self.threshold_distance = 1.5
        self.start_goal = self.final_goal = self.spot_center = self.spot_width = self.spot_length = None
        self.set_vehicle_info()

        self.start_parking_publisher = self.create_publisher(
            Bool,
            '/switch_to_parking',  # CARLA control topic
            10
        )

        self.start_goal_publisher = self.create_publisher(
            PoseStamped,
            '/start_goal',  # CARLA control topic
            10
        )

        self.final_goal_publisher = self.create_publisher(
            PoseStamped,
            '/final_goal',  # CARLA control topic
            10
        )
        self.count = 0

        self.goal_pose_publisher= self.create_publisher(
            PoseStamped,
            '/goal_pose',  # CARLA control topic
            10
        )

        self.clock = 0
        self.clock_sub = self.create_subscription(Time, '/clock', self.set_clock, 10)
        
        self.trigger = False

        self.cone_sub = self.create_subscription(PoseArray, '/cone_poses/map', self.cone_callback, 10)
        
        
        # self.spot_center, self.spot_width, self.spot_length = self.get_spot_size()

        timer_period = 0.5
        
        # self.timer = self.create_timer(timer_period, self.timer_callback)
            
        
    def set_clock(self, msg):
        self.clock = msg.data

    # def timer_callback(self):
    #     parking_msg = Bool()
    #     start_goal_msg = PoseStamped()
    #     final_goal_msg = PoseStamped()
        
    #     if self.spot_length is not None and self.start_goal is not None and self.final_goal is not None:
    #         if self.spot_length >= self.L_min:
    #             parking_msg.data = True

    #             # start_goal_msg.header.seq = 1
    #             start_goal_msg.header.stamp = rclpy.time.Time().to_msg()
    #             start_goal_msg.header.frame_id = "map"

    #             start_goal_msg.pose.position.x = self.start_goal[0]
    #             start_goal_msg.pose.position.y = self.start_goal[1]
    #             start_goal_msg.pose.position.z = 0.0

    #             # Corrigir!
    #             start_goal_msg.pose.orientation.x = 0.0
    #             start_goal_msg.pose.orientation.y = 0.0
    #             start_goal_msg.pose.orientation.z = -0.7
    #             start_goal_msg.pose.orientation.w = 0.7
    #             ############    
    #             # final_goal_msg.header.seq = 1
    #             final_goal_msg.header.stamp = rclpy.time.Time().to_msg()
    #             final_goal_msg.header.frame_id = "map"

    #             final_goal_msg.pose.position.x = self.final_goal[0]
    #             final_goal_msg.pose.position.y = self.final_goal[1]
    #             final_goal_msg.pose.position.z = 0.0

    #             # Corrigir!
    #             final_goal_msg.pose.orientation.x = 0.0
    #             final_goal_msg.pose.orientation.y = 0.0
    #             final_goal_msg.pose.orientation.z = -0.7
    #             final_goal_msg.pose.orientation.w = 0.7

    #             self.start_parking_publisher.publish(parking_msg)
    #             self.start_goal_publisher.publish(start_goal_msg)
    #             self.final_goal_publisher.publish(final_goal_msg)

    #             if self.count < 10:
    #                 self.goal_pose_publisher.publish(final_goal_msg)
    #                 self.get_logger().info("Enviando coordenadas!")
    #             self.count += 1

    #         else:
    #             parking_msg.data = False
    #             self.get_logger().warn("Vaga não encontrada ou menor do que o necessário!")
    #             self.start_parking_publisher.publish(parking_msg)
    #             self.count = 0
    #     else:
    #         self.get_logger().warn("Não há dimensões, ou posição inicial ou final!")

    def timer_callback(self):
        parking_msg = Bool()
        start_goal_msg = PoseStamped()
        final_goal_msg = PoseStamped()

        
        
        if self.spot_length is not None and self.start_goal is not None and self.final_goal is not None:
            if self.spot_length >= self.L_min:

                # start_goal_msg.header.seq = 1
                start_goal_msg.header.stamp = rclpy.time.Time().to_msg()
                start_goal_msg.header.frame_id = "map"

                start_goal_msg.pose.position.x = self.start_goal[0]
                start_goal_msg.pose.position.y = self.start_goal[1]
                start_goal_msg.pose.position.z = 0.0

                # Corrigir!
                start_goal_msg.pose.orientation.x = 0.0
                start_goal_msg.pose.orientation.y = 0.0
                start_goal_msg.pose.orientation.z = -0.7
                start_goal_msg.pose.orientation.w = 0.7
                ############    
                # final_goal_msg.header.seq = 1
                final_goal_msg.header.stamp = rclpy.time.Time().to_msg()
                final_goal_msg.header.frame_id = "map"

                final_goal_msg.pose.position.x = self.final_goal[0]
                final_goal_msg.pose.position.y = self.final_goal[1]
                final_goal_msg.pose.position.z = 0.0

                # Corrigir!
                final_goal_msg.pose.orientation.x = 0.0
                final_goal_msg.pose.orientation.y = 0.0
                final_goal_msg.pose.orientation.z = -0.7
                final_goal_msg.pose.orientation.w = 0.7

                parking_msg.data = True
                
                self.start_parking_publisher.publish(parking_msg)
                self.start_goal_publisher.publish(start_goal_msg)
                self.final_goal_publisher.publish(final_goal_msg)

                

            else:
                parking_msg.data = False
                self.start_parking_publisher.publish(parking_msg)
        else:
            # parking_msg.data = False
            self.get_logger().warn("Não há dimensões, ou posição inicial ou final!")
            parking_msg.data = True
                
            self.start_parking_publisher.publish(parking_msg)

            self.goal_pose_publisher.publish(PoseStamped())
            self.get_logger().info("Enviando coordenadas vazias!")
            # self.count = 0


        # if self.count < 2:
            
        #     self.count += 1
            
            


    def set_vehicle_info(self):
        self.width = 2.16
        self.length = 4.79
        self.overh = 0.89
        self.wheel_base = 3.0
        self.R = 5.0
        self.beta = 1.02
        self.Ri = self.wheel_base/np.tan(self.beta) - self.width/2
        self.Re = np.sqrt(np.power(self.Ri+self.width, 2) + np.power(self.wheel_base+self.overh, 2))
        self.L_min = self.overh + np.sqrt(self.Re**2 - self.Ri**2)
        return
    
    def cone_callback(self, msg):
        # Store cone positions from PoseArray message
        self.cones = [(pose.position.x, pose.position.y) for pose in msg.poses]
        
        # cones = [np.array([ 188.2138846 , -258.37533465]), np.array([ 190.39942438, -258.25486852]), np.array([ 190.41268506, -249.25856427]), np.array([ 187.70167536, -249.88588678])]
        # Cluster and average cone positions
        clustered_cones = self.cluster_cones(self.cones)
        self.get_logger().info(f"Cones: {clustered_cones}")
        
        if len(clustered_cones) == 4:
            if not self.trigger:
                # Calculate the parking spot dimensions
                self.spot_center, self.spot_width, self.spot_length = self.calculate_parking_spot_dimensions(clustered_cones)
                self.get_logger().info(f"Parking spot dimensions - Width: {self.spot_width:.2f}m, Length: {self.spot_length:.2f}m")
                self.start_goal, self.final_goal = self.get_parking_points()
                self.trigger = True
            else:
                self.goal_pose_publisher.publish(PoseStamped())
                self.get_logger().warn("Dimensions calculated!")
        else:
            self.get_logger().warn("Could not determine parking spot dimensions - require exactly 4 cones.")
        
        self.timer_callback()
        self.get_logger().warn("Called callback!")

    def cluster_cones(self, cones):
        # Clustering cones based on distance
        clusters = []
        for cone in cones:
            found_cluster = False
            for cluster in clusters:
                if any(distance.euclidean(cone, other_cone) < self.threshold_distance for other_cone in cluster):
                    cluster.append(cone)
                    found_cluster = True
                    break
            if not found_cluster:
                clusters.append([cone])
        
        # Average positions of each cluster to find the final cone positions
        averaged_cones = [np.mean(cluster, axis=0) for cluster in clusters if len(cluster) > 0]
        return averaged_cones
    # cones = [np.array([ 190.62325565, -256.26152087]), np.array([ 190.72545946, -247.38585075]), np.array([ 188.43944193, -247.66727318]), np.array([ 188.51063072, -256.46668261])]

    def calculate_parking_spot_dimensions(self, cones):
        # Ensure cones are numpy arrays for easy distance calculations
        cones = np.array(cones)

        # Calculate all pairwise distances
        dists = distance.cdist(cones, cones, 'euclidean')
        
        # Step 1: Identify the two points that are farthest apart
        flat_idx = np.argmax(dists)
        point1_idx, point3_idx = np.unravel_index(flat_idx, dists.shape)
        point1, point3 = cones[point1_idx], cones[point3_idx]

        # Step 2: Find the two remaining points and assign them as point2 and point4
        remaining_indices = [i for i in range(len(cones)) if i not in [point1_idx, point3_idx]]
        point2_idx, point4_idx = remaining_indices
        point2, point4 = cones[point2_idx], cones[point4_idx]
        
        # Ensure points are ordered in a rectangular order: point1, point2, point3, point4
        if np.linalg.norm(point1 - point2) > np.linalg.norm(point1 - point4):
            point2, point4 = point4, point2
        
        # Step 3: Calculate the width and length
        width = np.linalg.norm(point1 - point2)
        length = np.linalg.norm(point1 - point4)/1.3 # Fator de segurança
        
        # Step 4: Calculate the center of the rectangle
        center = (point1 + point2 + point3 + point4) / 4
        center_x, center_y = center[0], center[1]

        return np.array([center_x, center_y]), min(width, length), max(width, length)

    # def get_cones_poses(self):
    #     ## Corrigir!
    #     spot_width, spot_length = 2.3, 7.0
    #     spot_center = np.array([189.2, -254])
    #     cones = np.array([[spot_center[0]+spot_width/2, spot_center[1]+spot_length/2],
    #                       [spot_center[0]+spot_width/2, spot_center[1]-spot_length/2],
    #                       [spot_center[0]-spot_width/2, spot_center[1]+spot_length/2],
    #                       [spot_center[0]-spot_width/2, spot_center[1]-spot_length/2]])
    #     return cones
    
    # def get_spot_size(self):
    #     cones = self.cones.copy()
    #     x_center = (cones[0,0]+cones[2,0])/2
    #     y_center = (cones[0,1]+cones[1,1])/2
    #     spot_center = np.array([x_center, y_center])
    #     spot_width = cones[0,0]-cones[2,0]
    #     spot_length = cones[0,1]-cones[1,1]
    #     return spot_center, spot_width, spot_length
        
    def get_parking_points(self):
        start_goal = np.array([
                            self.spot_center[0]+self.width*1.5, 
                            self.spot_center[1]-self.spot_length/2-self.overh+self.wheel_base/2
                    ])
        final_goal = np.array([self.spot_center[0], self.spot_center[1]+self.spot_length/2-self.overh])
        return start_goal, final_goal



def main(args=None):
    rclpy.init(args=args)
    parking_node = ParallelParkingNode()
    rclpy.spin(parking_node)
    parking_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

