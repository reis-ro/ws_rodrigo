import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import cv2
from cv_bridge import CvBridge
import numpy as np

class ConePoseEstimator(Node):
    def __init__(self):
        super().__init__('cone_pose_estimator')

        # Initialize publishers, subscribers, and tf buffer/listener
        self.map_det_pub = self.create_publisher(PoseArray, '/cone_poses/map', 1)
        self.detections_sub = self.create_subscription(PoseArray, '/camera/cone_centers', self.detections_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/carla/ego_vehicle/rgb_front/right/camera_info', self.camera_info_callback, 10)
        self.depth_image_sub = self.create_subscription(Image, '/carla/ego_vehicle/depth_front/image', self.depth_callback, 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # For handling depth image and camera parameters
        self.bridge = CvBridge()
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        # Parameters for cone tracking
        self.distance_threshold = 2.0  # Increase this threshold as needed
        self.cone_memory = []
        self.get_logger().info("Cone Pose Estimator node initialized")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

    def detections_callback(self, detections_msg):
        if self.depth_image is None or None in (self.fx, self.fy, self.cx, self.cy):
            self.get_logger().warn("Depth image or camera info not received yet")
            return

        all_detections = PoseArray()
        all_detections.header.frame_id = "map"

        for pose in detections_msg.poses:
            u = int(pose.position.x)
            v = int(pose.position.y)

            # Retrieve depth value from the depth image at (u, v)
            depth = self.depth_image[v, u] if 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1] else None
            if depth is None or np.isnan(depth) or depth <= 0:
                self.get_logger().warn(f"Invalid depth value at ({u}, {v})")
                continue

            x_cam = (u - self.cx) * depth / self.fx
            y_cam = (v - self.cy) * depth / self.fy
            z_cam = depth

            pose_in_camera = PoseStamped()
            pose_in_camera.header.frame_id = "ego_vehicle/rgb_front_right"
            # pose_in_camera.pose.position.x = float(z_cam)
            # pose_in_camera.pose.position.y = float(-x_cam)
            # pose_in_camera.pose.position.z = float(y_cam)
            pose_in_camera.pose.position.x = float(x_cam)
            pose_in_camera.pose.position.y = float(y_cam)
            pose_in_camera.pose.position.z = float(z_cam)
            
            pose_in_camera.pose.orientation.x = 0.707
            pose_in_camera.pose.orientation.y = 0.707
            pose_in_camera.pose.orientation.z = 0.0
            pose_in_camera.pose.orientation.w = 0.0
            

            try:
                # Transform to the map frame instead of base_link
                transform = self.tf_buffer.lookup_transform("map", "ego_vehicle/rgb_front_right", rclpy.time.Time())
                pose_in_map = do_transform_pose(pose_in_camera.pose, transform)
                self.update_or_add_cone(pose_in_map)

            except Exception as e:
                self.get_logger().error(f"Transform error: {e}")

        all_detections.poses = [cone for cone in self.cone_memory]
        print('cones', [cone for cone in self.cone_memory])
        all_detections.header.stamp = rclpy.time.Time().to_msg()
        self.map_det_pub.publish(all_detections)

    def update_or_add_cone(self, new_pose):
        found = False
        for existing_pose in self.cone_memory:
            dist = np.sqrt(
                (existing_pose.position.x - new_pose.position.x) ** 2 +
                (existing_pose.position.y - new_pose.position.y) ** 2 +
                (existing_pose.position.z - new_pose.position.z) ** 2
            )
            if dist < self.distance_threshold:
                existing_pose.position.x = (existing_pose.position.x + new_pose.position.x) / 2
                existing_pose.position.y = (existing_pose.position.y + new_pose.position.y) / 2
                existing_pose.position.z = (existing_pose.position.z + new_pose.position.z) / 2
                found = True
                break
        if not found:
            self.get_logger().info("New Cone Added!")
            self.cone_memory.append(new_pose)

def main(args=None):
    rclpy.init(args=args)
    node = ConePoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
