#!/usr/bin/env python3

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class CameraHandler:
    """
    Handles subscription to a single camera topic, undistorts images,
    warps them into a reference frame, and stores the latest warped image.
    """

    def __init__(
        self,
        node,
        camera_info_dict,
        topic_name,
        queue_size=10,
        publish_undistorted=True
    ):
        """
        :param node: The ROS2 node that will create the subscription and publishers.
        :param camera_info_dict: A dictionary containing camera intrinsics & extrinsics.
        :param topic_name: The image topic to subscribe to.
        :param queue_size: Subscription queue size (default 10).
        :param publish_undistorted: If True, publish undistorted images.
        """
        self.node = node
        self.bridge = CvBridge()

        # Store camera info
        self.camera_info = camera_info_dict
        self.intrinsics = self.camera_info["intrinsics"]
        self.extrinsics = self.camera_info["extrinsics"]

        # Build camera_matrix and dist_coeffs
        self.camera_matrix, self.dist_coeffs = self._build_camera_matrix(self.intrinsics)
        self.transform_matrix, self.rotation_matrix, self.translation_vector = self._build_transform_matrix(self.extrinsics)

        # Latest images
        self.raw_cv_image = None
        self.undistorted_cv_image = None

        # Lock for thread-safe access
        self.lock = threading.Lock()

        # Publisher for undistorted images
        self.publish_undistorted = publish_undistorted
        if self.publish_undistorted:
            undist_topic = f"{topic_name}/undistorted"
            self.undistorted_pub = node.create_publisher(Image, undist_topic, 10)
            node.get_logger().info(f"[CameraHandler] Publishing undistorted images to {undist_topic}")
        else:
            self.undistorted_pub = None

        # Create the image subscription
        self.sub = node.create_subscription(
            Image,
            topic_name,
            self._callback,
            queue_size
        )
        node.get_logger().info(f"[CameraHandler] Subscribed to {topic_name}")

    def _build_rotation_matrix(self, extrinsics):
        roll, pitch, yaw = extrinsics['roll'], extrinsics['pitch'], extrinsics['yaw']
        # Convert degrees to radians if necessary
        # Assuming rotation angles are in radians
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        R = Rz @ Ry @ Rx
        return R
    
    def _build_translation_vector(self, extrinsics):
        t_dict = extrinsics['pose']['translation']
        t = np.array([t_dict['x'],t_dict['y'],t_dict['z']])
        return t


    def _build_transform_matrix(self, extrinsics):
        R = self._build_rotation_matrix(extrinsics)
        t = self._build_translation_vector(extrinsics)

        # Create 4x4 transform matrix
        T = np.vstack([
            np.hstack([R,t]),
            np.array([0,0,0,1])
        ])

        return T, R, t
    
    def _build_camera_matrix(self, intrinsics):
        """
        Constructs the camera intrinsic matrix and distortion coefficients.
        """
        fx = intrinsics["focal_length"]
        fy = fx  # Assuming square pixels; modify if fy differs
        cx = intrinsics["principal_point"]["x"]
        cy = intrinsics["principal_point"]["y"]

        # Distortion coefficients
        k1 = intrinsics["distortion"]["radial"].get("k1", 0.0)
        k2 = intrinsics["distortion"]["radial"].get("k2", 0.0)
        k3 = intrinsics["distortion"]["radial"].get("k3", 0.0)
        p1 = intrinsics["distortion"]["tangential"].get("p1", 0.0)
        p2 = intrinsics["distortion"]["tangential"].get("p2", 0.0)

        camera_matrix = np.array([
            [fx,   0.,  cx],
            [0.,   fy,  cy],
            [0.,   0.,   1.]
        ], dtype=np.float64)

        dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
        return camera_matrix, dist_coeffs

    def _callback(self, msg: Image):
        """
        Callback function for incoming image messages.
        Converts ROS Image to OpenCV, undistorts, and warps if configured.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"[CameraHandler] Error converting image: {e}")
            return

        with self.lock:
            self.raw_cv_image = cv_image

            # Undistort the image
            self.undistorted_cv_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

            # Publish undistorted image if required
            if self.publish_undistorted and self.undistorted_pub:
                try:
                    undist_msg = self.bridge.cv2_to_imgmsg(self.undistorted_cv_image, "bgr8")
                    undist_msg.header = msg.header
                    self.undistorted_pub.publish(undist_msg)
                except Exception as e:
                    self.node.get_logger().error(f"[CameraHandler] Error publishing undistorted image: {e}")

    def get_latest_raw(self):
        """
        Returns the latest raw OpenCV image.
        """
        with self.lock:
            return self.raw_cv_image

    def get_latest_undistorted(self):
        """
        Returns the latest undistorted OpenCV image.
        """
        with self.lock:
            return self.undistorted_cv_image


    def get_camera_matrix(self):
        """
        Returns the camera intrinsic matrix.
        """
        return self.camera_matrix
    
    def get_rotation_matrix(self):
        """
        Returns the extrinsic rotation matrix.
        """
        return self.rotation_matrix


