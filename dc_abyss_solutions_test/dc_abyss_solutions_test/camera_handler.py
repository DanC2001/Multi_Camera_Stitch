#!/usr/bin/env python3

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraHandler:
    """
    Handles subscription to a single camera topic, and stores:
      - Latest raw (OpenCV) image
      - Latest undistorted image
      - Camera intrinsics & extrinsics
    """
    def __init__(self, node, camera_info_dict, topic_name, queue_size=10):
        """
        :param node: The ROS2 node that will create the subscription.
        :param camera_info_dict: A dictionary containing camera intrinsics & extrinsics.
        :param topic_name: The image topic to subscribe to.
        :param queue_size: Subscription queue size (default 10).
        """
        self.node = node
        self.bridge = CvBridge()

        # Store camera info
        self.camera_info = camera_info_dict
        # Parse out intrinsics
        self.intrinsics = self.camera_info["intrinsics"]  # e.g. { "focal_length":..., "principal_point":..., "distortion":... }
        self.extrinsics = self.camera_info["extrinsics"]  # e.g. { "pose":... }
        
        # Build camera_matrix and dist_coeffs from the dictionary
        self.camera_matrix, self.dist_coeffs = self._build_camera_matrix(self.intrinsics)

        # We'll store the latest raw and undistorted images here
        self.raw_cv_image = None
        self.undistorted_cv_image = None

        # Create subscription
        self.sub = node.create_subscription(
            Image,
            topic_name,
            self._callback,
            queue_size
        )
        node.get_logger().info(f"[CameraHandler] Subscribed to {topic_name}")

    def _build_camera_matrix(self, intrinsics):
        """
        Builds the 3x3 camera matrix and distortion array from your dictionary.
        Expected structure similar to:
        {
          "image_size": {"x": 1028, "y": 752},
          "focal_length": 600.0974,
          "principal_point": { "x": 514.89, "y": 378.92 },
          "distortion": {
             "radial": { "k1": ..., "k2": ..., "k3": ... },
             "tangential": { "p1": ..., "p2": ... }
          }
        }
        """
        fx = intrinsics["focal_length"]
        fy = fx  # or could be different if aspect ratio differs
        cx = intrinsics["principal_point"]["x"]
        cy = intrinsics["principal_point"]["y"]

        # Distortion
        k1 = intrinsics["distortion"]["radial"]["k1"]
        k2 = intrinsics["distortion"]["radial"]["k2"]
        k3 = intrinsics["distortion"]["radial"].get("k3", 0.0)
        p1 = intrinsics["distortion"]["tangential"]["p1"]
        p2 = intrinsics["distortion"]["tangential"]["p2"]

        camera_matrix = np.array([
            [fx,   0.,  cx],
            [0.,   fy,  cy],
            [0.,   0.,   1.]
        ], dtype=np.float64)

        dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
        return camera_matrix, dist_coeffs

    def _callback(self, msg: Image):
        """
        Subscription callback. Convert ROS Image -> CV, store raw & undistorted images.
        """
        # Convert to OpenCV BGR image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.raw_cv_image = cv_image

        # Undistort using OpenCV
        # (If intrinsics or distortion are not valid, you might skip or handle gracefully.)
        self.undistorted_cv_image = cv2.undistort(
            cv_image,
            self.camera_matrix,
            self.dist_coeffs
        )

    def get_latest_raw(self):
        """
        :return: The latest raw OpenCV image, or None if no image received yet.
        """
        return self.raw_cv_image

    def get_latest_undistorted(self):
        """
        :return: The latest undistorted OpenCV image, or None if no image received yet.
        """
        return self.undistorted_cv_image

    def get_extrinsics(self):
        """
        :return: The extrinsics dictionary from the constructor for reference
        """
        return self.extrinsics