#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from dc_abyss_solutions_test.camera_handler import CameraHandler  # Ensure this import is correct based on your package structure


class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_node')

        # Declare and retrieve the JSON file path from parameters
        self.declare_parameter('intrinsics_extrinsics_file', '/workspaces/Multi_Camera_Stitch/dc_abyss_solutions_test/intrinsics_extrinsics.json')
        json_path = self.get_parameter('intrinsics_extrinsics_file').get_parameter_value().string_value

        if not os.path.exists(json_path):
            self.get_logger().error(f"Calibration JSON file does not exist: {json_path}")
            rclpy.shutdown()
            return

        # Load camera calibration data
        with open(json_path, 'r') as f:
            camera_data = json.load(f)

        # Initialize CameraHandlers for camera_1, camera_2, camera_3
        self.cam1_handler = CameraHandler(
            node=self,
            camera_info_dict=camera_data["camera_1"],
            topic_name='/camera_1/image',
            publish_undistorted=True
        )
        self.cam2_handler = CameraHandler(
            node=self,
            camera_info_dict=camera_data["camera_2"],
            topic_name='/camera_2/image',
            publish_undistorted=True
        )
        self.cam3_handler = CameraHandler(
            node=self,
            camera_info_dict=camera_data["camera_3"],
            topic_name='/camera_3/image',
            publish_undistorted=True
        )

        # Publisher for the final mosaic
        self.bridge = CvBridge()
        self.panorama_publisher = self.create_publisher(Image, '/panorama/image', 10)

        self.simple_publisher = self.create_publisher(Image, '/panorama/simple_hconcat', 10)

        # Create a timer that triggers the callback every 0.5 seconds
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("MultiCameraNode has been initialized.")

    def timer_callback(self):
        """
        This callback retrieves the latest images from each camera handler,
        stitches them together using OpenCV's Stitcher, and publishes the mosaic.
        """
        # Retrieve the latest images from each camera handler
        img1 = self.cam1_handler.get_latest_undistorted()
        img2 = self.cam2_handler.get_latest_undistorted()
        img3 = self.cam3_handler.get_latest_undistorted()

        if img1 is None or img2 is None or img3 is None:
            self.get_logger().warn("One or more images are not available yet.")
            return
        
        try:
            # Prepare images for stitching
            images = [img3, img2, img1]

            # Publish simple horcat stitch
            simple_image = cv2.hconcat(images)
            simple_msg = self.bridge.cv2_to_imgmsg(simple_image, encoding='bgr8')
            simple_msg.header.stamp = self.get_clock().now().to_msg()
            simple_msg.header.frame_id = 'simple hconcat pnaorama'
            self.simple_publisher.publish(simple_msg)

            # Get the homography matrix (Assime C2 is the center reference)
            H1 = self._get_homography(self.cam1_handler.camera_matrix, self.cam2_handler.transform_matrix, self.cam1_handler.transform_matrix)
            H2 = self._get_homography(self.cam2_handler.camera_matrix, self.cam2_handler.transform_matrix, self.cam2_handler.transform_matrix)
            H3 = self._get_homography(self.cam3_handler.camera_matrix, self.cam2_handler.transform_matrix, self.cam3_handler.transform_matrix)

            # Warp images
            height, width = self.cam1_handler.undistorted_cv_image.shape[:2]
            
            # Estimate size for panorama (This can be better, alot of the image gets cut off.)
            panorama_width = width * 3  
            panorama_height = height

            # Initialize panorama canvas
            panorama = np.zeros((panorama_height, panorama_width, 3), dtype=np.uint8)


            # Define translation to place camera_2 at the center
            translation = np.array([[1, 0, width],
                                    [0, 1, 0],
                                    [0, 0, 1]])
            
            ## The below method just adds the pics on top of the center image using weights. 
            ## Blending would be a better option, but for simplicity here we are. 
            # Start with the reference frame. 
            panorama[0:height, width:2*width] = self.cam2_handler.undistorted_cv_image

            # Warp and place camera_1
            warped1 = cv2.warpPerspective(self.cam1_handler.undistorted_cv_image, translation @ H1, (panorama_width, panorama_height))
            panorama = cv2.addWeighted(panorama, 1, warped1, 1, 0)

            # Warp and place camera_3
            warped2 = cv2.warpPerspective(self.cam3_handler.undistorted_cv_image, translation @ H3, (panorama_width, panorama_height))
            panorama = cv2.addWeighted(panorama, 1, warped2, 1, 0)

            # Show the panorama (Probably want to compress/resize the image, it gets pretty big)
            panorama_msg = self.bridge.cv2_to_imgmsg(panorama, encoding='bgr8')
            panorama_msg.header.stamp = self.get_clock().now().to_msg()
            panorama_msg.header.frame_id = 'panorama'
            self.panorama_publisher.publish(panorama_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer_callback: {e}")

    def _get_homography(self, K, T_ref, T_other):
        """
        Function to compute homography relative to another camera
        :param K: The camera intrinsic matrix.
        :param T_ref: The refrence extrinsic matrix
        :param T_other: The extrinsic matrix of the camera to convert
        """
        R_ref = T_ref[:3, :3]
        t_ref = T_ref[:3, 3]
        R_other = T_other[:3, :3]
        t_other = T_other[:3, 3]
        
        # Relative rotation and translation
        R_rel = R_ref @ R_other.T
        t_rel = R_ref @ (-R_other.T @ t_other + t_ref)
        H = K @ R_rel @ np.linalg.inv(K)
        return H




def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MultiCameraNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
