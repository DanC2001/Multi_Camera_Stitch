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
        self.declare_parameter('intrinsics_extrinsics_file', '/workspaces/Abyss_Solutions_Test_DanielCook/dc_abyss_solutions_test/intrinsics_extrinsics.json')
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
        self.mosaic_publisher = self.create_publisher(Image, '/mosaic/image', 10)

        self.simple_publisher = self.create_publisher(Image, '/mosaic/simple_hconcat', 10)

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

            # Initialize the Stitcher
            stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)

            # Perform stitching
            status, stitched = stitcher.stitch(images)

            if status != cv2.Stitcher_OK:
                self.get_logger().error(f"Stitching failed with status {status}")
                return

            # Convert the stitched image back to ROS Image message
            mosaic_msg = self.bridge.cv2_to_imgmsg(stitched, encoding='bgr8')
            mosaic_msg.header.stamp = self.get_clock().now().to_msg()
            mosaic_msg.header.frame_id = 'mosaic'

            # Publish the mosaic image
            self.mosaic_publisher.publish(mosaic_msg)
            self.get_logger().info("Published stitched mosaic image.")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer_callback: {e}")





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
