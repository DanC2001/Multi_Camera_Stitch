#!/usr/bin/env python3

import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class MultiCameraNode(Node):
    """
    A ROS2 node that subscribes to three camera topics and publishes
    a single image topic for review.
    """
    def __init__(self):
        super().__init__('multi_camera_node')
        
        # Create subscriptions to the three camera image topics
        self.camera1_sub = self.create_subscription(
            Image,
            '/camera_1/image',
            self.camera1_callback,
            10
        )

        self.camera2_sub = self.create_subscription(
            Image,
            '/camera_2/image',
            self.camera2_callback,
            10
        )

        self.camera3_sub = self.create_subscription(
            Image,
            '/camera_3/image',
            self.camera3_callback,
            10
        )

        # Publisher for the processes image
        self.stitch_pub = self.create_publisher(
            Image,
            '/multi_camera_view',  # This is our combined or processed output
            10
        )

        # Store bridge for converting image classes to cv2 classes
        self.bridge = CvBridge()

        # Internal storage for each camera’s last-received frame
        self.last_img_cam1: Image | None = None
        self.last_img_cam2: Image | None = None
        self.last_img_cam3: Image | None = None

        self.get_logger().info('MultiCameraNode has been started.')

    def camera1_callback(self, msg):
        """
        Callback for Camera 1’s images.
        """
        self.last_img_cam1 = msg
        self.try_publish_review()

    def camera2_callback(self, msg):
        """
        Callback for Camera 2’s images.
        """
        self.last_img_cam2 = msg
        self.try_publish_review()

    def camera3_callback(self, msg):
        """
        Callback for Camera 3’s images.
        """
        self.last_img_cam3 = msg
        self.try_publish_review()

    def try_publish_review(self):
        """
        Attempt to publish a combined image only if we have
        all three camera images available.
        """
        if (self.last_img_cam1 is not None and
            self.last_img_cam2 is not None and
            self.last_img_cam3 is not None):
            
            # -----------------------------------------------------
            # 1. Convert ROS Image messages to OpenCV images
            # -----------------------------------------------------

            # -----------------------------------------------------
            # 1. Convert ROS Image messages to OpenCV images
            # -----------------------------------------------------

            # For now, just re-publish one of the camera images as a placeholder
            # to demonstrate the pipeline.
            result_image_msg = self.last_img_cam1  

            self.stitch_pub.publish(result_image_msg)
            self.get_logger().info('Publishing "combined" review image.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
