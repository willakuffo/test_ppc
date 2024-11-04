import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')
        self.subscriber = self.create_subscription(
            msg_type=Imu,
            topic='/autodrive/f1tenth_1/imu',
            qos_profile=10,
            callback=self.imu_callback
        )


    def imu_callback(self,msg):
        self.get_logger().info(msg)


def main(args = None):
    rclpy.init(args=args)
    node = SimpleSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()