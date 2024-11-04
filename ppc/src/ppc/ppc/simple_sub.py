import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')

        #create sensor data structs

        self.imu = {}
        self.encoder = {'left_encoder':{
                                'position':0,
                                'velocity':0,
                                'effort':0 },
                        'right_encoder':{
                                'position':0,
                                'velocity':0,
                                'effort':0 }
                                }
        self.lidar = {}





        #create sensor subscribers
        self.imu_subscriber = self.create_subscription(
            msg_type=Imu,
            topic='/autodrive/f1tenth_1/imu',
            qos_profile=10,
            callback=self.imu_callback
        )
        self.left_encoder_subscriber = self.create_subscription(
            msg_type=JointState,
            topic='/autodrive/f1tenth_1/left_encoder',
            qos_profile=10,
            callback=self.left_encoder_callback
        )
        self.right_encoder_subscriber = self.create_subscription(
            msg_type=JointState,
            topic='/autodrive/f1tenth_1/right_encoder',
            qos_profile=10,
            callback=self.right_encoder_callback
        )

        self.right_encoder_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic='/autodrive/f1tenth_1/lidar',
            qos_profile=10,
            callback=self.lidar_callback
        )


    def imu_callback(self,msg):
        self.imu = {
            'orientation':{
                'x' : msg.orientation.x,
                'y' : msg.orientation.y,
                'z' : msg.orientation.z,
                'w' : msg.orientation.w
            },
            'linear_acceleration':{
                'x' : msg.linear_acceleration.x,
                'y' : msg.linear_acceleration.y,
                'z' : msg.linear_acceleration.z
            },
            'angular_velocity':{
                'x' : msg.angular_velocity.x,
                'y' : msg.angular_velocity.y,
                'z' : msg.angular_velocity.z
              }
        }

        #self.get_logger().info(f'IMU: {msg.linear_acceleration.x}')
    
    def left_encoder_callback(self,msg):
        self.encoder['left_encoder']['position'] = msg.position
        self.encoder['left_encoder']['velocity'] = msg.velocity
        self.encoder['left_encoder']['effort'] = msg.effort


    def right_encoder_callback(self,msg):
        self.encoder['right_encoder']['position'] = msg.position
        self.encoder['right_encoder']['velocity'] = msg.velocity
        self.encoder['right_encoder']['effort'] = msg.effort
    
    
    def lidar_callback(self,msg):
        self.lidar = {'angle_min' : msg.angle_min,
                        'angle_max' : msg.angle_max,
                        'angle_increment' : msg.angle_increment,
                        'time_increment' : msg.time_increment,
                        'scan_time' : msg.scan_time,
                        'range_min' : msg.range_min,
                        'range_max' : msg.range_max,
                        'ranges' : msg.ranges,
                        'intensities' : msg.intensities}
    
    


def main(args = None):
    rclpy.init(args=args)
    node = SimpleSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()