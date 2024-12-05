import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import pickle
from .naive_state_estimator import NaiveStateEstimator
from .lidar_processor import lidarProcessor
import time
class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')


        #naive estimator
        self.nse = NaiveStateEstimator()
        self.lidar_processor = lidarProcessor()

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
        self.ips = {}
        self.vtrue = 0


        self.world_lidar_track_bounds = []

        #create timer callback to run general processig of sensor data
        self.create_timer(timer_period_sec=0.03,
                      callback= self.pushToProcess)

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
        
        self.right_encoder_subscriber = self.create_subscription(
            msg_type=Float32,
            topic='/autodrive/f1tenth_1/speed',
            qos_profile=10,
            callback=self.speed_callback
        )

        self.ips_subscriber = self.create_subscription(
            msg_type=Point,
            topic='/autodrive/f1tenth_1/ips',
            qos_profile=10,
            callback=self.ips_callback
        )

    def pushToProcess(self):
        self.nse.imu = self.imu
        self.lidar_processor.lidar = self.lidar
        #self.lidar
        self.lidar_processor.ips = self.ips
        self.nse.encoder = self.encoder
        self.nse.vtrue = self.vtrue
        #print(self.imu.keys())



        if len(self.lidar.keys())>1:

            vals = self.nse.q2e([self.imu["orientation"]["x"], self.imu["orientation"]["y"], self.imu["orientation"]["z"], self.imu["orientation"]["w"]])
            self.lidar_processor.cur_orientation_z = self.nse.get_yaw()
            world_track_bounds = self.lidar_processor.process_lidar()
            self.world_lidar_track_bounds.append(world_track_bounds)
            
            #self.get_logger().info(f"Orientation about z-axis: {cur_orientation, vals[2]}")
            #print(vals)
            #print(self.lidar)
            #z = self.nse.get_yaw()
            #self.get_logger().info(f'q2e::{vals[0],self.nse.cur_orientation_z,z}')
      

            """
        self.get_logger().info(f'Lidar Angle max: {self.lidar_processor.lidar_angle_max}')
        self.get_logger().info(f'Lidar Angle min: {self.lidar_processor.lidar_angle_min}')
        self.get_logger().info(f'Lidar Angle increment: {self.lidar_processor.lidar_angle_increment}')
        self.get_logger().info(f'Lidar Angle ranges: {self.lidar_processor.lidar_ranges}')

        self.get_logger().info(f'ips x: {self.lidar_processor.ips_x}')
        self.get_logger().info(f'ips y: {self.lidar_processor.ips_y}')
        self.get_logger().info(f'ips z: {self.lidar_processor.ips_z}')"""
        """if len(self.imu.keys())>1: 
            self.nse.position_from_encoder()
        self.get_logger().info(f'velocity from imu: {self.nse.x,self.nse.y, self.vtrue}')"""




    def speed_callback(self, msg):
        self.vtrue = msg.data
        

    

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
        #self.get_logger().info(f'enc: {msg.velocity}')

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
    
    def ips_callback(self, msg):
        self.ips = {'x' : msg.x,
                    'y' : msg.y,
                    'z' : msg.z}
    
    def on_shutdown(self):
        trackfle = open("track_points_r",'wb')
        pickle.dump(self.world_lidar_track_bounds, trackfle)
        trackfle.close()
        print('track points saved')



def main(args = None):
    rclpy.init(args=args)
    node = SimpleSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()