import numpy as np
from .racecar_params import f1tenth_params
from scipy.spatial.transform import Rotation

class NaiveStateEstimator():
    def __init__(self):
        #sensor data from sensor subscribers
        self.imu = None
        self.encoder = None
        self.lidar = None

        #initialize states
        self.x = 0
        self.xold = 0
        self.y = 0
        self.yold = 0

        self.theta = 0
        self.theta_old = 0

        self.vtrue = 0

        self.ax_k_imu = 0
        self.ax_k_1_imu = 0

        self.vx_k_imu = 0
        self.vx_k_1_imu = 0

        self.prev_orientation =  -np.pi / 2 # initial orientation of the car's frame with respect to the world frame
        self.cur_orientation_z = 0

    def q2e(self,q):
        euler = Rotation.from_quat(q).as_euler('zxy')
        euler = np.mod(euler, 2*np.pi)
        return euler

    def quaternion_to_euler(self,q):
        x, y, z, w = q
        # Yaw (ψ)
        yaw = 2 * np.arctan2(2*(w*z + x*y) , 1 - 2*(x**2 + y**2))

        # Pitch (θ)
        pitch = np.arcsin(2*(w*y - z*x))
        
        # Roll (φ)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(y**2 + z**2))
        
        return roll, pitch, yaw
    
    def get_yaw(self):
        #self.orientation_from_ang_velocity()
        theta_imu = self.q2e([self.imu["orientation"]["x"], self.imu["orientation"]["y"], self.imu["orientation"]["z"], self.imu["orientation"]["w"]])[0] - np.pi # this is done to offset the orientation to begin at 0 radians

        """
        print(euler_2pi)
        theta_imu = euler_get_yaw[0]
        
        theta_imu_yaw = 0
        deviation = 0

        threshold = 2 * np.pi/180 # 5 degrees is used as threshold
        #print('ttt::',theta_imu)
         #wrap yaw from 0 to 2pi

        #self.cur_orientation_z = self.cur_orientation_z % 2*np.pi if self.cur_orientation_z >0 else self.cur_orientation_z % -2*np.pi
        # lhp - between 0 and pi theta imu = +ve
        if ((self.cur_orientation_z > 0) and (self.cur_orientation_z <= np.pi)):
            theta_imu_yaw = theta_imu + 0
            deviation = np.abs(self.cur_orientation_z - theta_imu)
            print("Q1", theta_imu, self.cur_orientation_z)

            if (deviation > threshold):
                self.cur_orientation_z = theta_imu_yaw
                print("Updated in Q1")


        # rhp - theta imu = -ve
        if ((self.cur_orientation_z > np.pi) and (self.cur_orientation_z <= 2*np.pi)):
            theta_imu_yaw = theta_imu + np.pi + np.pi
            deviation = np.abs(self.cur_orientation_z - theta_imu_yaw)
            print("Q2", theta_imu, self.cur_orientation_z)

            if (deviation > threshold):
                self.cur_orientation_z = theta_imu_yaw
                print("Updated in Q2")

        #-ve orientation clockwise
        # theta_imu -ve
        if ((self.cur_orientation_z < 0) and (self.cur_orientation_z >= -np.pi)):
            theta_imu_yaw = theta_imu + 0
            deviation = np.abs(self.cur_orientation_z - theta_imu_yaw)
            print("Q1 -ve", theta_imu, self.cur_orientation_z)

            if (deviation > threshold):
                self.cur_orientation_z = theta_imu_yaw
                print("Updated in Q1 -ve")


        # crossed theta_imu = +ve
        if ((self.cur_orientation_z < -np.pi) and (self.cur_orientation_z >= -2*np.pi)):
            theta_imu_yaw = theta_imu - np.pi - np.pi
            deviation = np.abs(self.cur_orientation_z - theta_imu_yaw)
            print("Q2 -ve", theta_imu, self.cur_orientation_z)

            if (deviation > threshold):
                self.cur_orientation_z = theta_imu_yaw
                print("Updated in Q2 -ve")
        
        self.prev_orientation = self.cur_orientation_z
        print(theta_imu, theta_imu_yaw, deviation)
        """

        print(theta_imu)
        self.cur_orientation_z = theta_imu
        return self.cur_orientation_z

    def orientation_z(self):
        #self.orientation_from_ang_velocity()
        theta_imu = self.quaternion_to_euler([self.imu["orientation"]["x"], self.imu["orientation"]["y"], self.imu["orientation"]["z"], self.imu["orientation"]["w"]])[2]

        
        threshold = 5 * np.pi/180 # 5 degrees is used as threshold

        # first quadrant - between 0 and pi/2
        if ((self.cur_orientation_z > 0) and (self.cur_orientation_z < np.pi/2)):
            theta_imu = theta_imu + np.pi/2
            deviation = np.abs(self.cur_orientation_z - theta_imu)
            print("Q1-Q1-Q1-Q1-Q1-Q1", theta_imu, self.cur_orientation_z)

            if (deviation > threshold):
                self.cur_orientation_z = theta_imu
                print("Updated in Q1")
        
        # second quadrant
        if ((self.cur_orientation_z > np.pi/2) and (self.cur_orientation_z < np.pi)):
            theta_imu = theta_imu + 0
            deviation = np.abs(self.cur_orientation_z - np.pi/2 - theta_imu)

            print("Q2-Q2-Q2-Q2-Q2-Q2", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu + np.pi/2
                print("Updated in Q2")
        
        # third quadrant
        if ((self.cur_orientation_z > np.pi) and (self.cur_orientation_z < 3*np.pi/2)):
            theta_imu = -(theta_imu - np.pi/2)
            deviation = np.abs(self.cur_orientation_z - np.pi - theta_imu)

            print("Q3-Q3-Q3-Q3-Q3-Q3", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu + np.pi
                print("Updated in Q3")

        # fourth quadrant
        if ((self.cur_orientation_z > 3*np.pi/2) and (self.cur_orientation_z < 2*np.pi)):
            theta_imu = -theta_imu + 0
            deviation = np.abs(self.cur_orientation_z - 3*np.pi/2 - theta_imu)

            print("Q4-Q4-Q4-Q4-Q4-Q4", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu + 3*np.pi/2
                print("Updated in Q4")

        # fourth quadrant (negative)
        if ((self.cur_orientation_z > -np.pi/2) and (self.cur_orientation_z < 0)):
            theta_imu = -(theta_imu + np.pi/2)

            deviation = np.abs(self.cur_orientation_z - theta_imu)

            print("Q5-Q5-Q5-Q5-Q5-Q5", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu
                print("Updated in Q5")
        
        # third quadrant (negative)
        if ((self.cur_orientation_z > -np.pi) and (self.cur_orientation_z < -np.pi/2)):
            theta_imu = -(theta_imu + 0)
            deviation = np.abs(self.cur_orientation_z + np.pi/2 - theta_imu)

            print("Q6-Q6-Q6-Q6-Q6-Q6", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu - np.pi/2
                print("Updated in Q6")
        
        # second quadrant (negative)
        if ((self.cur_orientation_z > -3*np.pi/2) and (self.cur_orientation_z < -np.pi)):
            theta_imu = theta_imu - np.pi/2
            deviation = np.abs(self.cur_orientation_z + np.pi + theta_imu)

            print("Q7-Q7-Q7-Q7-Q7-Q7", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu - np.pi
                print("Updated in Q7")

        # first quadrant (negative)
        if ((self.cur_orientation_z > 2*np.pi) and (self.cur_orientation_z < 3*np.pi/2)):
            theta_imu = -(theta_imu + 0)
            deviation = np.abs(self.cur_orientation_z + 3*np.pi/2 - theta_imu)

            print("Q8-Q8-Q8-Q8-Q8-Q8", theta_imu, self.cur_orientation_z)
            if (deviation > threshold):
                self.cur_orientation_z = theta_imu - 3*np.pi/2
                print("Updated in Q8")
        
        self.prev_orientation = self.cur_orientation_z
        return self.cur_orientation_z




    def velocity_from_imu(self):
        #current imu acc
        ax_k = self.imu['linear_acceleration']['x']

        #prev imu velocity
        vx_k_1 = self.vx_k_1_imu

        #naively integrate to get current velocity
        self.vx_k_imu = vx_k_1 + 0.05*ax_k 

        #assign prev acceleration as current for next time step
        self.vx_k_1_imu = self.vx_k_imu 
    
    def orientation_from_ang_velocity(self):
        # get current angular velocity
        cur_ang_velocity_z = self.imu['angular_velocity']['z']

        # Use euler integration to estimate current orientation
        self.cur_orientation_z = self.prev_orientation + 0.03*cur_ang_velocity_z

        # store new orienation as previous
        self.prev_orientation = self.cur_orientation_z

        #return self.cur_orientation_z


    
    def position_from_encoder(self):
        '''velocity of center of left and right ecooder axis'''
        left_enc_counts = self.encoder['left_encoder']['position'][0]
        right_enc_counts = self.encoder['right_encoder']['position'][0]


            
        left_enc_distance = (left_enc_counts/\
                        f1tenth_params['encoder_pulses_per_rev'])*2*np.pi\
                            *f1tenth_params['wheel_radius']

        right_enc_distance = (right_enc_counts/\
                        f1tenth_params['encoder_pulses_per_rev'])*2*np.pi\
                            *f1tenth_params['wheel_radius']


        avg_d = (right_enc_distance+left_enc_distance)/2
        self.x = avg_d

        

    def position_from_imu(self):
        pass
    
    
    def velocity_from_encoder(self):
        pass 

