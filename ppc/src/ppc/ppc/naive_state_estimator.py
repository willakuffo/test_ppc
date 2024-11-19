import numpy as np
from .racecar_params import f1tenth_params

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



    


    def quaternion_to_euler(self,q):
        x, y, z, w = q
        # Yaw (ψ)
        yaw = 2 * np.arctan(2*(w*z + x*y) / 1 - 2*(x**2 + y**2))
        
        # Pitch (θ)
        pitch = np.arcsin(2*(w*y - z*x))
        
        # Roll (φ)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(y**2 + z**2))
        
        return roll, pitch, yaw



    def velocity_from_imu(self):
        #current imu acc
        ax_k = self.imu['linear_acceleration']['x']

        #prev imu velocity
        vx_k_1 = self.vx_k_1_imu

        #naively integrate to get current velocity
        self.vx_k_imu = vx_k_1 + 0.05*ax_k 

        #assign prev acceleration as current for next time step
        self.vx_k_1_imu = self.vx_k_imu 
    
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

