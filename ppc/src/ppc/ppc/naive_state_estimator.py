import numpy as np
from .racecar_params import f1tenth_params

class NaiveStateEstimator():
    def __init__(self):
        #sensor data from sensor subscribers
        self.imu = None
        self.encoder = None
        self.lidar = None

        #initialize states
        self.velocity = [0,0]
        self.position = [0,0]
        self.orientation = 0 #yaw


        self.ax_k_imu = 0
        self.ax_k_1_imu = 0

        self.vx_k_imu = 0
        self.vx_k_1_imu = 0



    


    def quaternion_to_euler(self,q):
        w, x, y, z = q
        # Yaw (ψ)
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(x**2 + y**2))
        
        # Pitch (θ)
        pitch = np.arcsin(2*(w*y - z*x))
        
        # Roll (φ)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(y**2 + z**2))
        
        return roll, pitch, yaw



    def velocity_from_imu(self):
        #current imu acc
        ax_k = self.imu['linear_acceleration']['x']

        #prev imu acc
        ax_k_1 = self.ax_k_1_imu

        #naively integrate to get current velocity
        self.vx_k_imu = ax_k_1 + 0.03*ax_k 


        #assign prev acceleration as current for next time step
        self.ax_k_1_imu = ax_k
    


    def velocity_from_encoder(self):
        '''velocity of center of left and right ecooder axis'''
        pass

    def position_from_imu(self):
        pass
    
    
    def position_from_encoder(self):
        pass 

