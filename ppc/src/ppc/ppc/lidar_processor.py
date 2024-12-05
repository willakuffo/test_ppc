import math
import numpy as np

class lidarProcessor():
    def __init__(self):
        # sensor data from lidar
        # initializing variables for lidar
        self.lidar = {}

        # initializing variables for ips
        self.ips = None

        # initializing variables for current orientation about z-axis
        self.cur_orientation_z = None

        self.lidar_points_world_frame = [] # lidar readings in the world frame
        self.lidar_point_car_frame = []

    def process_lidar(self):
        """
        self.lidar_angle_max = self.lidar.get('angle_max')
        self.lidar_angle_min = self.lidar.get('angle_min')
        self.lidar_angle_increment = self.lidar.get('angle_increment')
        self.lidar_ranges = self.lidar.get('ranges')

        self.ips_x = self.ips.get('x')
        self.ips_y = self.ips.get('y')
        self.ips_z = self.ips.get('z')
        """

        #print(type(self.lidar_ranges))
        lidar_readings = [] # lidar readings in car's frame
        ips_readings = [] # distance of the car
        self.lidar_points_world_frame = [] # lidar readings in the world frame

        for count, lidar_range in enumerate(self.lidar["ranges"], start=0):
            if (math.isinf(lidar_range)) != True:
                #lidar_info = (count * self.lidar["angle_increment"], range)
                ips_info = [self.ips.get('x'), self.ips.get('y'), self.ips.get('z')]
                
                """
                print(f"Lidar info:\tAngle: {lidar_info[0]}\tRange: {lidar_info[1]}")
                print(f"IPS info:\tX: {ips_info[0]}\tY:{ips_info[1]}Z:{ips_info[2]}")
                """

                # converting polar coordinates to cartesian coordinates in racecar body frame
                # 2.355 is 135 degrees in radians

                current_point_x = lidar_range * np.cos(-2.355 + count * self.lidar["angle_increment"])
                current_point_y = lidar_range * np.sin(-2.355 + count * self.lidar["angle_increment"])
                lidar_readings.append((current_point_x, current_point_y))
                ips_readings.append(ips_info)

        # obtaining the points in world frame
        rotation_matrix = np.array([
            [np.cos(self.cur_orientation_z), -np.sin(self.cur_orientation_z)],
            [np.sin(self.cur_orientation_z), np.cos(self.cur_orientation_z)]
        ])

        
        # converting each lidar reading in car's frame to world frame using (x_world = A*x_car + d)
        for i in range(len(lidar_readings)):
            #print(ips_readings[i][0], ips_readings[i][1])
            # 
            self.lidar_points_world_frame.append(np.dot(rotation_matrix, np.asarray(lidar_readings[i])) - np.asarray((ips_readings[i][0], ips_readings[i][1])))

        #print(lidar_points_world_frame)
        np.savetxt("Lidar_Readings_car1.txt", lidar_readings, delimiter= " ")
        np.savetxt("Lidar_Readings1.txt", self.lidar_points_world_frame, delimiter = " ")
        return self.lidar_points_world_frame