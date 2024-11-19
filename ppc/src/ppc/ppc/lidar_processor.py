import math

class lidarProcessor():
    def __init__(self):
        # sensor data from lidar
        # initializing variables for lidar
        self.lidar = {}

        # initializing variables for ips
        self.ips = None

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
        lidar_readings = []
        ips_readings = []
        
        for count, range in enumerate(self.lidar["ranges"], start=1):
            if (math.isinf(range)) != True:
                #lidar_info = (count * self.lidar["angle_increment"], range)
                ips_info = [self.ips.get('x'), self.ips.get('y'), self.ips.get('z')]
                
                """
                print(f"Lidar info:\tAngle: {lidar_info[0]}\tRange: {lidar_info[1]}")
                print(f"IPS info:\tX: {ips_info[0]}\tY:{ips_info[1]}Z:{ips_info[2]}")
                """

                # converting polar coordinates to cartesian coordinates
                current_point_x = range.cos(count * self.lidar["angle_increment"])
                current_point_y = range.sin(count * self.lidar["angle_increment"])
                lidar_readings.append((current_point_x, current_point_y))
                ips_readings.append(ips_info)

                # obtaining the points in world frame
                



        