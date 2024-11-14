class lidarProcessor():
    def __init__(self):
        # sensor data from lidar
        # initializing variables for lidar
        self.lidar = None
        self.lidar_angle_max = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.lidar_ranges = None

        # initializing variables for ips
        self.ips = None
        self.ips_x = None
        self.ips_y = None 
        self.ips_z = None

    def process_lidar(self):
        self.lidar_angle_max = self.lidar.get('angle_max')
        self.lidar_angle_min = self.lidar.get('angle_min')
        self.lidar_angle_increment = self.lidar.get('angle_increment')
        self.lidar_ranges = self.lidar.get('ranges')

        self.ips_x = self.ips.get('x')
        self.ips_y = self.ips.get('y')
        self.ips_z = self.ips.get('z')