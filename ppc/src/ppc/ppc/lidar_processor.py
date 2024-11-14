class lidarProcessor():
    def __init__(self):
        # sensor data from lidar
        self.lidar = None
        self.lidar_angle_max = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.lidar_ranges = None

    def process_lidar(self):
        self.lidar_angle_max = self.lidar.get('angle_max')
        self.lidar_angle_min = self.lidar.get('angle_min')
        self.lidar_angle_increment = self.lidar.get('angle_increment')
        self.lidar_ranges = self.lidar.get('ranges')
