from src.subsystems.sensors.IMU import IMU

class SensorArray:

    def __init__(self, BP, ultrasonicRotorPort):
        self.BP = BP
        self.imu = IMU()
        self.imu.initialize()
        self.ultrasonicRotorPort = ultrasonicRotorPort
        self.yaw = 0

    def update(self):
        self.imu.update()