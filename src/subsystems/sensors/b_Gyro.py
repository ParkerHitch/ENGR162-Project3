import brickpi3
import math

class EV3Gyro:
    def __init__(self, BP, port):
        self.BP = BP
        self.port = port
        self.BP.set_sensor_type(self.port, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS)
        self.yawOff = 0

    def getYawRads(self):
        try:
            return math.radians((self.BP.get_sensor(self.port)[0] * -1) - self.yawOff)
        except brickpi3.SensorError:
            return None

    def getYawDPS(self):
        try:
            return self.BP.get_sensor(self.port)[1] * -1
        except brickpi3.SensorError:
            return None
    
    def zeroYaw(self):
        self.yawOff = self.BP.get_sensor(self.port)[0] * -1
