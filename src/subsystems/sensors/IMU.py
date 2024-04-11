from lib.MPU9250 import MPU9250
from lib.Vector3 import Vector3
import time
import math
import config

class IMU:

    def __init__(self):
        self.mpu9250 = MPU9250()
        self.accel = Vector3(0,0,0)
        self.gyro = Vector3(0,0,0)
        self.mag = Vector3(0,0,0)
        self.lastUpdated = time.time_ns()
        self.zeroYaw = 0


    # sets self.biases and self.std
    def initialize(self):
        print("Initializing IMU...")
        print("IMU Initialized")
        self.lastUpdated = time.time_ns()


    def update(self):
        accel = self.mpu9250.readAccel()
        accel = Vector3(accel.y, -accel.x, accel.z)
        gyro = self.mpu9250.readGyro()
        mag = self.mpu9250.readMagnet()
        mag = Vector3(mag.x, -mag.y, mag.z)

        if(mag.mag() > 0.0001):
            self.mag = mag

        if(gyro.mag() > 0.0001):
            self.gyro = gyro
        
        if(accel.mag() > 0.0001):
            self.accel = accel

    def getMag(self):
        return self.mag

    def getYaw(self):
        return self.mag.xy().angle()
    
    def getYawRate(self):
        return self.gyro.z
