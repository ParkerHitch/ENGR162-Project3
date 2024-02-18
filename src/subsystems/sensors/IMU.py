from lib.MPU9250 import MPU9250
from lib.Vector3 import Vector3
import numpy as np
import time
import math
import config

class IMU:
    def __init__(self):
        self.mpu9250 = MPU9250()
        self.accel = Vector3(0,0,0)
        self.gyro = Vector3(0,0,0)
        self.lastUpdated = time.time_ns()


    # sets self.biases and self.std
    def initialize(self):
        print("Initializing IMU...")
        print("IMU Initialized")
        self.lastUpdated = time.time_ns()


    def update(self):
        self.accel = self.mpu9250.readAccel()
        self.gyro = self.mpu9250.readGyro()
        
