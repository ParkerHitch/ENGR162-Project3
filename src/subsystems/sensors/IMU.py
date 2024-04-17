from lib.MPU9250 import MPU9250
from lib.Vector2 import Vector2
from lib.Vector3 import Vector3
import time
import math
import config
import numpy as np
import config

class IMU:

    def __init__(self, hardVec=np.zeros(3), softMat=np.eye(3)):
        self.mpu9250 = MPU9250()
        self.accel = Vector3(0,0,0)
        self.gyro = Vector3(0,0,0)
        self.mag = Vector3(0,0,0)
        self.hardIronOffset = hardVec
        self.softIronTrans = softMat
        self.lastUpdated = time.time_ns()
        self.zeroYaw = 0


    # sets self.biases and self.std
    def initialize(self):
        print("Initializing IMU...")
        print("IMU Initialized")
        self.lastUpdated = time.time_ns()

    def update(self):
        accel = self.mpu9250.readAccel()
        gyro = self.mpu9250.readGyro()
        self.rawMag = self.mpu9250.readMagnet()
        
        if(np.linalg.norm(self.rawMag) > 0.0001):
            mag = np.matmul((self.rawMag - self.hardIronOffset), self.softIronTrans)
            mag = Vector3(mag[0], mag[1], mag[2])
            self.mag = mag
        
        if(gyro.mag() > 0.0001):
            self.gyro = gyro
        
        if(accel.mag() > 0.0001):
            accel = accel.rotateAboutY(config.IMU_PITCH)
            self.accel = accel

    def printMag(self):
        print("UNFILTRD:", math.degrees(Vector2(self.rawMag[0], self.rawMag[1]).angle()))
        print("FILTERED:", math.degrees(self.mag.xy().angle()))


    def getMag(self):
        return self.mag

    def getYaw(self):
        return self.mag.xy().angle()
    
    def getYawRate(self):
        return self.gyro.z

    def getPlanarAccel(self):
        return Vector2(self.accel.x, self.accel.y)

    def saveCalibrationData(self, outputFileName):
        test = input(f"Press enter to begin. Ctrl+c to stop. Will be outputted to {outputFileName}. Press -1 and Enter to cancel. ")
        if test=="-1":
            print("cancelled")
            return
        with open(outputFileName, "w") as out:
            while True:
                try:
                    time.sleep(0.02)
                    self.update()
                    newReading = self.getMag()
                    if(newReading.mag() > 0.01):
                        out.write(newReading.__str__() + "\n")
                except KeyboardInterrupt:
                    break
            out.close()
            print(f"Data saved to {outputFileName}")

