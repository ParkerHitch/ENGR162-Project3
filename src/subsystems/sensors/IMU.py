from os import set_blocking
import re
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
      
        # lambertus values
        # self.magZBaseline = 180
        # self.magZThresh = 350
        # self.magZBaseline = -50
        # self.magZThresh = 30
        self.hazCount = 0
        self.magZBaseline = -150
        self.magZThresh = -290

        # self.yawOff = config.IMU_YAW_0_DEF
        # self.off90 = config.IMU_OFF90_DEF
        # self.off180 = config.IMU_OFF180_DEF
        # self.off270 = config.IMU_OFF270_DEF
        #


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

    def getYawRaw(self):
        return self.mag.xy().angle()

    def readMagZBaseline(self):
        readingSum = 0
        readingCount = 0
        
        for _ in range(20):
            self.update()
            readingCount += 1
            readingSum += self.mag.z
            time.sleep(0.1)

        self.magZBaseline = readingSum / readingCount

    def readMagZThresh(self):
        readingSum = 0
        readingCount = 0
        
        for _ in range(20):
            self.update()
            readingCount += 1
            readingSum += self.mag.z
            time.sleep(0.1)

        self.magZThresh = readingSum / readingCount


    def hasHazard(self):
        if self.magZThresh > self.magZBaseline:
            if self.mag.z > self.magZThresh:
                self.hazCount += 1
            elif self.hazCount > 0:
                self.hazCount -= 1
        else:
            if self.mag.z < self.magZThresh:
                self.hazCount += 1
            elif self.hazCount > 0:
                self.hazCount -= 1

        if self.hazCount > 3:
            return True
        return False

    def printHazInfo(self):
        print("magZBase:", self.magZBaseline)
        print("magZThresh:", self.magZThresh)


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

