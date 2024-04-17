from src.subsystems.sensors.IMU import IMU
from src.subsystems.sensors.g_Ultrasonic import GroveUltrasonic
from src.subsystems.sensors.b_Ultrasonic import EV3Ultrasonic
from src.subsystems.sensors.DualIR import DualIR
import math
import config

class SensorArray:

    def __init__(self, BP, ultrasonicLeftPort, ultrasonicRightPort, frontUltrasonicPort): 
        self.BP = BP
        self.imu = IMU(config.MAG_HARD_VEC, config.MAG_SOFT_TRANS)
        self.imu.initialize()
        self.yaw = 0
        self.distLeft = GroveUltrasonic(ultrasonicLeftPort)
        self.distRight = GroveUltrasonic(ultrasonicRightPort)
        self.distFront = GroveUltrasonic(frontUltrasonicPort)

        self.irSense = DualIR()
        self.irReadingsL = [0.0] * 25
        self.irReadingsR = [0.0] * 25
        self.irIndex = 0

        self.targetMotorPos = 0


    def update(self):
        self.imu.update()

        read = self.irSense.IR_Read()
        self.irReadingsL[self.irIndex] = read[0]
        self.irReadingsR[self.irIndex] = read[1]
        self.irIndex = (self.irIndex + 1) % len(self.irReadingsL)

        # if self.atTargetRotorAng():
        #     self.BP.set_motor_power(self.rotorP, 0)
        # elif self.BP.get_motor_encoder(self.rotorP)  < self.targetMotorPos:
        #     self.BP.set_motor_power(self.rotorP, 25)
        # elif self.BP.get_motor_encoder(self.rotorP)  > self.targetMotorPos:
        #     self.BP.set_motor_power(self.rotorP, -25)

        return

    ### --- IR SENSOR --- ###

    # Returns the most recent IR reading. In range [0, 1.00]
    def getCurrentIRVal(self):
        return (self.irReadingsL[self.irIndex-1], self.irReadingsR[self.irIndex-1]) # -1 is fine because Python

    # Returns the average of the IR readings over the past 20 readings (about 0.4s)
    # In range [0, 1.0]
    def getAverageIRVal(self):
        return (sum(self.irReadingsL) / len(self.irReadingsL), sum(self.irReadingsR) / len(self.irReadingsR))
    
    def hasIRHazard(self):
        return sum(self.getAverageIRVal())/2 > config.IR_THRESH



    # Returns a tuple (leftDist, righDist, frontDist) in inches
    def readUltrasonics(self):
        return self.distLeft.getInches(), self.distRight.getInches(), self.distFront.getInches()

    ### --- ULTRASONIC ROTOR --- ###
    #
    # # Set rotor angle (radians)
    # def setTargetRotorAng(self, ang):
    #     self.targetMotorPos = -math.degrees(ang)
    #     # self.BP.set_motor_position(self.rotorP, self.targetMotorPos)
    #     #
    # # True if ultrasonic rotor is at the target angle. Else false
    # def atTargetRotorAng(self):
    #     return math.fabs(self.BP.get_motor_encoder(self.rotorP) - self.targetMotorPos) <= 5
    
    # Zeros yaw. Assumes that white is facing forward
    # def zeroUltrasonicRotor(self):
    #     self.BP.offset_motor_encoder(self.rotorP, self.BP.get_motor_encoder(self.rotorP))
    #
    # # Returns angle of rotor in radians
    # def getRotorAng(self):
    #     return -math.radians(self.BP.get_motor_encoder(self.rotorP))
    # 
    # # Returns angle of rotor in degrees
    # def getRotorAngDeg(self):
    #     return -self.BP.get_motor_encoder(self.rotorP)
