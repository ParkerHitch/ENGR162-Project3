from src.subsystems.sensors.IMU import IMU
from src.subsystems.sensors.g_Ultrasonic import GroveUltrasonic
import math
import config

class SensorArray:

    def __init__(self, BP, ultrasonicRotorPort, ultrasonWhtPort, ultrasonBlkPort):
        self.BP = BP
        # self.imu = IMU()
        # self.imu.initialize()
        self.rotorP = ultrasonicRotorPort
        self.yaw = 0
        self.distBlk = GroveUltrasonic(ultrasonBlkPort)
        self.distWht = GroveUltrasonic(ultrasonWhtPort)

        self.targetMotorPos = 0


    def update(self):
        return
        # self.imu.update()

    # Set rotor angle (radians)
    def setTargetRotorAng(self, ang):
        self.targetMotorPos = -math.degrees(ang)
        self.BP.set_motor_position(self.rotorP, self.targetMotorPos)

    # True if ultrasonic rotor is at the target angle. Else false
    def atTargetRotorAng(self):
        return math.fabs(self.BP.get_motor_encoder(self.rotorP) - self.targetMotorPos) <= 5
    
    # Returns a tuple whiteDist, blackDist in inches
    def readUltrasonics(self):
        return self.distWht.getInches(), self.distBlk.getInches()

    # Zeros yaw. Assumes that white is facing forward
    def zeroUltrasonicRotor(self):
        self.BP.offset_motor_encoder(self.rotorP, self.BP.get_motor_encoder(self.rotorP))

    # Returns angle of rotor in radians
    def getRotorAng(self):
        return -math.radians(self.BP.get_motor_encoder(self.rotorP))
    
    # Returns angle of rotor in degrees
    def getRotorAngDeg(self):
        return -self.BP.get_motor_encoder(self.rotorP)