from src.subsystems.SensorArray import SensorArray
from src.subsystems.Drivetrain import TwoWheel
from lib.Vector2 import Vector2
import time
import config
import math

class NaiveLocalizer:

    def __init__(self, sensorArray: SensorArray, drivetrain: TwoWheel):
        self.sensors = sensorArray
        self.pos = Vector2()
        self.yaw = 0
        self.yawOffset = 0
        self.dt = drivetrain
        self.lastUpdated = time.time_ns()

        self.motRPos = math.radians(self.dt.getREncoder())
        self.motLPos = math.radians(self.dt.getLEncoder())


    def update(self):

        now = time.time_ns()
        # convert to seconds
        deltaT = (now - self.lastUpdated) / (1e9)
        self.lastUpdated = now

        self.sensors.update()
        # self.yaw = self.sensors.imu.getMag().xy().angle() - self.yawOffset

        newRPos = math.radians(self.dt.getREncoder())
        newLPos = math.radians(self.dt.getLEncoder())

        # Update position based on encoders

        tdR = (newRPos - self.motRPos)/deltaT
        tdL = (newLPos - self.motLPos)/deltaT

        self.motRPos = newRPos
        self.motLPos = newLPos

        thetaDot = (tdR - tdL) * config.WHEEL_RADIUS / config.WHEEL_SEPARATION
        avgSpd   = (tdR + tdL) * config.WHEEL_RADIUS / 2
        xDot = avgSpd * math.cos(self.yaw + thetaDot/2)
        yDot = avgSpd * math.sin(self.yaw + thetaDot/2)
        
        self.pos.x += xDot * deltaT
        self.pos.y += yDot * deltaT
        self.yaw += thetaDot * deltaT

    def zeroYaw(self):
        self.yaw = 0
        # self.yawOffset = self.sensors.imu.getMag().xy().angle()

    def getYaw(self):
        return self.yaw
    

    def zeroPos(self):
        self.pos.x = 0.0
        self.pos.y = 0.0

    def getPos(self):
        return self.pos