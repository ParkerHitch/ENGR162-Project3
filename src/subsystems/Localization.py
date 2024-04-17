from src.subsystems.SensorArray import SensorArray
from src.subsystems.Drivetrain import TwoWheel
from lib.Vector2 import Vector2
import time
import config
import math
import numpy as np
import rtime as rt
from src.subsystems.Mapping import Maze

class KalmanLocalizer:
    
    # State vector:
    # 0- x
    # 1- y
    # 2- theta
    # 3- xdot
    # 4- ydot
    # 5- thetadot
    # 6- xdoubledot
    # 7- ydoubledot

    # Control (u) vector:
    # 0- lefAngtVel
    # 1- rightAngVel

    # Measurement (z) vector:
    # 0- x
    # 1- y
    # 2- theta
    # 3- thetadot
    # 4- xdoubledot
    # 5- ydoubledot

    def __init__(self, sensorArray: SensorArray, drivetrain: TwoWheel):
        self.sensors = sensorArray
        self.dt = drivetrain
        self.reset()

    def reset(self):
        
        self.pos = Vector2()
        self.yaw = 0
        self.lastUpdated = time.time_ns()
        # Actual State Vector
        self.state = np.zeros((8, 1))
        # Covariance matrix. IMPORTANT
        self.covMat = np.eye(8) * 0.01
        # TODO: FIGURE OUT INITIAL VALUES

        # Jacobian of f(x,y). Used in predict step
        self.matF = np.zeros((8, 8))
        # Initialize parts of matrix that are just 1
        self.matF[0,0] = 1
        self.matF[1,1] = 1
        self.matF[2,2] = 1
        self.matF[6,6] = 1
        self.matF[7,7] = 1

        # Initialize Q (process noise) matrix:
        self.matQ = np.eye(8, dtype=float) * 0.05
        self.matR = np.eye(6, dtype=float) * 0.03
        # and R
        self.matR[0,0] = 0.1
        self.matR[1,1] = 0.1
        self.matR[2,2] = 0.2

        # Initialize h matrix. Transition from x to z.
        self.matH = np.zeros((6,8))
        self.matH[0,0] = 1
        self.matH[1,1] = 1
        self.matH[2,2] = 1
        self.matH[3,5] = 1
        self.matH[4,6] = 1
        self.matH[5,7] = 1

        self.prevDistR = None
        self.prevDistL = None
        self.prevDistF = None

        self.motRPos = math.radians(self.dt.getREncoder())
        self.motLPos = math.radians(self.dt.getLEncoder())

    def resetReadings(self):
        self.prevDistR = None
        self.prevDistL = None
        self.prevDistF = None

    # Updates our location. see ekfUpdate for the ekf update step
    def update(self, doUpdate=True):
        newRPos = math.radians(self.dt.getREncoder())
        newLPos = math.radians(self.dt.getLEncoder())

        # Update position based on encoders

        tdR = (newRPos - self.motRPos)/rt.dt
        tdL = (newLPos - self.motLPos)/rt.dt

        self.motRPos = newRPos
        self.motLPos = newLPos

        self.ekfPredict([tdR, tdL])

        if not doUpdate:
            return
        
        distLeft, distRight, distFront = self.sensors.readUltrasonics()
        # print("Distances", distLeft, distRight, distFront)
        # print(self.getPos())
        measuredPos, xUnc, yUnc = self.threeSenseLocalize(self.getPos(), self.getYaw(), distLeft, distRight, distFront)
        self.matR[0,0] = xUnc
        self.matR[1,1] = yUnc
        self.matR[2,2] = 0.75
        # print("R", self.matR)
        # measuredPos = self.getPos()

        # Accel in worldspace
        accel = self.sensors.imu.getPlanarAccel().rotate(self.getYaw())

        # print("Accel", accel)

        # self.ekfUpdate(np.array([ measuredPos.x, measuredPos.y, self.sensors.imu.getYaw(), self.sensors.imu.getYawRate(), accel.x, accel.y]))
        self.ekfUpdate(np.array([ measuredPos.x, measuredPos.y, 0, self.sensors.imu.getYawRate(), accel.x, accel.y]))
        return

    def getPos(self):
        return Vector2(self.state[0, 0], self.state[1, 0])

    def getYaw(self):
        return self.state[2, 0]

    # updates state vector to be a prediction according to wheel velocities
    def ekfPredict(self, u):
        v = np.sum(u)*config.WHEEL_RADIUS/2
        thetaDot = (u[1] - u[0]) * config.WHEEL_RADIUS / (2 * config.WHEEL_SEPARATION)
        xDot = v * math.cos(self.state[2, 0] + thetaDot/2)
        yDot = v * math.sin(self.state[2, 0] + thetaDot/2)

        # Performing f(x, u)
        self.state[0, 0] = self.state[0,0] + (xDot * rt.dt) + (self.state[6,0] * rt.dt * rt.dt / 2)
        self.state[1,0 ] = self.state[1,0] + (yDot * rt.dt) + (self.state[7,0] * rt.dt * rt.dt / 2)
        self.state[2,0] = self.state[2,0] + thetaDot * rt.dt
        self.state[3,0] = xDot
        self.state[4,0] = yDot
        self.state[5,0] = thetaDot
        self.state[6,0] = self.state[6,0]
        self.state[7,0] = self.state[7,0]

        # Update the F matrix
        # since derivative of cos is -sin we can use yDot    
        self.matF[0,2] = -rt.dt * yDot
        self.matF[0,6] = rt.dt * rt.dt / 2
        # same here
        self.matF[1,2] = rt.dt * xDot
        self.matF[1,7] = rt.dt * rt.dt / 2
        # Aaand same here
        self.matF[3,2] = -yDot
        self.matF[4,2] = xDot
        # other rows just have 1 or are 0. set in __init__

        # Calculate process noise q.
        # NEED MORE RESEARCH
        # Should probably be a constant

        # update covariance matrix
        self.covMat = np.matmul(np.matmul(self.matF, self.covMat), np.transpose(self.matF)) + self.matQ

    # Performs the update step based on a measurement and measurement noise
    def ekfUpdate(self, z):
        z.shape = (6,1)
        # print("stateI",self.state)
        # print("z",z)
        # print("h", self.matH)
        y = z - np.matmul(self.matH, self.state)

        # print("y",y)
        #print(np.matmul(np.matmul(self.matH, self.covMat), self.matH.T) + self.matR)

        K = np.matmul(np.matmul(self.covMat, self.matH.T), 
                      np.linalg.inv(np.matmul(np.matmul(self.matH, self.covMat), self.matH.T) + self.matR))

        # print("K",K)

        self.state = self.state + np.matmul(K, y)

        self.covMat = np.matmul(np.eye(8) - np.matmul(K,self.matH),self.covMat)


    # NIGHTMARE NIGHTMARE NIGHTMARE NIGHTMARE NIGHTMARE
    def threeSenseLocalize(self, robotPos: Vector2, yaw: float, distLeft: float, distRight: float, distFront: float):
        
        dr = 0
        dl = 0
        df = 0

        if self.prevDistR != None:
            dr = distRight - self.prevDistR
        if self.prevDistL != None:
            dl = distLeft - self.prevDistL
        if self.prevDistF != None:
            df = distFront - self.prevDistF
        self.prevDistR = distRight
        self.prevDistL = distLeft
        self.prevDistF = distFront

        dr = dr/rt.dt
        dl = dl/rt.dt
        df = df/rt.dt

        nearestYaw = round(yaw / (math.pi/2)) % 4
        if nearestYaw%2 == 0:
            # lrCol = round((robotPos.x + config.ULTRASONIC_LR_X_OFFSET) / config.MAZE_GRID_SIZE)
            # robotRow = round(robotPos.y / config.MAZE_GRID_SIZE)
            
            readingX = robotPos.x + (config.ULTRASONIC_FORWARD_OFFSET + distFront) * math.cos(yaw)
            readingLY = robotPos.y + (config.ULTRASONIC_LR_Y_OFFSET + distLeft ) * math.cos(yaw) + config.ULTRASONIC_LR_X_OFFSET * math.sin(yaw)
            readingRY = robotPos.y - (config.ULTRASONIC_LR_Y_OFFSET + distRight) * math.cos(yaw) + config.ULTRASONIC_LR_X_OFFSET * math.sin(yaw)

            tileCoordFX = round((readingX  - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            tileCoordLY = round((readingLY - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            tileCoordRY = round((readingRY - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            
            expectedReadingX  = tileCoordFX * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)
            expectedReadingLY = tileCoordLY * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)
            expectedReadingRY = tileCoordRY * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)

            diffX = expectedReadingX - readingX
            diffYR = expectedReadingRY - readingRY
            diffYL = expectedReadingLY - readingLY
            diffY = (diffYR*distLeft + diffYL*distRight) / (distLeft + distRight)
            # print("readingX", readingX)
            # print("readingLY", readingLY)
            # print("readingRY", readingRY)
            #
            # print("TileFX", tileCoordFX)
            # print("TileLY", tileCoordLY)
            # print("TileRY", tileCoordRY)
            #
            # print("expectedX", expectedReadingX)
            # print("expectedLY", expectedReadingLY)
            # print("expectedRY", expectedReadingRY)

            # uncertanty calculation. larger distance or larger derivative = more uncertain
            xDist = config.ULTRASONIC_FORWARD_OFFSET+abs(distFront) 
            yDist = config.ULTRASONIC_LR_Y_OFFSET+max(abs(distLeft), abs(distRight))

            dx = abs(df)
            dy = max(abs(dr),abs(dl))

            xUnc = (1+dx**3)*(6*xDist)/(4*config.MAZE_GRID_SIZE)
            yUnc = (1+dy**3)*(6*yDist)/(4*config.MAZE_GRID_SIZE)

            return robotPos.add(Vector2(diffX, diffY)), xUnc, yUnc
        else:
            # lrCol = round((robotPos.x + config.ULTRASONIC_LR_X_OFFSET) / config.MAZE_GRID_SIZE)
            # robotRow = round(robotPos.y / config.MAZE_GRID_SIZE)
            
            readingY = robotPos.y + (config.ULTRASONIC_FORWARD_OFFSET + distFront) * math.cos(yaw)
            readingLX = robotPos.x + (config.ULTRASONIC_LR_Y_OFFSET + distLeft ) * math.cos(yaw) + config.ULTRASONIC_LR_X_OFFSET * math.sin(yaw)
            readingRX = robotPos.x - (config.ULTRASONIC_LR_Y_OFFSET + distRight) * math.cos(yaw) + config.ULTRASONIC_LR_X_OFFSET * math.sin(yaw)

            tileCoordFY = round((readingY  - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            tileCoordLX = round((readingLX - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            tileCoordRX = round((readingRX - (config.MAZE_GRID_SIZE/2)) /  config.MAZE_GRID_SIZE)
            
            expectedReadingY  = tileCoordFY * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)
            expectedReadingLX = tileCoordLX * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)
            expectedReadingRX = tileCoordRX * config.MAZE_GRID_SIZE + (config.MAZE_GRID_SIZE/2)

            diffY = expectedReadingY - readingY
            diffXR = expectedReadingRX - readingRX
            diffXL = expectedReadingLX - readingLX
            diffX = (diffXR*distLeft + diffXL*distRight) / (distLeft + distRight)
     
            # print("readingX", readingY)
            # print("readingLY", readingLX)
            # print("readingRY", readingRX)
            #
            # print("TileFX", tileCoordFY)
            # print("TileLY", tileCoordLX)
            # print("TileRY", tileCoordRX)
            #
            # print("expectedX", expectedReadingY)
            # print("expectedLY", expectedReadingLX)
            # print("expectedRY", expectedReadingRX)
            
            xDist = config.ULTRASONIC_LR_Y_OFFSET+max(abs(distLeft), abs(distRight))
            yDist = config.ULTRASONIC_FORWARD_OFFSET+abs(distFront) 

            dx = max(abs(dr),abs(dl))
            dy = abs(df)

            xUnc = (1+dx**3)*(6*xDist)/(4*config.MAZE_GRID_SIZE)
            yUnc = (1+dy**3)*(6*yDist)/(4*config.MAZE_GRID_SIZE)

            return robotPos.add(Vector2(diffX, diffY)), xUnc, yUnc

        

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

    def addPosCorrection(self, correction:Vector2):
        self.pos = self.pos.add(correction)

    def getPos(self):
        return self.pos
