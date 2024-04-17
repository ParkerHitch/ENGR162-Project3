from lib.Vector2 import Vector2
import lib.RMath as rmath
from src.subsystems.Drivetrain import TwoWheel
from src.subsystems.Localization import KalmanLocalizer
from src.subsystems.SensorArray import SensorArray
from src.util.PID import *
import math
from src.subsystems.Mapping import Maze

# SCAN_FWD_BACK = 101
# SCAN_LEFT_RIGHT = 102

DRIVE_TO_NEXT_POINT = 201
SCANNING = 202
ROTATE_TO_ANGLE_PRE_MOVE = 203
ROTATE_TO_ANGLE_POST_MOVE = 204

HAZARD_BACKUP = 300

class GnC:

    def __init__(self, dt: TwoWheel, sense: SensorArray, loc: KalmanLocalizer, maze: Maze):
        self.dt = dt
        self.sense = sense
        self.loc = loc
        self.maze = maze
        self.rotPID = rotationPID(0.85, 0.02, 0.05, 0.3, math.radians(5), maxI=5)
        # self.rotPID = rotationPID(1, 0.02, 0.05, 0.3, math.radians(5), maxI=5)
        self.posPID = genericPID(0.5, 0, 0, 0.25, 0.25)
        self.dest = Vector2()
        self.mazePath = []
        self.mazeState = -1
        self.currentPathInd = 0

        self.readingCount = 0
        self.readingSumF = 0
        self.readingSumR = 0
        self.readingSumL = 0

    # returns true if at dest
    def beelineToDest(self) -> bool:
        diff = self.dest.sub(self.loc.pos)
        yawDiff = rmath.maxClamp(diff.mag(), (self.loc.getYaw() - diff.angle()))
        dir = Vector2.fromAngle(self.loc.getYaw())

        # print(diff)
        # print(dir)

        scalarDiff = diff.dot(dir)

        # print(scalarDiff)

        rot = self.rotPID.updateLoop(yawDiff)
        drive = (1 - 0.5*math.fabs(rot* (1/self.rotPID.max))) * - self.posPID.updateLoop(scalarDiff)

        self.dt.setPowers(drive - rot, drive + rot)
        
        return self.rotPID.atSetpoint() and self.posPID.atSetpoint()
    
    def turnAndDriveToDest(self) -> bool:
        diff = self.dest.sub(self.loc.pos)
        yawDiff = rmath.maxClamp(diff.mag(), (self.loc.getYaw() - diff.angle()))
        dir = Vector2.fromAngle(self.loc.getYaw())

        # print(diff)
        # print(dir)

        scalarDiff = diff.dot(dir)

        print(scalarDiff)
        print(math.degrees(yawDiff))
        print()

        rot = self.rotPID.updateLoop(yawDiff)
        drive = -self.posPID.updateLoop(scalarDiff)

        if not self.rotPID.atSetpoint():
            drive = 0

        self.dt.setPowers(drive - rot, drive + rot)
        
        return self.rotPID.atSetpoint() and self.posPID.atSetpoint()

    def rotateToAngle(self):
        rot = self.rotPID.updateLoop(self.loc.getYaw())
        self.dt.setPowers(-rot, rot)


    def setDest(self, dest: Vector2):
        self.rotPID.setDest(0)
        self.posPID.setDest(0)
        self.dest = dest

    def startScan(self):
        self.mazeState = SCANNING

        self.readingCount = 0
        self.readingSumF = 0
        self.readingSumR = 0
        self.readingSumL = 0

    def startRotatePreMove(self):
        if self.currentPathInd != len(self.mazePath)-1:
            self.startScan()
            dir = Vector2(self.mazePath[self.currentPathInd+1][0],self.mazePath[self.currentPathInd+1][1]).sub(
                Vector2(self.mazePath[self.currentPathInd][0],self.mazePath[self.currentPathInd][1])
            ).angle()
            self.rotPID.setDest(dir)
            self.mazeState = ROTATE_TO_ANGLE_PRE_MOVE
        else:
            self.startScan()

    def startRotatePostMove(self):
        self.startScan()
        dir = Vector2(self.mazePath[self.currentPathInd+1][0],self.mazePath[self.currentPathInd+1][1]).sub(
            Vector2(self.mazePath[self.currentPathInd][0],self.mazePath[self.currentPathInd][1])
        ).angle()
        self.rotPID.setDest(dir)
        self.mazeState = ROTATE_TO_ANGLE_POST_MOVE


    def startDriveToNext(self):
        self.setDest(Vector2(self.mazePath[self.currentPathInd+1][0],self.mazePath[self.currentPathInd+1][1]).mul(config.MAZE_GRID_SIZE))
        self.mazeState = DRIVE_TO_NEXT_POINT




    def irHazardDetected(self):
        self.maze.addIrHazard(self.mazePath[self.currentPathInd+1][0],self.mazePath[self.currentPathInd+1][1])
        self.startHazardBackup()

    def magHazardDetected(self):
        self.maze.addMagHazard(self.mazePath[self.currentPathInd+1][0],self.mazePath[self.currentPathInd+1][1])
        self.mazeState = HAZARD_BACKUP
        
    def startHazardBackup(self):
        self.setDest(Vector2(self.mazePath[self.currentPathInd][0],self.mazePath[self.currentPathInd][1]).mul(config.MAZE_GRID_SIZE))
        self.mazeState = HAZARD_BACKUP



    def startMazeFresh(self):
        self.loc.reset()
        self.startScan()

    # Should be called in enabledPeriodic
    def mainMazeLoop(self) -> bool:
        print("MAZESTATE", self.mazeState)
        
        self.sense.update() 

        if self.mazeState == -1:
            self.maze.print()
            return True
        
        # if self.mazeState == SCAN_FWD_BACK or self.mazeState == SCAN_LEFT_RIGHT:
        #     self.dt.setPowers(0,0)
        #     if self.sense.atTargetRotorAng():
        #         if self.readingCount1 < 10:
        #             whiteVal, blackVal = self.sense.readUltrasonics()
        #             self.readingCount1 += 1
        #             self.readingCount2 += 1
        #             self.readingSum1 += whiteVal
        #             self.readingSum2 += blackVal
        #         else:
        #             if self.mazeState == SCAN_FWD_BACK:
        #                 self.scanResult[0] = config.ULTRASONIC_ROTOR_RADIUS + self.readingSum1 / self.readingCount1
        #                 self.scanResult[2] = config.ULTRASONIC_ROTOR_RADIUS + self.readingSum2 / self.readingCount2
        #                 self.startScanLeftRight()
        #             else:
        #                 self.scanResult[1] = config.ULTRASONIC_ROTOR_RADIUS + self.readingSum1 / self.readingCount1
        #                 self.scanResult[3] = config.ULTRASONIC_ROTOR_RADIUS + self.readingSum2 / self.readingCount2
        #                 
        #                 # print(self.scanResult)
        #
        #                 correction = self.maze.updateMap(self.loc.pos, self.loc.getYaw(), self.scanResult)
        #                 self.loc.addPosCorrection(correction)
        #                 # print(correction)
        #                 # self.mazeState = -1
        #                 self.gotoNextMazePoint()
        
        if self.mazeState == ROTATE_TO_ANGLE_PRE_MOVE:
            self.loc.update(False)
            self.rotateToAngle()
            print(self.rotPID.currentError)
            print(self.rotPID.setpoint)
            print(self.loc.getYaw())
            if abs(self.rotPID.currentError) < math.radians(25):
                self.startDriveToNext()

        elif self.mazeState == DRIVE_TO_NEXT_POINT:
            self.loc.update()

            if self.sense.hasIRHazard():
                self.irHazardDetected()
                self.dt.setPowers(0,0)
                return False

            if self.dest.sub(self.loc.getPos()).mag() < 2:
                self.dt.setPowers(0,0)
                self.startRotatePostMove()
            else:
                self.beelineToDest()
        
        elif self.mazeState == HAZARD_BACKUP:
            self.loc.update()
            self.dt.setPowers(0,0)
            # if self.dest.sub(self.loc.getPos()).mag() < 2:
            #     self.startRotatePostMove()
            # else:
            #     self.beelineToDest() 


        elif self.mazeState == ROTATE_TO_ANGLE_POST_MOVE:
            self.loc.update(False)
            self.rotateToAngle()
            if self.rotPID.atSetpoint():
                self.startScan()

        elif self.mazeState == SCANNING:
            self.dt.setPowers(0,0)
            if self.readingCount < 10:
                ld, rd, fd = self.sense.readUltrasonics()
                self.readingCount += 1
                self.readingSumR += rd
                self.readingSumL += ld
                self.readingSumF += fd
            else:
                readings = [self.readingSumR, self.readingSumF, self.readingSumL]
                readings = [r/self.readingCount for r in readings]
                self.maze.update3Sense(self.loc.getPos(), self.loc.getYaw(), readings)
                self.genNewExplorationPlan(Maze.getNearestTileCoord(self.loc.getPos()), Maze.nearestDir(self.loc.getYaw()))
                self.startRotatePreMove()

        
        return False

    # Maze Planning
    # def gotoNextMazePoint(self):
    #     # always follow the left wall
    #     robotTile = Maze.getNearestTileCoord(self.loc.getPos())
    #     for dir in range(1, -3, -1):
    #         # if there isn't a wall in that direction
    #         if self.maze.isWallVec(robotTile, dir % 4) < 0.5:
    #             # go that way
    #             self.setDest(Maze.nexTileWorldPos(robotTile, dir % 4))
    #             self.mazeState = DRIVE_TO_NEXT_POINT
    #             return
    #             
    #     # if all of our surrounding walls are walls then re-scan
    #     self.startScanFwdBack()

    # updates mazePath to be a plan to reach the nearest completely unexplored area
    def genNewExplorationPlan(self, startingPoint: Vector2, startDir=0):
        currentTile = [round(startingPoint.x), round(startingPoint.y)]
        currentDir = startDir
        self.mazePath = []
        while not self.maze.isCompletelyUnexplored(currentTile[0], currentTile[1]):
            i = self.pathInd(currentTile[0], currentTile[1])
            if i != -1:
                # We have already visited the current tile. Remove all items after it, as they are redundant
                self.mazePath = self.mazePath[:i+1]
            else:
                self.mazePath.append((currentTile[0], currentTile[1]))
            for d in range(currentDir+3,currentDir+7):
                dir = d % 4
                # If is open. Assume all unknown walls are open
                if self.maze.isWall(currentTile[0], currentTile[1], dir) < config.MAZE_UNKNOWN_UPPER:
                    potentialNext = Maze.nextTile(currentTile[0], currentTile[1], dir)
                    if self.maze.isSafe(potentialNext):
                        currentTile = potentialNext
                        currentDir = dir
                        break
        print(self.mazePath)
        self.currentPathInd = 0

    def pathInd(self, tileX, tileY):
        i = 0
        for item in self.mazePath:
            if item[0]==tileX and item[1]==tileY:
                return i
        return -1




    #
    # def genForSegment(self, startingPoint, startingDir):
    #     possiblyOpen = [False, False, False, False]
    #     currentTile = startingPoint
    #     currentDir = 
    #     while True:
    #         possiblyOpen = self.maze.getPossibleOpen(currentTile[0], currentTile[1])
    #         if sum(possiblyOpen) == 1:
    #             for dir in range(4)
    #
