from lib.Vector2 import Vector2
from typing import List
import math
import config
import lib.RMath as rmath

# Maze composed of tiles and walls
# Each tile has 4 walls that can either be open or closed

class Maze:
        
    # vertWalls is an array of arrays of the probabilities of all vertical walls in a row
    # so vertWalls[R] is an array of the probabilities of left/right walls existing in row R
    # horzWalls is an array of arrays of the probabilities of all horizontal walls in a column
    # so horzWalls[C] is an array of the probabilites of top/bottom walls existing in column C

    # vertWalls[0][0] = the probability of left wall of firstTile
    # horzWalls[0][0] = the probability of top wall of firstTile

    # firstTileY = top
    # lastTileY = bottom

    firstTileX: int = 0
    firstTileY: int = 0
    lastTileX: int = 0
    lastTileY: int = 0

    # def __init__(self, fillExpansionsWith=0.5):
    #     self.vertWalls = [[0.5, 0.5]]
    #     self.horzWalls = [[0.5, 0.5]]
    #     self.defaultFill = fillExpansionsWith

    # tilesW = initall space to allocate for number of tiles in x-component
    # tilesH = initall space to allocate for number of tiles in y-component
    def __init__(self, tilesW=1, tilesH=1, startFill=0.5, fillExpansions=0.5, centerX=0, centerY=0):
        self.vertWalls = []
        for i in range(tilesH):
            self.vertWalls.append([startFill]*(tilesW+1))

        self.horzWalls = []
        for i in range(tilesW):
            self.horzWalls.append([startFill]*(tilesH+1))
        
        self.defaultFill = fillExpansions

        self.magHazards = []
        self.irHazards  = []
        
        if tilesW%2 == 1:
            self.firstTileX = centerX - (tilesW//2)
            self.lastTileX  = centerX + (tilesW//2)
        else:
            self.firstTileX = centerX - (tilesW//2) + 1
            self.lastTileX  = centerX + (tilesW//2)

        if tilesH%2 == 1:
            self.firstTileY = centerY + (tilesW//2)
            self.lastTileY  = centerY - (tilesW//2)
        else:
            self.firstTileY = centerY + (tilesW//2) - 1
            self.lastTileY  = centerY - (tilesW//2)

    # def getWall(self, tileX: int, tileY: int, dir: int) -> float:
    #     return


    # Params: x&y pos of tile. dir: 0 = +x, 1 = +y, 2 = -x, 3 = -y
    def isWall(self, tileX: int, tileY: int, dir: int) -> float:
        if (dir%2)==0:
            return self.vertWalls[self.firstTileY -  tileY][tileX - self.firstTileX + 1 - (dir//2)]
        else:
            return self.horzWalls[tileX - self.firstTileX][self.firstTileY - tileY + (dir//2)]
        
    def isWallVec(self, tileVec: Vector2, dir: int) -> float:
        return self.isWall(tileVec.x, tileVec.y, dir)

    def getPossibleOpen(self, tileX, tileY):
        return [ self.isWall(tileX, tileY, d) < config.MAZE_UNKNOWN_UPPER for d in range(4)]
    def isCompletelyUnexplored(self, tileX, tileY):
        count = 0
        for d in range(4):
            w = self.isWall(tileX, tileY, d)
            if w < config.MAZE_UNKNOWN_UPPER and w > config.MAZE_UNKNOWN_LOWER:
                count += 1
        return count == 4

    # Params: x&y pos of tile. dir: 0 = +x, 1 = +y, 2 = -x, 3 = -y
    def setWallUnsafe(self, tileX: int, tileY: int, dir: int, val: float):
        if (dir%2)==0:
            self.vertWalls[self.firstTileY -  tileY] \
                          [tileX - self.firstTileX + 1 - (dir//2)] = val
        else:
            self.horzWalls[tileX - self.firstTileX] \
                          [self.firstTileY - tileY + (dir//2)] = val

    def setWall(self, tileX: int, tileY: int, dir: int, val: float):
        if(tileX > self.lastTileX):
            for row in self.vertWalls:
                row.extend([self.defaultFill] * (tileX - self.lastTileX))
            # bad
            # self.horzWalls.extend(
            #     [[self.defaultFill] * len(self.horzWalls[0]) ] * (tileX - self.lastTileX))
            # cannot do this because [[1,2]] * 3 = an array containing 3 REFERENCES to the [1,2] array so all get changed when change 1
            # need to successively append to create new objects
            for i in range(tileX - self.firstTileX):
                self.horzWalls.append([self.defaultFill] * len(self.horzWalls[0]))
    
            self.lastTileX = tileX
        elif(tileX < self.firstTileX):
            for i in range(self.firstTileX - tileX):
                for row in self.vertWalls:
                    row.insert(0, self.defaultFill)
                self.horzWalls.insert(0, [self.defaultFill] * len(self.horzWalls[0]))
            self.firstTileX = tileX
        
        if(tileY < self.lastTileY):
            # self.vertWalls.extend(
            #     [ [self.defaultFill] * len(self.vertWalls[0]) ] * (self.lastTileY - tileY))
            for i in range(self.lastTileY - tileY):
                self.vertWalls.append([self.defaultFill] * len(self.vertWalls[0]))
            for col in self.horzWalls:
                col.extend([self.defaultFill] * (self.lastTileY - tileY))

            self.lastTileY = tileY
        elif(tileY > self.firstTileY):
            for i in range(tileY - self.firstTileY):
                for col in self.horzWalls:
                    col.insert(0, self.defaultFill)
                self.vertWalls.insert(0, [self.defaultFill] * len(self.vertWalls[0]))
            self.firstTileY = tileY

        self.setWallUnsafe(tileX, tileY, dir, val)


    def print(self):
        for i in range(len(self.vertWalls)):
            self.printHorzRow(i)
            self.printVertRow(i)
        self.printHorzRow(len(self.vertWalls))
            
    def printHorzRow(self, row):
        for i in range(len(self.horzWalls)):
            print(" ", end="")
            val = self.horzWalls[i][row]
            print(" " if val<=0.5 else "-", end="")
        print(" ")

    def printVertRow(self, row):
        for val in self.vertWalls[row]:
            print(" " if val<=0.5 else "|", end="")
            print(" ", end="")
        print()

# actual intense logic
    
    @staticmethod
    def getNearestTileCoord(realPos: Vector2):
        return Vector2(
            round(realPos.x / config.MAZE_GRID_SIZE),
            round(realPos.y / config.MAZE_GRID_SIZE)
        )
    
    @staticmethod
    def nexTileWorldPos(initialTile: Vector2, dir: int):
        return initialTile.mul(config.MAZE_GRID_SIZE).add(Vector2(config.MAZE_GRID_SIZE, 0).rotate(dir * math.pi/2))
    @staticmethod
    def nextTile(tileX, tileY, dir):
        if dir == 0:
            return [tileX + 1, tileY]
        elif dir == 1:
            return [tileX, tileY + 1]
        elif dir == 2:
            return [tileX - 1, tileY]
        else:
            return [tileX, tileY - 1]
    
    def threeSenseLocalize(self, robotPos: Vector2, yaw: float, distLeft: float, distRight: float, distFront: float):
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
            diffY = (diffYR*abs(distLeft) + diffYL*abs(distRight)) / (abs(distLeft) + abs(distRight))

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

            xDist = config.ULTRASONIC_FORWARD_OFFSET+abs(distFront) 
            yDist = config.ULTRASONIC_LR_Y_OFFSET+max(abs(distLeft), abs(distRight))

            xUnc = (6*xDist)/(4*config.MAZE_GRID_SIZE)
            yUnc = (3*yDist)/(4*config.MAZE_GRID_SIZE)

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
            diffX = (diffXR*abs(distLeft) + diffXL*abs(distRight)) / (abs(distLeft) + abs(distRight))
     
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

            xUnc = (3*xDist)/(4*config.MAZE_GRID_SIZE)
            yUnc = (6*yDist)/(4*config.MAZE_GRID_SIZE)


            return robotPos.add(Vector2(diffX, diffY)), xUnc, yUnc
 

    #
    # # updates the map based on robot location orientation and scan result
    # # only updates walls around the tile the robot is currently in
    # # Also returns a position correction vector. 
    # # To use, add the returned vector to the robot's current position to get its new position
    # def updateMap(self, robotPos: Vector2, robotYaw: float, scanResult: List[float]) -> Vector2:
    #     scanOrigin = robotPos.add(config.ULTRASONIC_ROTOR_OFFSET.rotate(robotYaw))
    #     nearestTileX = round(robotPos.x / config.MAZE_GRID_SIZE)
    #     nearestTileY = round(robotPos.y / config.MAZE_GRID_SIZE)
    #     
    #     # Expected if surrounded by walls
    #     expectedScan = [(nearestTileX*config.MAZE_GRID_SIZE) + config.MAZE_GRID_SIZE/2 - scanOrigin.x,
    #                     (nearestTileY*config.MAZE_GRID_SIZE) + config.MAZE_GRID_SIZE/2 - scanOrigin.y,
    #                     scanOrigin.x - ((nearestTileX*config.MAZE_GRID_SIZE) - config.MAZE_GRID_SIZE/2),
    #                     scanOrigin.y - ((nearestTileY*config.MAZE_GRID_SIZE) - config.MAZE_GRID_SIZE/2)]
    #
    #     scanDelta = [
    #             expectedScan[0] - scanResult[0],
    #             expectedScan[1] - scanResult[1],
    #             scanResult[2] - expectedScan[2],
    #             scanResult[3] - expectedScan[3]]
    #     
    #     # print(scanDelta)
    #
    #     for dir in range(4):
    #         if scanResult[dir] > 1.5 * (expectedScan[dir]):
    #             self.setWall(nearestTileX,nearestTileY, dir, 0)
    #             scanDelta[dir] = 0
    #         else:
    #             self.setWall(nearestTileX,nearestTileY, dir, 1)
    #
    #     correction = Vector2(
    #         (scanDelta[0] + scanDelta[2]) / 2,
    #         (scanDelta[1] + scanDelta[3]) / 2
    #     )
    #     return correction

   
    # # Updates map and returns an expected location
    # def localizeAndMap(self, robotPos: Vector2, posUncertanty, robotAng: float, rotorAng: float, sensorReadingW: float, sensorReadingB: float):
    #     expected = robotPos
    #     count = 1
    #     if sensorReadingW != None:
    #         expected.add( self.locAndMapHalf(robotPos, posUncertanty, robotAng, rotorAng, sensorReadingW))
    #         count += 1
    #     if sensorReadingB != None:
    #         expected.add( self.locAndMapHalf(robotPos, posUncertanty, robotAng, rotorAng+ math.pi/2, sensorReadingB))
    #         count += 1
    #     expected = expected.mul(1/count)
    #     return expected
    #     
    #
    #
    # def locAndMapHalf(self, robotPos: Vector2, posUncertanty, robotAng: float, rotorAng: float, sensorReadingW: float):
    #     increment = Vector2.fromAngle(robotAng + rotorAng)
    #     currentTileW = Maze.getNearestTileCoord(robotPos)
    #     currentPos = Vector2(robotPos.x, robotPos.y)
    #
    #     totalDist = 0
    #     distEW = 0
    #     distNS = 0
    #     walls = []
    #     wallDists = []
    #     closest = -1
    #     while totalDist < sensorReadingW + 2.5*config.G_ULTRASONIC_STDEV:
    #         #print(totalDist)
    #         realativePos = currentPos.sub(currentTileW.mul(config.MAZE_GRID_SIZE))
    #         #print(realativePos)
    #         if increment.x != 0:
    #             if increment.x > 0:
    #                 distEW = ((config.MAZE_GRID_SIZE/2) - realativePos.x) / increment.x
    #             else:
    #                 distEW = ((-config.MAZE_GRID_SIZE/2) - realativePos.x) / increment.x
    #         else:
    #             distEW = 999999
    #         if increment.y != 0:
    #             if increment.y > 0:
    #                 distNS = ((config.MAZE_GRID_SIZE/2) - realativePos.y) / increment.y
    #             else:
    #                 distNS= ((-config.MAZE_GRID_SIZE/2) - realativePos.y) / increment.y
    #         else:
    #             distNS = 999999
    #         if abs(distEW) < abs(distNS):
    #             # move L/R
    #             walls.append([currentTileW.x, currentTileW.y, 0 if increment.x > 0 else 2])
    #             traverse = increment.mul(distEW)
    #             totalDist += traverse.mag()
    #             wallDists.append(totalDist)
    #             currentTileW.x += 1 if increment.x > 0 else -1
    #             currentPos = currentPos.add(traverse)
    #             if closest == -1:
    #                 closest = 0
    #             elif abs(wallDists[closest]-sensorReadingW) > abs(totalDist-sensorReadingW):
    #                 closest = len(wallDists)-1
    #         else:
    #             # move U/D
    #             walls.append([currentTileW.x, currentTileW.y, 1 if increment.y > 0 else 3])
    #             traverse = increment.mul(distNS)
    #             totalDist += traverse.mag()
    #             wallDists.append(totalDist)
    #             currentTileW.y += 1 if increment.y > 0 else -1
    #             currentPos = currentPos.add(traverse)
    #             if closest == -1:
    #                 closest = 0
    #             elif abs(wallDists[closest]-sensorReadingW) > abs(totalDist-sensorReadingW):
    #                 closest = len(wallDists)-1
    #     
    #     for i in range(closest):
    #         self.setWall(walls[i][0], walls[i][1], walls[i][2], 0.5 * self.isWall(walls[i][0], walls[i][1], walls[i][2]))
    #     self.setWall(walls[closest][0], walls[closest][1], walls[closest][2], 1.5 * self.isWall(walls[closest][0], walls[closest][1], walls[closest][2]))
    #     expectedWhite = wallDists[closest]
    #     return robotPos.add(increment.mul(expectedWhite-sensorReadingW))

        

