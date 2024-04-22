from lib.Vector2 import Vector2
import math
import config

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
        self.exitPoint  = None
        
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
    def isCompletelyExplored(self, tileX, tileY):
        count = 0
        for d in range(4):
            w = self.isWall(tileX, tileY, d)
            if w < config.MAZE_UNKNOWN_UPPER and w > config.MAZE_UNKNOWN_LOWER:
                count += 1
        return count == 0

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

    def setEndpoint(self, tileX, tileY):
        self.exitPoint = [tileX, tileY]

    def addIrHazard(self, tileX, tileY, strength):
        self.irHazards.append((tileX, tileY, strength))

    def addMagHazard(self, tileX, tileY, strength):
        self.magHazards.append((tileX, tileY, strength))

    def isSafe(self, tile) -> bool:
        for hazT in [self.irHazards, self.magHazards]:
            for haz in hazT:
                if tile[0]==haz[0] and tile[1]==haz[1]:
                    return False
        return True

    def isIr(self, tileX, tileY):
        for haz in self.irHazards:
            if tileX==haz[0] and tileY==haz[1]:
                return True
        return False

    def isMag(self, tileX, tileY):
        for haz in self.magHazards:
            if tileX==haz[0] and tileY==haz[1]:
                return True
        return False

    def isExit(self, tileX, tileY):
        if self.exitPoint != None:
            if tileX == self.exitPoint[0] and tileY == self.exitPoint[1]:
                return True
        return False

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

    
    def coordsRelativeToLL(self, tileX, tileY):
        return (tileX - self.firstTileX, tileY - self.lastTileY)


    def trimSelf(self):
        newVertWalls = []
        newHorzWalls = []
        newFirstY = 0
        newLastY = self.lastTileY
        newFirstX = 0
        newLastX = self.lastTileX

        for y in range(self.firstTileY, 0, -1):
            for x in range(self.firstTileX, self.lastTileX+1):
                if not self.isCompletelyUnexplored(x, y):
                    newFirstY = y
                    break
                if self.isIr(x,y) or self.isMag(x,y) or self.isExit(x,y):
                    newFirstY = y
                    break
            else:
                continue
            break

        for y in range(newFirstY, self.lastTileY - 1, -1):
            for x in range(self.firstTileX, self.lastTileX+1):
                if not self.isCompletelyUnexplored(x, y):
                    # print("partialX",y)
                    break
                if self.isIr(x,y) or self.isMag(x,y) or self.isExit(x,y):
                    # print("obj",y)
                    break
            else:
                newLastY = y + 1
                break
            continue

        newVertWalls = self.vertWalls[self.firstTileY-newFirstY : self.firstTileY-newLastY + 1]
        for c in range(0, len(self.horzWalls)):
            newHorzWalls.append(self.horzWalls[c][self.firstTileY-newFirstY : self.firstTileY-newLastY + 2])

        self.firstTileY = newFirstY
        self.lastTileY = newLastY 
        self.vertWalls = newVertWalls
        self.horzWalls = newHorzWalls

        # self.printRealFormat()

        # print(newVertWalls)
        # print(newHorzWalls)
        for x in range(self.firstTileX, 0):
            for y in range(self.firstTileY, self.lastTileY-1, -1):
                # print(x,y)
                if not self.isCompletelyUnexplored(x, y):
                    newFirstX = x
                    break
                if self.isIr(x,y) or self.isMag(x,y) or self.isExit(x,y):
                    newFirstX = x
                    break
            else:
                continue
            break

        for x in range(newFirstX, self.lastTileX + 1):
            for y in range(self.firstTileY, self.lastTileY-1, -1):
                if not self.isCompletelyUnexplored(x, y):
                    # print("partialX",x)
                    break
                if self.isIr(x,y) or self.isMag(x,y) or self.isExit(x,y):
                    # print("obj",x)
                    break
            else:
                newLastX = x - 1
                break
            continue

        newHorzWalls = []
        newVertWalls = []

        newHorzWalls = self.horzWalls[newFirstX - self.firstTileX : newLastX - self.firstTileX + 1]
        for r in range(0, len(self.vertWalls)):
            newVertWalls.append(self.vertWalls[r][newFirstX - self.firstTileX: newLastX - self.firstTileX + 2])

        # print(newHorzWalls)
        # print(newVertWalls)

        self.firstTileX = newFirstX
        self.lastTileX = newLastX
        self.vertWalls = newVertWalls
        self.horzWalls = newHorzWalls


        # print(newFirstX, newLastX)
        # print(newFirstY, newLastY)



    def printRealFormat(self):
        print(self.toStrRealFormat())

    def toStrRealFormat(self):
        str = ""
        # Reminder that first and last are both inclusive
        for y in range(self.firstTileY, self.lastTileY-1, -1):
            for x in range(self.firstTileX, self.lastTileX+1):
                if x==0 and y==0:
                    str += '5'
                elif self.isIr(x,y):
                    str += '2'
                elif self.isMag(x,y):
                    str += '3'
                elif self.isExit(x, y):
                    str += '4'
                elif self.isCompletelyExplored(x, y):
                    str += '1'
                else:
                    str += '0'
                if(x!=self.lastTileX):
                    str += ','
            str += '\n'
        return str

    def saveMapToFile(self, fName):
        with open(fName, "w") as outfile:
            outfile.write(f"Team: 08\nMap: {config.MAP_NUM}\nUnit Length: {config.MAZE_GRID_SIZE:.2f}\nUnit: in\n")
            origin = self.coordsRelativeToLL(0,0)
            outfile.write(f"Origin: ({origin[0]},{origin[1]})\n")
            outfile.write("Notes: Map generated by the absolute ballers that are Team 8 :)\n")
            outfile.write(self.toStrRealFormat())
            

    def saveHazToFile(self, fName):
        with open(fName, "w") as outfile:
            outfile.write(f"Team: 08\nMap: {config.MAP_NUM}\n")
            outfile.write("Notes: All coordinates are relative to the GEARS starting location. Assume that the GEARS starts pointing in the +x direction. The +y direction is, of course, 90 degrees CCW from +x. Basically everything should line up with the map generated in the other csv file.\n")
            outfile.write("\nHazard Type, Parameter of Interest, Parameter Value, Hazard X Coordinate (in), Hazard Y Coordinate (in)\n")
            for magHaz in self.magHazards:
                outfile.write(f"Electrical/Magnetic Activity Source, Field strength (uT), {magHaz[2]:.0f}, {magHaz[0]*config.MAZE_GRID_SIZE:.2f}, {magHaz[1]*config.MAZE_GRID_SIZE:.2f}\n")
            for irHaz in self.irHazards:
                outfile.write(f"High Temperature Heat Source, Radiated Power (W), {irHaz[2]}, {irHaz[0]*config.MAZE_GRID_SIZE:.2f}, {irHaz[1]*config.MAZE_GRID_SIZE:.2f}\n")


# actual intense logic

    def update3Sense(self, robotPos: Vector2, yaw: float, readings):
        lrCol = round((robotPos.x + config.ULTRASONIC_LR_X_OFFSET) / config.MAZE_GRID_SIZE)
        robotRow = round(robotPos.y / config.MAZE_GRID_SIZE)
        nearestYaw = round(yaw / (math.pi/2)) % 4

        self.setWall(lrCol, robotRow, (nearestYaw+2)%4, 0)

        i = 0
        for d in range(nearestYaw+3, nearestYaw+3+3):
            dir = d % 4
            if(readings[i] > 0.5 * config.MAZE_GRID_SIZE):
                print(d,"-0",readings[i])
                self.setWall(lrCol, robotRow, dir, 0)
            else:
                print(d,"-1",readings[i])
                self.setWall(lrCol, robotRow, dir, 1)
            i += 1
        



    @staticmethod
    def getNearestTileCoord(realPos: Vector2):
        return Vector2(
            round(realPos.x / config.MAZE_GRID_SIZE),
            round(realPos.y / config.MAZE_GRID_SIZE)
        )

    @staticmethod
    def nearestDir(yaw):
        return round(yaw / (math.pi/2)) % 4


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

        

