from lib.Vector2 import Vector2
from src.subsystems.Mapping import Maze
import config
import math


def localizeAndMap(self, robotPos: Vector2, posUncertanty, robotAng: float, rotorAng: float, sensorReadingW: float, sensorReadingB: float):
    increment = Vector2.fromAngle(robotAng + rotorAng)
    currentTileW = Maze.getNearestTileCoord(robotPos)
    currentPos = Vector2(robotPos.x, robotPos.y)

    totalDist = 0
    distEW = 0
    distNS = 0
    expectedWhite = Vector2()
    while totalDist < sensorReadingW + 2.5*config.G_ULTRASONIC_STDEV:
        #print(totalDist)
        realativePos = currentPos.sub(currentTileW.mul(config.MAZE_GRID_SIZE))
        #print(realativePos)
        if increment.x != 0:
            if increment.x > 0:
                distEW = ((config.MAZE_GRID_SIZE/2) - realativePos.x) / increment.x
            else:
                distEW = ((-config.MAZE_GRID_SIZE/2) - realativePos.x) / increment.x
        else:
            distEW = 999999
        if increment.y != 0:
            if increment.y > 0:
                distNS = ((config.MAZE_GRID_SIZE/2) - realativePos.y) / increment.y
            else:
                distNS= ((-config.MAZE_GRID_SIZE/2) - realativePos.y) / increment.y
        else:
            distNS = 999999
        if abs(distEW) < abs(distNS):
            # move L/R
            #walls.append([currentTileW.x, currentTileW.y, 0 if increment.x > 0 else 2])
            traverse = increment.mul(distEW)
            totalDist += traverse.mag()
            if sensorReadingW + 2.5*config.G_ULTRASONIC_STDEV:
            #wallDists.append(totalDist)
            currentTileW.x += 1 if increment.x > 0 else -1
            currentPos = currentPos.add(traverse)

        else:
            # move U/D
            walls.append([currentTileW.x, currentTileW.y, 1 if increment.y > 0 else 3])
            traverse = increment.mul(distNS)
            totalDist += traverse.mag()
            wallDists.append(totalDist)
            currentTileW.y += 1 if increment.y > 0 else -1
            currentPos = currentPos.add(traverse)

    return(walls, wallDists)


print("Starting")
print(localizeAndMap(None, Vector2(0,0), None, 7*math.pi/6, 0, 20, 9))

