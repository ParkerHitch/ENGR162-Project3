import math
from os import walk
from lib.Vector2 import Vector2
import config
from src.subsystems.Localization import KalmanLocalizer
import numpy as np

def threeSenseLocalize(robotPos: Vector2, yaw: float, distLeft: float, distRight: float, distFront: float) -> Vector2:
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

        return robotPos.add(Vector2(diffX, diffY))

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
        
        return robotPos.add(Vector2(diffX, diffY))

print(config.MAZE_GRID_SIZE)
print(threeSenseLocalize(Vector2(0,0).mul(config.MAZE_GRID_SIZE), math.radians(20), 5.5, 5.5, 3))

k = KalmanLocalizer(None, None, None)
while True:
    k.ekfUpdate(np.array([ 0,0, 0, math.pi, 0,0]))
    print("State",k.state)

