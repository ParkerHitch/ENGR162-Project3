from lib.Vector2 import Vector2
from src.subsystems.Mapping import Maze
import config

class GnC:

    def __init__(self):
        self.dest = Vector2()
        self.maze = Maze(21,21)
        self.mazePath = []
        self.mazeState = -1

        self.scanResult = [0.0, 0.0, 0.0, 0.0]
        self.readingSum1 = 0
        self.readingCount1 = 0
        self.readingSum2 = 0
        self.readingCount2 = 0

    # updates mazePath to be a plan to reach the nearest completely unexplored area
    def genNewExplorationPlan(self, startingPoint: Vector2):
        currentTile = [round(startingPoint.x), round(startingPoint.y)]
        currentDir = 0
        self.mazePath = []
        while not self.maze.isCompletelyUnexplored(currentTile[0], currentTile[1]):
            print(currentTile)
            i = self.pathInd(currentTile[0], currentTile[1])
            if i != -1:
                # We have already visited the current tile. Remove all items after it, as they are redundant
                print("OlD:", self.mazePath)
                self.mazePath = self.mazePath[:i+1]
                print("NEW:", self.mazePath)
                print("IND:", i)
            else:
                self.mazePath.append((currentTile[0], currentTile[1]))
            for d in range(currentDir+3,currentDir+7):
                dir = d % 4
                print(f"   {dir}")
                # If is open. Assume all unknown walls are open
                if self.maze.isWall(currentTile[0], currentTile[1], dir) < config.MAZE_UNKNOWN_UPPER:
                    currentTile = Maze.nextTile(currentTile[0], currentTile[1], dir)
                    currentDir = dir
                    break
        self.mazePath.append((currentTile[0], currentTile[1]))

    def pathInd(self, tileX, tileY):
        i = 0
        for item in self.mazePath:
            if item[0]==tileX and item[1]==tileY:
                return i
            i = i + 1
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
gnc = GnC()
mz = gnc.maze

mz.setWall(0,0,0,1)
mz.setWall(0,0,3,1)
mz.setWall(0,0,2,1)

mz.setWall(0,1,0,0)
mz.setWall(0,1,1,0)
mz.setWall(0,1,2,0)
mz.setWall(0,1,3,0)

mz.setWall(1,1,0,1)
mz.setWall(1,1,3,1)
mz.setWall(1,1,1,1)


mz.print()

gnc.genNewExplorationPlan(Vector2(0,0))

print(gnc.mazePath)


