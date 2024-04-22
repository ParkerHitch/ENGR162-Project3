from src.subsystems.Mapping import Maze
import config
from lib.Vector2 import Vector2
from math import pi

map = Maze(10,10)

map.update3Sense(Vector2(0,0).mul(config.MAZE_GRID_SIZE), 0, [100,100,100])
map.update3Sense(Vector2(0,1).mul(config.MAZE_GRID_SIZE), pi/2, [100,100,100])
map.update3Sense(Vector2(0,2).mul(config.MAZE_GRID_SIZE), pi/2, [100,100,100])
map.update3Sense(Vector2(1,2).mul(config.MAZE_GRID_SIZE), 0, [100,100,100])
map.setEndpoint(2,2)
map.addIrHazard(-1,1, 23)
map.addMagHazard(-1,2, 123.839245767863)

map.printRealFormat()
# print(map.firstTileX,"-",map.lastTileX)
# print(map.firstTileY,"-",map.lastTileY)
# print(map.coordsRelativeToLL(0,0))

map.trimSelf()

map.printRealFormat()

prefix = input("Enter prefix for CSVs: ")
map.saveMapToFile(prefix + "team08_map.csv")
map.saveHazToFile(prefix + "team08_haz.csv")
print(f"Saved to: {prefix}team08_map.csv and {prefix}team08_haz.csv!")

