from math import pi, radians
from lib.Vector2 import Vector2
# corosponds to 50 ticks per second
NS_PER_TICK = 2e7

# brickpi ports (I really don't want to create 2 BrickPi3 instances)
BP_PORT_1 = 0x01
BP_PORT_2 = 0x02
BP_PORT_3 = 0x04
BP_PORT_4 = 0x08

BP_PORT_A = 0x01
BP_PORT_B = 0x02
BP_PORT_C = 0x04
BP_PORT_D = 0x08

### Robot Characeteristics
# Radius of wheels (inches)
WHEEL_RADIUS = 2.6 / 2
# Distance between centers of 2 wheels width-wise (inches)
WHEEL_SEPARATION = 9.75

# Vector from center of rotation to the center of the ultrasonic rotor (inches)
# +x is forward
ULTRASONIC_ROTOR_OFFSET = Vector2(3, 0)
# ULTRASONIC_ROTOR_OFFSET = Vector2(0, 0)
# Distance from center of rotor to the point where the sensors read zero (inches)
ULTRASONIC_ROTOR_RADIUS = 2


### Maze Characteristics
# Width of a typical square (inches)
MAZE_GRID_SIZE = 18