from math import pi, radians
from lib.Vector2 import Vector2
import lib.RMath as rmath
import numpy as np

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

### --- Robot Characeteristics ---
# Radius of wheels (inches)
WHEEL_RADIUS = 1.42
# Distance between centers of 2 wheels width-wise (inches)
WHEEL_SEPARATION = 6.5

# How far in front of the wheels (inches) the L/R ultrasonic sensors are
ULTRASONIC_LR_X_OFFSET = 1.5
# How far to the left/right of the center the 0 reading of the L/R ultrasonic sensors are
ULTRASONIC_LR_Y_OFFSET = 3.25

# How far in front of the wheels (inches) the 0 reading of the front ultrasonic sensor is
ULTRASONIC_FORWARD_OFFSET = 4.5

### --- Sensor Characteristics ---
# Standard deviation of the grovepi ultrasonic sensors (cm)
G_ULTRASONIC_STDEV = 2.5
IR_THRESH = 100

MAG_HARD_VEC = np.array([-10.7330, 12.4613, -47.3299])
MAG_SOFT_TRANS = np.array([[1.0584, -0.0169, 0.0524],
    [-0.0169, 1.0680, -0.0802],
    [0.0524, -0.0802, 0.8934]])

IMU_PITCH = radians(6)

IMU_YAW_0_DEF = 0
IMU_OFF90_DEF = pi/2
IMU_OFF180_DEF = pi
IMU_OFF270_DEF = 3*pi/2


# in inches per second^2
GRAVITY = 386.0886

### --- Maze Characteristics ---
# Width of a typical square (inches)
MAZE_GRID_SIZE = rmath.cm2in(40)
# MAZE_GRID_SIZE = 14.25

MAZE_UNKNOWN_LOWER = 0.4
MAZE_UNKNOWN_UPPER = 0.6

# Output files
MAP_NUM = 6

