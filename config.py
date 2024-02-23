from math import pi, radians
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

# Physical characeteristics
# Radius of wheels (inches)
WHEEL_RADIUS = 2.6 / 2
# Distance between centers of 2 wheels width-wise (inches)
WHEEL_SEPARATION = 9.8
