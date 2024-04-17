from math import pow, sqrt, cos, sin
from lib.Vector2 import Vector2

class Vector3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def mag(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))
    
    def add(self, vec2):
        return Vector3(self.x + vec2.x, self.y + vec2.y, self.z + vec2.z)

    def rotateAboutY(self, ang):
        return Vector3( self.x*cos(ang) - self.z*sin(ang),
                        self.y,
                        self.x*sin(ang) + self.z*cos(ang))



    # swizzles
    def xy(self):
        return Vector2(self.x, self.y)

    def __str__(self):
        return "" + str(self.x) + ", " + str(self.y) + ", " + str(self.z)
