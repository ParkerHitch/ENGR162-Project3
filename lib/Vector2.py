from math import pow, sqrt, atan2, cos, sin
import math

class Vector2:

    def __init__(self, x: float=0, y: float=0):
        self.x = x
        self.y = y

    @staticmethod
    def fromAngle(theta):
        x = math.cos(theta)
        y = math.sin(theta)
        return Vector2(x, y)

    def mag(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2))
    
    def add(self, vec2):
        return Vector2(self.x + vec2.x, self.y + vec2.y)
    
    def sub(self, vec2):
        return Vector2(self.x - vec2.x, self.y - vec2.y)

    def dot(self, vec2):
        return self.x*vec2.x + self.y*vec2.y
    
    def mul(self, scalar: float):
        return Vector2(self.x * scalar, self.y * scalar)

    # returns a new vector rotated ang radians ccw
    def rotate(self, ang):
        return Vector2( self.x*cos(ang) - self.y*sin(ang),
                        self.x*sin(ang) + self.y*cos(ang))

    def angle(self):
        return atan2(self.y, self.x)

    def __str__(self):
        return str(self.x) + ", " + str(self.y)
