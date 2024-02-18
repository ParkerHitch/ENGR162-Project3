from math import pow, sqrt

class Vector3:

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def mag(self):
        return sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))
    
    def add(self, vec2):
        return Vector3(self.x + vec2.x, self.y + vec2.y, self.z + vec2.z)