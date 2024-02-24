import config
import grovepi
import lib.RMath as rmath

class GroveUltrasonic:

    # Initialize an ultrasonic sensor on port "D" + digitalPort
    def __init__(self, digitalPort):
        self.port = digitalPort
    
    # returns distance in centimeters
    def getRawVal(self):
        return grovepi.ultrasonicRead(self.port)
    
    # reads distance in inches
    def getInches(self):
        return rmath.cm2in(grovepi.ultrasonicRead(self.port))
