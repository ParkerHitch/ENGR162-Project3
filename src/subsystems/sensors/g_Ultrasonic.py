import config
import grovepi

class GroveUltrasonic:

    # Initialize an ultrasonic sensor on port "D" + digitalPort
    def __init__(self, digitalPort):
        self.port = digitalPort
    
    # 0 if black 1 if white
    def getRawVal(self):
        return grovepi.ultrasonicRead(self.port)
    
    def hasWall(self):
        return self.getRawVal() < config.DIST_THRESH
