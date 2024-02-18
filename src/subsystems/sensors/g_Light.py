import grovepi

class GroveLightSensor:

    port = 0

    # Initialize a llight sensor on port "A" + analogPort
    def __init__(self, analogPort, offset=0):
        self.port = analogPort
        self.offset = offset
        grovepi.pinMode(self.port, "INPUT")
    
    # returns a value in the range: [0,1024)
    # higher is brighter reading
    def getRawVal(self):
        return grovepi.analogRead(self.port) + self.offset
    
    # returns brightness as a percentage [0,1.00]
    # higher percentage = brighter environment
    def getBrightness(self):
        return self.getRawVal() / 1023.0
