import grovepi

class GroveLineFinder:

    # Initialize a llight sensor on port "D" + digitalPort
    def __init__(self, digitalPort):
        self.port = digitalPort
        grovepi.pinMode(self.port, "INPUT")
    
    # 0 if no magnet 1 if magnet
    def getRawVal(self):
        return grovepi.digitalRead(self.port)
