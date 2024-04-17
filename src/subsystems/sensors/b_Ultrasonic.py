import lib.RMath as rmath

class EV3Ultrasonic:
    def __init__(self, BP, port):
        self.BP = BP
        self.port = port
        self.BP.set_sensor_type(self.port, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        self.lastReading = 0
        self.currentReading = 0
        self.replaced = False

    def getRawVal(self):
        self.lastReading = self.currentReading
        self.currentReading = self.BP.get_sensor(self.port)
        return self.currentReading

    def getInchesUnsafe(self):
        return rmath.cm2in(self.getRawVal())
    
    def getInchesFiltered(self):
        self.getRawVal()
        if self.currentReading == 0 or self.currentReading == 255:
            self.currentReading = self.lastReading
            return rmath.cm2in(self.currentReading)
        # print("RawFront", self.currentReading)
        if abs(self.currentReading - self.lastReading) > self.lastReading*0.7:
            if self.replaced == False:
                self.replaced = True
                return rmath.cm2in(self.lastReading)
        self.replaced = False
        return rmath.cm2in(self.currentReading)

