import grovepi

class GroveLineFinder:

    port = 0
    last10 = []

    # Initialize a llight sensor on port "D" + digitalPort
    def __init__(self, digitalPort):
        self.port = digitalPort
        self.last10 = []
        grovepi.pinMode(self.port, "INPUT")
    
    # 0 if black 1 if white
    def getRawVal(self):
        return 1 - grovepi.digitalRead(self.port)
    
    def readandAverage(self):
        latest = self.getRawVal()
        self.last10.insert(0, latest)
        if(len(self.last10) > 10):
            self.last10.pop()
        
        sum = 0
        for i in range(0, len(self.last10)):
            sum += pow(2, -i) * self.last10[i]

        return sum/2
