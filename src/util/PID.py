import time
import config
import lib.RMath as rmath

class genericPID:

    kP = 0
    kI = 0
    kD = 0
    max = 1

    setpoint = 0

    currentError = None
    errorDerivative = None
    integralAccumulator = 0
    maxI = float('inf')
    lastUpdated = 0

    deadband = 0.1
    velDeadband = 0.01

    def __init__(self, p, i, d, max=1, deadband=0.1, velDeadband=0.01, maxI=float('inf')):
        self.kP = p
        self.kI = i
        self.kD = d
        self.max = max

        self.deadband = deadband
        self.velDeadband = velDeadband
        self.maxI = maxI
        
        self.lastUpdated = time.time_ns()

    def setDest(self, dest):
        self.integralAccumulator = 0
        self.setpoint = dest
        self.currentError = None

    def updateLoop(self, pos):
        correction = 0.0
        now = time.time_ns()

        # if the loop is running at normal speed then deltaT will be 1.
        # division just helps to keep our kP, kI, & kD values in reasonable ranges.
        deltaT = (now - self.lastUpdated) / config.NS_PER_TICK
        
        newError = self.setpoint - pos

        # p term
        correction += newError * self.kP

        if(self.currentError != None):
            # d term
            # prevent jumps in derivative when we alter the setpoint (this dramatically alters error in a single frame)
            self.errorDerivative = ((newError - self.currentError) / deltaT)
            correction += self.errorDerivative * self.kD

            # i term
            # use a trappazoidal approximation of the error function
            self.integralAccumulator += ((newError + self.currentError)/2) * deltaT
            if self.integralAccumulator > self.maxI:
                self.integralAccumulator = self.maxI
            elif self.integralAccumulator < -self.maxI:
                self.integralAccumulator = -self.maxI
            correction += self.integralAccumulator * self.kI

        # update values for next time
        self.currentError = newError
        self.lastUpdated = now

        correction = rmath.clamp(-self.max, self.max, correction)

        return correction
    
    def getErr(self):
        return self.currentError
    
    def resetIntegralAccumulator(self):
        self.integralAccumulator = 0

    def atSetpoint(self):
        return self.currentError != None and abs(self.currentError) < self.deadband and abs(self.errorDerivative) < self.velDeadband