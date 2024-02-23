
class TwoWheel:

    def __init__(self, BP, portL, portR):
        self.BP = BP
        self.leftP = portL
        self.rightP = portR
        self.setLEncoder(0)
        self.setREncoder(0)

    def setPowers(self, leftPower, rightPower):
        self.BP.set_motor_power(self.leftP , -int(leftPower * 100))
        self.BP.set_motor_power(self.rightP, -int(rightPower * 100))
    
    def getLEncoder(self):
        return -self.BP.get_motor_encoder(self.leftP)
    def setLEncoder(self, newPos):
        # lego motors only support offset for some reason.
        self.BP.offset_motor_encoder(self.leftP, newPos + self.getLEncoder())

    def getREncoder(self):
        return -self.BP.get_motor_encoder(self.rightP)
    def setREncoder(self, newPos):
        # lego motors only support offset for some reason.
        self.BP.offset_motor_encoder(self.rightP, newPos + self.getREncoder())