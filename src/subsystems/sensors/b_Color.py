import brickpi3

class NXTColor:
    def __init__(self, BP, port):
        self.BP = BP
        self.port = port
        self.BP.set_sensor_type(self.port, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

    def getRawVal(self):
        try:
            value = self.BP.get_sensor(self.port)
            return value
        except brickpi3.SensorError as error:
            return -1
