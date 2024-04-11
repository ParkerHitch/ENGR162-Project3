import grovepi

class DualIR:
    # Set up function

    def __init__(self):
        self.sensor1= 14		# Pin 14 is A0 Port.
        self.sensor2 = 15		# Pin 15 is A0 Port.
        grovepi.pinMode(self.sensor1,"INPUT")
        grovepi.pinMode(self.sensor2,"INPUT")

    # # Output function
    # def IR_PrintValues(self):
    #         try:
    #                 sensor1= 14		# Pin 14 is A0 Port.
    #                 sensor2 = 15		# Pin 15 is A0 Port.
    #                 sensor1_value = grovepi.analogRead(sensor1)
    #                 sensor2_value = grovepi.analogRead(sensor2)
    #
    #                 print ("One = " + str(sensor1_value) + "\tTwo = " + str(sensor2_value))
    #                 #time.sleep(.1) # Commenting out for now
    #
    #         except IOError:
    #                 print ("Error")

    #Read Function
    def IR_Read(self):
        sensor1_value = grovepi.analogRead(self.sensor1)
        sensor2_value = grovepi.analogRead(self.sensor2)

        return (sensor1_value, sensor2_value)
