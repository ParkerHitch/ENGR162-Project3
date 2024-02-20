from src.subsystems.sensors.IMU import IMU

dist = -2

file = open("test/MagDistTest.csv", "w")

imu = IMU()
imu.initialize()

file.write("Distance from IMU is on +y axis\n")

while dist!=-1:
    dist = int(input("Enter distance from magnet (cm): "))
    if(dist == -1):
        break
    imu.update()
    mag = imu.getMag()
    print(mag)

    file.write(str(dist) +", " + str(mag)+"\n")

file.close()
