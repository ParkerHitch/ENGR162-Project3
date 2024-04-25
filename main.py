import time
import brickpi3
import config
import math
from src.subsystems.Drivetrain import TwoWheel
from src.subsystems.SensorArray import SensorArray
from src.subsystems.Localization import KalmanLocalizer
from src.subsystems.Mapping import Maze
from src.subsystems.GnC import GnC
import rtime

print("Hello Project 3!")

BP = brickpi3.BrickPi3()

# 0  = disabled
# -1 = stop python
# anything else = enabled
state = 0

dt: TwoWheel
sensors: SensorArray
localizer: KalmanLocalizer
gnc: GnC

# stuff to do upon starting python
def robotInit():
    global dt, BP, sensors, localizer, gnc, maze
    dt = TwoWheel(BP, config.BP_PORT_D, config.BP_PORT_A)
    dt.setPowers(0,0)
    sensors = SensorArray(BP, 2, 7, 8, config.BP_PORT_1)   
    # sensors.zeroUltrasonicRotor()
    maze = Maze(31,31)
    localizer = KalmanLocalizer(sensors, dt)

    gnc = GnC(dt, sensors, localizer, maze, config.BP_PORT_B, BP)

    return

def enable():
    global state
    state = 1
    onEnable()

def disable():
    global state
    state = 0
    onDisable()

def stop():
    global state, BP
    state = -1
    BP.reset_all()

# when robot switches to enable
def onEnable():
    print("Enabled!")

# 50 times per second while enabled
def enabledPeriodic():
    global state, gnc, dt, localizer, sensors
    rtime.tick()
    if state == 1:
        test = gnc.mainMazeLoop()
        if test:
            gnc.startDropoff()
            state = 10
    elif state == 10:
        gnc.updateDropoff()
    elif state == 600:
        sensors.imu.update()
        # print("YAW-RAW", sensors.imu.getYawRaw())
        # print("YAW-INTERPOLATED:", sensors.imu.geyYawPositiveInterpolated())
        # print("accel",gnc.sense.imu.accel)
        print("mag", gnc.sense.imu.getMag().cleanStr())
    elif state == 601:
        sensors.imu.update()

        if sensors.imu.hasHazard():
            print("RUN!!!!!!!!!!!!!!!!!!")
            sensors.imu.hazCount = 0

    # elif state == 300:
    #     localizer.update()
    #     rot = gnc.rotPID.updateLoop(localizer.getYaw())
    #     dt.setPowers(-rot, rot)
    #     print("YAW", localizer.getYaw())
    #     print("THETADOT", localizer.getThetaDot())
    # elif state == 301:
    #     localizer.update()
    #     rot = gnc.rotPID.updateLoop(localizer.getYaw())
    #     dt.setPowers(-0.2, 0.2)
    #     print("YAW", localizer.getYaw())
    #     print("THETADOT", localizer.getThetaDot())       
    # elif state == 302:
    #     if gnc.wheelRadiusTest():
    #         dt.setPowers(0,0)
    #         state = 0
    # elif state == 303:
    #     gnc.backDriveTest()
    return

# runs once when robot becomes disabled (including when powered on)
def onDisable():
    print("Disabled.")

# runs 50 times per second while disabled
def disabledPeriodic():
    global state, dt, gnc, sensors, maze
    dt.setPowers(0,0)
    gnc.unpowerCargo()
    state = int(input("Enter new state: "))
    rtime.tick()
    if state == 1:
        gnc.startMazeFresh()
    elif state == 2:
        state = 1
    elif state == 10:
        gnc.startDropoff()
    # elif state == -2:
    #     state = 0
    #     print(gnc.mazePath)
    elif state == -3:
        gnc.maze.print()
    elif state == -4:
        gnc.maze.printRealFormat()

    elif state == 20:
        prefix = input("Enter prefix for CSVs: ")
        maze.saveMapToFile(prefix + "team08_map.csv")
        maze.saveHazToFile(prefix + "team08_haz.csv")
        print(f"Saved to: {prefix}team08_map.csv and {prefix}team08_haz.csv!")
        state = 0

    elif state == 21:
        test = input("Trimming map. Enter y to confirm: ")
        if test=="y":
            maze.trimSelf()
            print("Trimming success!")
        else:
            print("Did not trim")
        state = 0



    elif state == -600:
        sensors.imu.readMagZBaseline()
        state = 0
    elif state == -601:
        sensors.imu.readMagZThresh()
        state = 0
    elif state == -602:
        sensors.imu.printHazInfo()
    elif state == -500:
        print("IR READINGS", sensors.irSense.IR_Read())
    
    # elif state == 302:
    #     gnc.zeroDTWheels()
    # elif state == 303:
    #     gnc.startBackDriveTest()
    elif state == -302:
        print(dt.getLEncoder(), dt.getREncoder())
        state = 0
    # elif state == 300:
    #     gnc.rotPID.setDest(math.pi/2)
    return


# enable/disable state logic and calling
robotInit()
onDisable()
disable()
while state!=-1:
    start = time.perf_counter_ns()
    try:
        print("State: ", state)
        if(state >= 1):
            enabledPeriodic()
        else:
            disabledPeriodic()
        # limiting the speed of our loop
        diff = time.perf_counter_ns() - start
        if(diff > config.NS_PER_TICK):
            print(f"LOOP OVERRUN. TOOK {diff}ns")
        else:
            time.sleep((config.NS_PER_TICK - diff) / 1e9)
    except KeyboardInterrupt:
        if(state >= 1):
            disable()
        else:
            stop()

