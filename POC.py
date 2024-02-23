import time
import brickpi3
import config
import math
from typing import List
from src.subsystems.Drivetrain import TwoWheel
from src.subsystems.SensorArray import SensorArray
from src.subsystems.Localization import NaiveLocalizer
from src.util.PID import *
from lib.Vector2 import *

print("Hello Project 3!")

BP = brickpi3.BrickPi3()

# 0  = disabled
# -1 = stop python
# anything else = enabled
state = 0

dt: TwoWheel
sensors: SensorArray
localizer: NaiveLocalizer
destAng: float
rotPID: rotationPID
posPID: genericPID

destPoints: List[Vector2]
currentPoint = 0

# stuff to do upon starting python
def robotInit():
    global dt, BP, sensors, localizer, rotPID, posPID
    rotPID = rotationPID(0.85, 0.02, 0.05, 0.3, math.radians(5), maxI=5)
    posPID = genericPID(0.3, 0, 0, 0.25, 0.25)
    dt = TwoWheel(BP, config.BP_PORT_D, config.BP_PORT_C)
    dt.setPowers(0,0)
    sensors = SensorArray(BP, config.BP_PORT_A)
    localizer = NaiveLocalizer(sensors, dt)
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
    global dt, state, localizer, rotPID, posPID, destPoints, currentPoint
    localizer.update()
    if state == 2:
        print(localizer.getYaw())
        diff = rotPID.updateLoop(localizer.getYaw())
        dt.setPowers(-diff, diff)
    elif state == 4:
        diff = destPoints[currentPoint].sub(localizer.pos)
        yawDiff = rmath.maxClamp(diff.mag(), (localizer.getYaw() - diff.angle()))
        dir = Vector2.fromAngle(localizer.getYaw())

        # print(diff)
        # print(dir)

        scalarDiff = diff.dot(dir)

        # print(scalarDiff)

        rot = rotPID.updateLoop(yawDiff)
        drive = (1 - 0.5*math.fabs(rot* (1/rotPID.max))) * -posPID.updateLoop(scalarDiff)

        dt.setPowers(drive - rot, drive + rot) 
        # dt.setPowers(-rot, rot)
        # dt.setPowers(drive, drive)


    elif state == 20:
        print(localizer.getYaw())
        print(localizer.motLPos, localizer.motRPos)
    else:
        print(f"Invalid state: {state}")
        disable()
    return

# runs once when robot becomes disabled (including when powered on)
def onDisable():
    print("Disabled.")

# runs 50 times per second while disabled
def disabledPeriodic():
    global state, dt, destAng, localizer, rotPID, destPoints, posPID
    dt.setPowers(0,0)
    state = int(input("Enter new state: "))
    if state == -2:
        localizer.update()
        localizer.zeroYaw()
        print("Yaw zeroed")
        state = 0
    elif state == -3:
        localizer.update()
        localizer.zeroPos()
        print("Position zeroed")
        state = 0
    elif state == 2:
        destAng = math.radians(float(input("Enter desired angle (degrees): ")))
        rotPID.setDest(destAng)
    elif state == 4:
        destPoints = []
        numPoints = int(input("Enter the number of points to navigate to (or -1 to abort): "))
        if numPoints == -1:
            state = 0
        else:
            for i in range(numPoints):
                x = float(input(f"Enter x position of point {i+1} (inches): "))
                y = float(input(f"Enter y position of point {i+1} (inches): "))
                destPoints.append(Vector2(x, y))

            rotPID.setDest(0)
            posPID.setDest(0)

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

