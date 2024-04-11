import time
import brickpi3
import config
import math
from typing import List
from src.subsystems.Drivetrain import TwoWheel
from src.subsystems.SensorArray import SensorArray
from src.subsystems.Localization import KalmanLocalizer
from src.subsystems.GnC import BasicGnC
from src.subsystems.Mapping import Maze
from src.util.PID import *
from lib.Vector2 import *
import rtime as rt

print("Hello Project 3!")

BP = brickpi3.BrickPi3()

# 0  = disabled
# -1 = stop python
# anything else = enabled
state = 0

dt: TwoWheel
sensors: SensorArray
localizer: KalmanLocalizer
destAng: float
rotPID: rotationPID
posPID: genericPID
gnc: BasicGnC
maze: Maze
rt.tick()

destPoints: List[Vector2]
currentPoint = 0

# stuff to do upon starting python
def robotInit():
    global dt, BP, sensors, localizer, gnc, maze
    dt = TwoWheel(BP, config.BP_PORT_D, config.BP_PORT_C)
    dt.setPowers(0,0)
    sensors = SensorArray(BP, config.BP_PORT_A, 7, 8, 1)
    sensors.zeroUltrasonicRotor()
    maze = Maze(21,21)
    localizer = KalmanLocalizer(sensors, dt, maze)
    # gnc = BasicGnC(dt, sensors, localizer)
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
    rt.tick()
    print("Enabled!")

# 50 times per second while enabled
def enabledPeriodic():
    rt.tick()
    global dt, state, localizer, destPoints, currentPoint, gnc
    localizer.update()
    
    if state==1:
        print(localizer.getPos(), localizer.getYaw())
    # if state == 1:
    #     failed = gnc.mainMazeLoop()
    #     if failed:
    #         disable()
    #     
    # elif state == 2:
    #     print(localizer.getYaw())
    #     diff = rotPID.updateLoop(localizer.getYaw())
    #     dt.setPowers(-diff, diff)
    # elif state == 4:
    #     if gnc.turnAndDriveToDest():
    #     # if gnc.beelineToDest():
    #         state = 41
    # elif state == 41:
    #     dt.setPowers(0,0)
    #     try:
    #         print(f"Arrived at point {currentPoint}")
    #         print("[^+c] to continue to next point (or stop if reached all points)")
    #         while True:
    #             time.sleep(0.1)
    #             continue
    #     except KeyboardInterrupt:
    #         currentPoint += 1
    #         if currentPoint >= len(destPoints):
    #             state = 0
    #         else:
    #             gnc.setDest(destPoints[currentPoint])
    #             state = 4
    # elif state == 20:
    #     print(localizer.getYaw())
    #     print(localizer.motLPos, localizer.motRPos)
    else:
        print(f"Invalid state: {state}")
        disable()
    return

# runs once when robot becomes disabled (including when powered on)
def onDisable():
    print("Disabled.")

# runs 50 times per second while disabled
def disabledPeriodic():
    global state, dt, destAng, localizer, destPoints, currentPoint
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
                currentPoint = 0
            gnc.setDest(destPoints[0])
    # elif state == 1:
    #     gnc.startMazeSolve()
    elif state == -1:
        gnc.maze.print()
        state = 0

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

