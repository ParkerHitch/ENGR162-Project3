import time
import brickpi3
import config
from src.subsystems.Drivetrain import TwoWheel
from src.subsystems.SensorArray import SensorArray
from src.subsystems.Localization import NaiveLocalizer

print("Hello Project 3!")

BP = brickpi3.BrickPi3()

# 0  = disabled
# -1 = stop python
# anything else = enabled
state = 0

dt: TwoWheel
sensors: SensorArray
localizer: NaiveLocalizer

# stuff to do upon starting python
def robotInit():
    global dt, BP, sensors, localizer
    dt = TwoWheel(BP, config.BP_PORT_D, config.BP_PORT_C)
    dt.setPowers(0,0)
    sensors = SensorArray(BP, config.BP_PORT_A)
    localizer = NaiveLocalizer(sensors)
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
    global dt, state, localizer
    localizer.update()
    if state == 1:
        dt.setPowers(0.50,0.50)
    elif state == 2:
        dt.setPowers(-0.50,0.50)
    return

# runs once when robot becomes disabled (including when powered on)
def onDisable():
    print("Disabled.")

# runs 50 times per second while disabled
def disabledPeriodic():
    global state, dt
    dt.setPowers(0,0)
    state = int(input("Enter new state: "))
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

