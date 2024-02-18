import time
import brickpi3
import config


print("Hello Project 3!")

BP = brickpi3.BrickPi3()

# 0  = disabled
# -1 = stop python
# anything else = enabled
state = 0


# stuff to do upon starting python
def robotInit():
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
    return

# runs once when robot becomes disabled (including when powered on)
def onDisable():
    print("Disabled.")

# runs 50 times per second while disabled
def disabledPeriodic():
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

