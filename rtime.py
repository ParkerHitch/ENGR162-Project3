import time

runTimeNS = 0
runTime = 0
dt = 0
dtNS = 0

last = 0

def tick():
    global dt, now, last

    now = time.perf_counter()
    dt = now - last
    last = now

