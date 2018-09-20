from donkeycar.parts.actuator import BluePill
from donkeycar import Vehicle
from donkeycar.parts.turtle import *
from donkeycar.parts.transform import Lambda

def obstacle_stop(throttle, dist):
    if dist < 500:
        throttle = -1
    return throttle

v = Vehicle()

rw = RouteWriter()

route = rw.generate()

# rr = RouteReader(route)
bp = BluePill('/dev/ttyACM0')

v.mem["mode"] = "local"
v.mem["angle"] = 0
v.mem["raw_throttle"] = 0.325
v.mem["distance"] = 9999

# v.add(bp, inputs=["angle", "throttle", "mode"], outputs=['x1', 'x2', 'x3', 'distance'])

decision_maker = Lambda(obstacle_stop)
printer = Lambda(print)
v.add(decision_maker, inputs=["raw_throttle", "distance"], outputs=["throttle"])
v.add(printer, inputs=["distance", "throttle"])
# v.add(rr, outputs=["angle", "throttle"])
v.add(bp, inputs=["angle", "throttle", "mode"], outputs=['x1', 'x2', 'x3', 'distance'])

v.start()
