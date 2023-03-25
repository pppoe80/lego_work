#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

m = Motor(port=Port.D)
sw = StopWatch()
data = DataLog(append=True,name="Data1")

def PID(past_time,present_time,target,real,ki,kp,kd,anti_over,range):
    error = target - real
    i = ki*error*(present_time-past_time)
    p = kp*error
    d = kd*error/(present_time-past_time)
    rvalue = p+i+d
    if (anti_over == False):
        return rvalue
    elif (abs(rvalue)<range):
        return rvalue
    else :
        return rvalue/abs(rvalue)*range

MAX_TIME = 10
time_present = 0
TARGET = 180

ev3.speaker.say("Beep,I'm going to run!")
data.log("target",TARGET)
data.log("time_mill","angle")

while True:
    time_last = time_present
    time_present = sw.time()
    data.log(time_present,m.angle())
    if (time_present>MAX_TIME*1000):
        break
    m.dc(PID(past_time=time_last,
             present_time=time_present,
             target = TARGET,
             real = m.angle(),
             ki=0.5,kd=0.1,kp=0.05,
             anti_over=True,
             range=7)
        /7*100)
