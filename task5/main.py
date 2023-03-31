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
dc_l = Motor(Port.A)
dc_r = Motor(Port.B)
ultraSS = UltrasonicSensor(Port.C)
i = 0
kList = [(0.1,20)]
UMAX = 7
desired_pList = [(50)]
modeList = ["st"]

def Controller_St(dist=0.0,kin=0)->float:
    rvalue = dist*kList[kin](0)
    if(abs(rvalue)>UMAX):
        rvalue = UMAX*rvalue/abs(rvalue)
    return rvalue/UMAX*100


def DCStep(mode = "st",dist=0.0,hdata = None,kin = 0):
    if(mode == "st"):
        dc_r = Controller_St(dist = dist,kin=kin)
        dc_l = Controller_St(dist = dist,kin=kin)
        hdata.log("None","None",dist,"None",kList[kin](0))
    else:
        ""


def Finish(mode = "st",MT=30000,time=0)->bool:
    if (mode =="st"):
        return MT<time


def Process(
            i   #step
            ):
    data.log("x y   dist    angle   k   time")
    sw = StopWatch()
    while Finish(modeList[i],time=sw.time()):
        dist = ultraSS.distance()
        DCStep(dist= dist,mode=modeList[i],hdata=data,kin=i)


while True:
    data = DataLog(name="Data"+str(i),append="True")
    Process(i)
    i = i + 1
