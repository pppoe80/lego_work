from numpy import arctan2,sqrt,sin,cos
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
ultraSS_l = UltrasonicSensor(Port.C)
ultraSS_r = UltrasonicSensor(Port.D)
i = 0
kList = [(0.1,20)]
UMAX = 7
desired_pList = [(50,50)]
modeList = [""]
R = 0
B = 0

def Controller_St(dist=0.0,kin=0)->float:
    rvalue = dist*kList[kin](0)
    if(abs(rvalue)>UMAX):
        rvalue = UMAX*rvalue/abs(rvalue)
    return rvalue/UMAX*100

def Controller_Vfunction(i,x,y,angle,Wheel = 'L'):
    rvalue = desired_pList[i][0]*disterr(x,y,angle)
    avalue = desired_pList[i][1]*anglerr(x,y,angle)
    if(Wheel == 'L'):
        avalue = -avalue
    rvalue = rvalue+avalue
    if(abs(rvalue)>UMAX):
        rvalue = UMAX*rvalue/abs(rvalue)
    return rvalue/UMAX*100

def disterr(x,y,i):
    return sqrt(((desired_pList[i][0]-x))*(desired_pList[i][0]-x)+(desired_pList[i][1]-y)*(desired_pList[i][1]-y))

def anglerr(x,y,i,angle):
    return arctan2(desired_pList[i][1]-y,desired_pList[i][0]-x) - angle

def angle(langle,rangle):
    return (rangle-langle)*R/B

def Lf(langle,rangle):
    return (rangle+langle)*R/2

def Finish(mode = "st",MT=30000,time=0,x=0,y=0,i = 0)->bool:
    if (mode =="st"):
        return MT<time
    else:
        return disterr(x,y,i) < 20


def Process(
            i   #step
            ):
    data.log("x y   dist    angle   k   time    angleL  angleR  L")
    sw = StopWatch()
    time_p = 0
    while Finish(modeList[i],time_p):
        time = time_p
        dist = sqrt(ultraSS_l.distance()*ultraSS_r.distance())
        if(modeList[i] == "st"):
            dc_r.dc(Controller_St(dist = dist,kin=kList[i][0]))
            dc_l.dc(Controller_St(dist = dist,kin=kList[i][0]))
            data.log("None","None",dist,"None",kList[i][0],time,"None","None")
        else:
            L_Present = Lf(latemp,ratemp)
            latemp = dc_l.angle()
            ratemp = dc_r.angle()
            x += (L_Present - L)*cos()
            y += (L_Present - L)*sin()
            L = L_Present
            ta = angle(latemp,ratemp)
            dc_l.dc(Controller_Vfunction(i,x,y,angle(latemp,ratemp),'L'))
            dc_r.dc(Controller_Vfunction(i,x,y,angle(latemp,ratemp),''))
            time_p = sw.time()
            data.log(x,y,dist,ta,kList[i],time_p,latemp,ratemp,L_Present)

while (i < 1):
    data = DataLog(name="Data"+str(i),append="True")
    Process(i)
    i = i + 1
