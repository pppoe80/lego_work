#!/usr/bin/env python3
from ev3dev.ev3 import *
from ev3dev2.power import PowerSupply
from math import *
import time

motor = LargeMotor('outA')

max_voltage = round(PowerSupply().measured_volts,2)

motor_input =100    # Min = -100, Max = 100
total_time = 1      # After this many seconds, the motor will stop

for i in range(8):
    
    data = open('data' + str(i+1)+ '.txt','w')
    motor.run_direct(duty_cycle_sp = 0)
    time.sleep(1)
    timestart = time.time()
    motor.position = 0
    while True:

        # Calculating time up to 3 decimal places
        

        timenow = round(time.time()-timestart, 3)

        # Applying voltage
        motor.run_direct(duty_cycle_sp = motor_input-i*25)

        # Writing the following data: applied voltage (volts), time passed (seconds), motor speed (degrees per second)
        
        data.write(str(max_voltage*(motor_input-i*25)/100)+' '+'{0:.3f}'.format(timenow)+' '+str(motor.speed)+' '+str(motor.position)+'\n')

        if timenow > total_time:
            motor.run_direct(duty_cycle_sp = 0)
            break
