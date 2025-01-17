from motor_test import test_motor
import time
from pymavlink import mavutil
import numpy as np


def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    step = 0
    while step < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)
        step += 0.2

def straight(mav_connection, seconds: float, coeff:float=1):
    power = coeff * [-100, -100, 100, 100, 0, 0]
    run_motors_timed(mav_connection, seconds, power.tolist())

def donut(mav_connection, seconds: float, reverse: bool):
    power = [50, 100, -50, 100] if not reverse else [100, -50, 100, 50]
    run_motors_timed(mav_connection, seconds, power)

def crab_donut(mav_connection, seconds: float, reverse: bool):
    power = [-50, 50, 100, 100] if not reverse else [100, -50, 50, 100]
    run_motors_timed(mav_connection, seconds, power)

def crab(mav_connection, seconds: float, coeff:float = 1):
    power = coeff * [-100, 100, -100, 100, 0, 0]
    run_motors_timed(mav_connection, seconds, power)

def corkscrew(mav_connection, seconds: float, opposite: int=1):
    power = int(opposite) * [-100, 100, 100, -100, -100, -100]
    run_motors_timed(mav_connection, seconds, power)

def spin(mav_connection, seconds: float, clockwise: int=1):
    power = int(clockwise) * [100, -100, -100, 100, 0, 0]
    run_motors_timed(mav_connection, seconds, power)

if __name__ == "__main__":
    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    1	Forward/Backward	-100	-100	100	100	0	0
		                    100	100	-100	-100	0	0
    2	Crab Left/Right	-100	100	-100	100	0	0
		                100	-100	100	-100	0	0
    3	Donut	50	100	-50	100	0	0
    4	Reverse Donut	100	-50	100	50	0	0
    5	Up/Down	0	0	0	0	100	100
		        0	0	0	0	-100	-100
    6	Clockwise/Counterclockwise	-100	100	100	-100	0	0
		100	-100	-100	100	0	0"""
    
    # corkscrew down
    corkscrew(mav_connection, 5)
    
    # forward
    straight(mav_connection, 10, 1)
    
    # half reverse donut
    donut(mav_connection, 12, True)
    
    #donut
    donut(mav_connection, 21, False)    

    # half reverse donut
    donut(mav_connection, 10, True)

    # backward
    straight(mav_connection, 5,  -1)
        
    # crab right
    crab(mav_connection, 10, -1)
    
    # half reverse donut
    crab_donut(mav_connection, 12, True)

    # donut
    crab_donut(mav_connection, 21, False)

    # half reverse donut
    crab_donut(mav_connection, 10, True)

    crab(mav_connection, 5)

    # reverse donut
    donut(mav_connection, 20, True)

    #corkscrew
    corkscrew(mav_connection, 5, -1)

    # stop
    run_motors_timed(mav_connection, seconds=1, motor_settings=[0, 0, 0, 0, 0, 0])
    
    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)

