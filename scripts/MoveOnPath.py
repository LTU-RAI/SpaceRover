#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from lx16a import *
import math
from bicycle_model import *

LX16A.initialize("/dev/ttyUSB-Servo") # LX16A.initialize("/dev/ttyUSB0")

## Variables
rad_to_deg = 180/math.pi
deg_to_rad = math.pi/180
wheel_circumference = 0.387 # [m]

linear_max_speed = 518 # 518 # 0.2 m/s, range is [0 1000] 
angular_max_steer = 80 # 0-240 + bias(140-160) => range of [0 80]
angular_throttle = 700 # time to finish rotation command, in miliseconds

lin_velocity = 0
ang_velocity = 0
_v = 0
_w = 0

try:
    driveRF = LX16A(1)
    steerRF = LX16A(2)
    driveR = LX16A(3)
    steerRB = LX16A(4)
    driveRB = LX16A(5)

    driveLF = LX16A(6)
    steerLF = LX16A(7)
    driveL = LX16A(8)
    steerLB = LX16A(9)
    driveLB = LX16A(10)

    steerRF.set_angle_limits(0,240)
    steerRB.set_angle_limits(0,240)
    steerLF.set_angle_limits(0,240)
    steerLB.set_angle_limits(0,240)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()
  
# get velocity commands
def callback_vel(data):
    global lin_velocity, ang_velocity, _v, _w
    lin_velocity = data.linear.x
    ang_velocity = data.angular.z
    
    if _v == lin_velocity:
    	lin_velocity = 0
    else:
    	lin_velocity = data.linear.x

    if _w == ang_velocity:
    	ang_velocity = 0
    else:
    	ang_velocity = data.angular.z

    _v = lin_velocity
    _w = ang_velocity
    # for testing
    rospy.loginfo(f"Recieved /cmd_vel data. lin_velocity: {lin_velocity}, ang_velocity: {ang_velocity}")
    
# set wheel steering angle
def set_wheel_pos(angle, throttle):
    steerLF.move(150 - angle['front_left'],throttle)		# straight: 150
    steerRF.move(160 - angle['front_right'],throttle)	# straight: 160
    steerLB.move(150 - angle['rear_left'],throttle)		# straight: 150
    steerRB.move(140 - angle['rear_right'],throttle)		# straight: 140
    rospy.sleep(0.1)
    
def drive1(speed):

    driveRF.motor_mode(speed['front_right'])
    driveRB.motor_mode(speed['rear_right'])
    driveR.motor_mode(speed['mid_right'])
    driveL.motor_mode(-speed['mid_left'])
    driveLF.motor_mode(-speed['front_left'])
    driveLB.motor_mode(-speed['rear_left'])
    rospy.sleep(0.1)
    

# set drive speed
def drive(speed):

    driveRF.motor_mode(-speed['front_right'])
    driveRB.motor_mode(-speed['rear_right'])
    driveR.motor_mode(-speed['mid_right'])
    driveL.motor_mode(speed['mid_left'])
    driveLF.motor_mode(speed['front_left'])
    driveLB.motor_mode(speed['rear_left'])
    rospy.sleep(0.1)
    
def stop():
    driveRF.motor_mode(0)
    driveRB.motor_mode(0)
    driveL.motor_mode(0)
    driveR.motor_mode(0)
    driveLF.motor_mode(0)
    driveLB.motor_mode(0) 
    steerLF.move(150)		# straight: 150	
    steerRF.move(160)		# straight: 160	
    steerLB.move(150)		# straight: 150	
    steerRB.move(140)		# straight: 140
    rospy.sleep(0.1) 
    
def set_steer_throttle(wheel_steer):
    wheel_state = driveRF.get
    
def set_wheel_state(wheel_steer, wheel_velocity):
    # get max and min angular/linear velocity to scale speed 
    ## todo: find a nicer way to do this
    max_steer = max(wheel_steer.values())
    min_steer = min(wheel_steer.values())
    max_velocity = max(wheel_velocity.values())
    min_velocity = min(wheel_velocity.values())
    if abs(max_steer) > abs(min_steer):
        max_steer = abs(min_steer)
    if abs(min_velocity) > abs(max_velocity):
        max_velocity = abs(min_velocity)    
        
    ## Scales angular and linear velocity to fit max speed limits if needed 
    # angular limiter, outputs value in range [-angular_max_steer angular_max_steer]
    if abs(max_steer) > (angular_max_steer*deg_to_rad):  
        for key in wheel_steer:
            wheel_steer[key] = int((wheel_steer[key]*angular_max_steer)/max_steer)
    else:
        for key in wheel_steer:
            wheel_steer[key] = int(wheel_steer[key]*rad_to_deg)
    # velocity limiter, outputs value in [-linear_max_speed linear_max_speed]
    if abs(max_velocity) > (wheel_circumference*linear_max_speed)/1000:
        for key in wheel_velocity:
            wheel_velocity[key] = int((wheel_velocity[key]*linear_max_speed)/max_velocity)
    else:
        for key in wheel_velocity:
            wheel_velocity[key] = int((wheel_velocity[key]*1000)/wheel_circumference)
    
    rospy.loginfo(f'max_steer: {abs(max_steer)}, wheel_steer: {wheel_steer}')
    return wheel_steer, wheel_velocity
    
def MoveOnPath():
    rospy.init_node('MoveOnPath', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, callback_vel) 
    rate = rospy.Rate(10)
    
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        if (lin_velocity<0):
            wheel_steer, wheel_velocity = servo_ackerman(lin_velocity, ang_velocity) 
                # set correct servo values
            wheel_steer, wheel_velocity = set_wheel_state(wheel_steer, wheel_velocity)
                # change wheel position
            set_wheel_pos(wheel_steer, angular_throttle)
            rospy.sleep(0.1) # might be redundant. do testing to see if behaviour is nice
                # drive 
            drive1(wheel_velocity)
            rate.sleep()
            rospy.loginfo("pressed down")
        else:
            if (abs(ang_velocity) < 0.05 and abs(lin_velocity) < 0.05):
                stop()
                rospy.loginfo("At location.")
            else:
            # ackerman
                wheel_steer, wheel_velocity = servo_ackerman(lin_velocity, ang_velocity) 
                # set correct servo values
                wheel_steer, wheel_velocity = set_wheel_state(wheel_steer, wheel_velocity)
                # change wheel position
                set_wheel_pos(wheel_steer, angular_throttle)
                rospy.sleep(0.1) # might be redundant. do testing to see if behaviour is nice
                # drive 
                drive(wheel_velocity)
                rate.sleep()
                #rospy.spin()
        
if __name__ == '__main__': 
     try:
         MoveOnPath()
     except rospy.ROSInterruptException:
        pass