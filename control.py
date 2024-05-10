#!/usr/bin/env python3
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

### PID Control Params ###
kp = 2.0
kd = 1.5
ki = 0.5
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.

tau = 0
output_min = -3.14
output_max = 3.14

integ_min = 0
integ_max = 0

# PID Memory
integrator = 0
differentiator = 0
prev_err = 0
# prev_measurement = 0
output = 0

def pid_update(error):
	global integrator, differentiator, prev_err, output
	global integ_max, integ_min

	error = -error # change sign of error

	proportional = kp*error
	integrator = integrator + ki * (error + prev_err)

	if(integrator > integ_max):
		integrator = integ_max
	elif(integrator < integ_min):
		integrator = integ_min
	
	differentiator = kd*(error - prev_err)

	output = proportional + integrator + differentiator

	# scale the output

	if output > output_max:
		output = output_max
	elif output < output_min:
		output = output_min

	print(output)
	prev_err = error

### ***************** ###

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.

vel_input = 10

# Publisher for moving the car.
command_pub = rospy.Publisher('/wall_follow', AckermannDriveStamped, queue_size = 1)

def control(data: pid_input):
	global output, vel_input

	pid_update(data.pid_error)
	steer_angle = output

	# turning
	if(data.front_distance < 3):
		steer_angle = 80
		vel_input = 4
	else:
		vel_input = 10
	
	# print(steer_angle)

	command = AckermannDrive()

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = vel_input
	command.steering_angle = steer_angle
	# command.steering_angle_velocity = 10.0
	command.acceleration = 0.0

	ads = AckermannDriveStamped()
	ads.drive = command
	
	command_pub.publish(ads)
	
if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	rospy.Subscriber("/error", pid_input, control)
	rospy.spin()