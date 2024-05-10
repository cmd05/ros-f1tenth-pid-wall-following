#!/usr/bin/env python3

import math
import rospy

# message imports
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# declare globas
angle_range = 360  # Hokuyo 4LX has 240 degrees FoV for scan

forward_projection = 1.5	# distance (in m) that we project the car forward
						    # for correcting the error. You have to adjust this.(1.5)(0.5)

desired_distance = 0.9  # distance from the wall (in m). (defaults to right wall)(0.9)

error = 0.0		# initialize the error

# vel = 1.0
# car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

pub = rospy.Publisher('/error', pid_input, queue_size=10)

def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.

	# 1080 readings
	# -180 ... 0, 1 ... 179
	# step = 0.33 deg

	return data.ranges[(angle+180)*3 + 1] # "middle" (deg+0.66)th angle range

def callback(data: LaserScan):
	global forward_projection
	global error

	# theta = None  # you need to try different values for theta
         # obtain the ray distance for theta
	 # obtain the ray distance for 0 degrees (i.e. directly to the right of the car)

	theta = 50
	a = getRange(data,90-theta)
	b = getRange(data,90)
	swing_theta = math.radians(theta)

	# print(a, b)
	
	## Your code goes here to determine the error as per the alrorithm 
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement
	ratio = (a*math.cos(swing_theta) - b) / a*math.sin(swing_theta)
	swing_alpha = math.atan((ratio))
	ab = b*math.cos(swing_alpha)
	cd = ab + forward_projection*math.sin(swing_alpha)

	
	error = desired_distance - cd
	
	print(-error)
	
	msg = pid_input()	# An empty msg is created of the type pid_input
						# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.front_distance = getRange(data, 0)
	msg.front_right = ab

	# msg.pid_vel = vel		# velocity error can also be sent.

	pub.publish(msg)

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/scan",LaserScan,callback)
	rospy.spin()