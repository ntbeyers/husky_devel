#!/usr/bin/env python

#Copyright: Nonlinear and Autonomous Systems Laboratory
#Michigan Technological University

#Author(s): Nathan Beyers
#Created: 2015-9-30


import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import tf
from math import pi

class gpsLocalization(object):
	def __init__(self, namespace, odomTopic, imuTopic, xOffset, yOffset):
		rospy.init_node(namespace +'gpsLocalization') #start the control node

		rospy.Subscriber(odomTopic, Odometry, self.updatePosition)
		rospy.Subscriber(imuTopic, Imu, self.updateRotation)

		self.rate = rospy.Rate(10) #rate at 1 Hz


		self.pose_msg = Pose2D() #create a 2D pose message and initialize values
		self.pose_msg.x = float('nan')
		self.pose_msg.y = float('nan')
		self.pose_msg.theta = float('nan')

		self.ang_msg = Pose2D() #create a 2D pose message and initialize values
		self.ang_msg.x = float('nan')
		self.ang_msg.y = float('nan')
		self.ang_msg.theta = float('nan')

		self.roll = 0
		self.pitch = 0
		self.yaw = 0

		self.xOffset = xOffset
		self.yOffset = yOffset

	def getStates(self, num):
	    # get position
	    agent_id = 0  #Change for multiple agents
	    x = self.pose_msg.x + self.xOffset
	    y = self.pose_msg.y + self.yOffset
	    z = 0
	    yaw = self.pose_msg.theta
	    pitch = 0
	    roll = 0
	    return agent_id, x, y, z, yaw, pitch, roll


	def updatePosition(self,data):
		self.pose_msg.x = data.pose.pose.position.x
		self.pose_msg.y = data.pose.pose.position.y

	def updateRotation(self,data):
		quaternion = [data.orientation.x, data.orientation.y,data.orientation.z, data.orientation.w]
		euler = tf.transformations.euler_from_quaternion(quaternion, axes = 'rzxy')
		self.pose_msg.theta = euler[0] #+3*pi/2

if __name__ == "__main__":
	loc = gpsLocalization()