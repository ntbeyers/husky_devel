#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import time
import numpy as np

class posePlot2D(object):
	def __init__(self):

		plt.axis([0, 30, -10, 20])
		plt.ion()
		plt.show()
		self.x = 0
		self.y = 0
		self.theta = 0
		rospy.init_node('posePlot2D') #start the control node

		rospy.Subscriber('/gps_pose', Pose2D, self.updatePlot)

		self.rate = rospy.Rate(1) #rate at 1 Hz
		self.run()

	def run(self):
		while not rospy.is_shutdown():
			plt.scatter(self.x, self.y)
			plt.plot([self.x,self.x+np.cos(self.theta)], [self.y,self.y+np.sin(self.theta)])
			plt.draw()
			self.rate.sleep()


	def updatePlot(self,data):
		self.x=data.x
		self.y=data.y
		self.theta = data.theta
		


if __name__ == "__main__":
	plotter = posePlot2D()