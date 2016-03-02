#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
# import numpy as np
from numpy import arange, pi, sqrt, arctan2, sin, cos
# from time import time, sleep
# from datetime import datetime
# import logging
from sensor_msgs.msg import LaserScan  # Messages from Lidar System
# import sys
from sys import path, exit
path.append('/home/naslab/husky_devel/src/microgrid_demo/modules')
from path_planning import GapFinder
from gpsLocalization import gpsLocalization


every_other = 1
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]

# logging.basicConfig(filename='control.log', format=50 * '=' +
# '\n%(asctime)s %(message)s', level=logging.DEBUG)  # set up debug logging


class LidarLogger(object):

    def __init__(self):
        self.path_planner = GapFinder(.6)
        #rospy.init_node('lidarLogger')  # start the control node
        self.logger = []
        self.connection = gpsLocalization('husky1test','husky1/odometry/gps','husky1/imu/data')
        self.subscriber = rospy.Subscriber('husky1/scan', LaserScan, self.readDistance)
        self.rate = rospy.Rate(10)  # set rate to 10 Hz
        # read = 0
        while not rospy.is_shutdown() and len(self.logger) < 1:
            self.rate.sleep()
            # rospy.spin()

        # try:
        #     pass
        # self.__run__() #start motion
        # except Exception, e:
            # print e
        #     print "Error during runtime. Exiting."
        #     logging.exception(e)
        # finally:
        #     print "Run complete."
        # print self.logger  # REMOVE LATER
        #     np.save('scan' + str(datetime.now()), self.logger)

    # def __run__(self):
    # sleep(5) #delay before startup
    #     while not rospy.is_shutdown():
    #         self.rate.sleep()

    def readDistance(self, data):
        self.subscriber.unregister()
        distances = list(data.ranges)[0::every_other]
        # print distances
        self.path_planner.filterReadings(distances, angles)
        # print self.path_planner
        x, y = self.path_planner.polarToCartesian()
        crap1, robot_x, robot_y, robot_z, robot_yaw, crap, crap2 = self.connection.getStates(0)
        target_x = 7
        target_y = 4
        diff_x = target_x - robot_x
        diff_y = target_y - robot_y
        print robot_x
        distance = sqrt(diff_x**2 + diff_y**2)
        # print 'here'
        # if distance < .1:
            # print '.'
        #     return
        # print 'here'
        angle = arctan2(diff_y, diff_x) - robot_yaw
        subgoal_distance, subgoal_angle = self.path_planner.planPath(distance, -angle)
        print distance, -angle, subgoal_distance, subgoal_angle
        # scan = data.ranges
        # print scan
        # self.logger = scan
        # print "Lidar Scan Recieved. Logging data..."
        f0 = plt.figure(1,figsize=(9,9))
        ax0 = f0.add_subplot(111)
        nums = len(self.path_planner.possible_travel)
        # for i in range(nums):
        # ax0.plot(x, y, 'r.')
        # print self.path_planner.subgoals
        reading_x = [0] * nums
        reading_y = [0] * nums
        subgoal_x = []
        subgoal_y = []
        for i in range(len(self.path_planner.possible_travel)):
            # print robot_x , self.path_planner.possible_travel[i] , -self.path_planner.readings_polar[i][1] , calibrating_theta,robot_yaw
            # print self.path_planner.possible_travel
            reading_x[i] = robot_x + self.path_planner.readings_polar[i][0] * cos(robot_yaw - self.path_planner.readings_polar[i][1])
            reading_y[i] = robot_y + self.path_planner.readings_polar[i][0] * sin(robot_yaw - self.path_planner.readings_polar[i][1])
            x[i] = robot_x + self.path_planner.possible_travel[i] * cos(-self.path_planner.readings_polar[i][1] + robot_yaw)
            y[i] = robot_y + self.path_planner.possible_travel[i] * sin(-self.path_planner.readings_polar[i][1] + robot_yaw)
            if i in self.path_planner.subgoals:
                # print x[i]
                subgoal_x = subgoal_x + [x[i]]
                subgoal_y = subgoal_y + [y[i]]

        # print subgoal_x
        # if environment_state is 'not_safe':
        # elif environment_state is 'safe':
        #     ax0.plot(target_distance * cos(target_angle),
        #              target_distance * sin(target_angle), 'go', markersize=20)
        # elif environment_state is 'close_to_obstacle':
        #     ax0.plot(target_distance * cos(target_angle),
        #              target_distance * sin(target_angle), 'ro', markersize=20)
        ax0.plot(subgoal_x, subgoal_y, 'ko', markersize=20, label='Subgoal Candidate')
        ax0.plot(robot_x + subgoal_distance * cos(robot_yaw - subgoal_angle),
                 robot_y + subgoal_distance * sin(robot_yaw - subgoal_angle), 'go', markersize=20, label='Best Subgoal')
        ax0.plot(robot_x, robot_y, 'ms', markersize=10, label='Robot')
        ax0.plot(target_x, target_y, 'cs', markersize=10, label='Destination')
        ax0.plot(x, y, 'b.', markersize=10, label='Possible Travel')
        ax0.plot(reading_x, reading_y, 'r.', markersize=10, label='Lidar Reading')
        ax0.set_xlabel('X (m)')
        ax0.set_ylabel('Y (m)')
        ax0.legend()
        ax0.axis('equal')
        plt.tight_layout()
        plt.draw()
        plt.pause(.1)
        raw_input("<Hit Enter To Close>")
        plt.close(f0)
        return


if __name__ == "__main__":
    try:
        lidarLog = LidarLogger()
    except rospy.ROSInterruptException, e:
        raise e