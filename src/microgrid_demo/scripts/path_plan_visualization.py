#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from numpy import arange, pi, sqrt, arctan2, sin, cos
from sensor_msgs.msg import LaserScan  # Messages from Lidar System
from sys import path, exit
path.append('/home/naslab/husky_devel/src/microgrid_demo/modules')
from path_planning import GapFinder
from gpsLocalization import gpsLocalization

every_other = 1
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]

class LidarLogger(object):

    def __init__(self):
        self.path_planner = GapFinder(.6)
        self.logger = []
        self.connection = gpsLocalization('husky1test','husky1/odometry/gps','husky1/imu/data',0,0)
        self.subscriber = rospy.Subscriber('husky1/scan', LaserScan, self.readDistance)
        self.rate = rospy.Rate(10)  # set rate to 10 Hz
        while not rospy.is_shutdown() and len(self.logger) < 1:
            self.rate.sleep()

    def readDistance(self, data):
        self.subscriber.unregister()
        distances = list(data.ranges)[0::every_other]

        self.path_planner.filterReadings(distances, angles)

        x, y = self.path_planner.polarToCartesian()
        crap1, robot_x, robot_y, robot_z, robot_yaw, crap, crap2 = self.connection.getStates(0)
        target_x = 7
        target_y = 4
        diff_x = target_x - robot_x
        diff_y = target_y - robot_y
        print robot_x
        distance = sqrt(diff_x**2 + diff_y**2)

        angle = arctan2(diff_y, diff_x) - robot_yaw
        subgoal_distance, subgoal_angle = self.path_planner.planPath(distance, -angle)
        print distance, -angle, subgoal_distance, subgoal_angle

        f0 = plt.figure(1,figsize=(9,9))
        ax0 = f0.add_subplot(111)
        nums = len(self.path_planner.possible_travel)

        reading_x = [0] * nums
        reading_y = [0] * nums
        subgoal_x = []
        subgoal_y = []
        for i in range(len(self.path_planner.possible_travel)):

            reading_x[i] = robot_x + self.path_planner.readings_polar[i][0] * cos(robot_yaw - self.path_planner.readings_polar[i][1])
            reading_y[i] = robot_y + self.path_planner.readings_polar[i][0] * sin(robot_yaw - self.path_planner.readings_polar[i][1])
            x[i] = robot_x + self.path_planner.possible_travel[i] * cos(-self.path_planner.readings_polar[i][1] + robot_yaw)
            y[i] = robot_y + self.path_planner.possible_travel[i] * sin(-self.path_planner.readings_polar[i][1] + robot_yaw)
            if i in self.path_planner.subgoals:

                subgoal_x = subgoal_x + [x[i]]
                subgoal_y = subgoal_y + [y[i]]


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