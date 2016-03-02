#!/usr/bin/env python

import rospy
from numpy import arctan2, sqrt, arange, pi, sin, cos, save
from sys import path
path.append('../modules/')
from gpsLocalization import gpsLocalization
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder
# from datetime import datetime

every_other = 3
increment = pi * .5 / 180
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other]
kp = .4 / 1
kd = .3

targets = [[2, [1, -.3]], [0, [-1.5, -.05]]]
log_length = 4096
log = [[]] * log_length
i = 0
finished_edge = False


def withDistance(x, y, theta, distance):
    new_x = x + distance * cos(theta)
    new_y = y + distance * sin(theta)
    return new_x, new_y


class Navigation(object):

    def __init__(self):

        self.gap = .7
        self.agent_id = 1
        self.stage = 0
        self.substage = 0

        self.connection = LabNavigation()
        self.path_planner = GapFinder(self.gap)
        self.actuation = ROS2DimActuate()
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)

        self.tracker.setID(self.agent_id)

        sleep_time = 7
        while sleep_time > 0:
            print "Mission starts in:", sleep_time
            sleep_time -= 1
            sleep(1)
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        print 'Starting the Navigation'
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.move, queue_size=1)

        rospy.spin()

    def move(self, data):
        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id)
        print '-----------------------------'
        global i
        global finished_edge

        # extract distance data and analyze them
        distances = list(data.ranges)[0::every_other]

        if self.substage == 0:
            self.path_planner.filterReadings(distances, angles)
            closest_reading, closest_reading_angle = self.path_planner.getMinimumReading()

            # dynamic obstacle collision avoidance
            closest_reading = min(closest_reading, 2 * self.gap)
            time_now = time()
            self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
            self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)

            # set velocity based on dynamic obstacle movement
            controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
            controlled_velocity = max(0.0, min(controlled_velocity, 1.0))
            self.actuation.setTangentialVelocityLimit(min(1, controlled_velocity))

            # find destination and analyze it
            target_object = self.connection.getStates(targets[self.stage][0])
            target = withDistance(target_object[1], target_object[2], target_object[4], targets[self.stage][1][0])
            # print target
            target_x = target[0]
            target_y = target[1]
            diff_x = target_x - x
            diff_y = target_y - y
            self.distance = sqrt(diff_x**2 + diff_y**2)

            print 'here1'
            # plan path to the target
            angle = arctan2(diff_y, diff_x) - yaw  # find direction towards target in robots coordinate frame
            subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
            subgoal_angle2 = -subgoal_angle

            # go to the point designated by path planner
            self.tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle2)

            # See if reached the destination
            if self.distance < .1:
                print '\033[92m' + '\033[1m' + 'ARRIVED TO GATE' + '\033[0m'

                # face direction
                if targets[self.stage][1][0] < 0:
                    desired_facing = self.connection.getStates(targets[self.stage][0])[4]
                else:
                    desired_facing = pi + self.connection.getStates(targets[self.stage][0])[4]
                self.tracker.faceDirection(desired_facing)
                self.substage = 1
                sleep(1)

            # save some of the variable needed for next iteration
            self.prev_closest_reading = closest_reading
            self.prev_time = time_now

        elif self.substage == 1:
            
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - targets[self.stage][1][1]
            print front_travel, front_error
            if abs(front_error) < .03:
                self.substage = 2
                print 'BREAKINGGGGGGGGGGGGGGGGGGGG'
                sleep(5)
                
            self.actuation.actuate(.5 * front_error, 0)

        elif self.substage == 2:
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - targets[self.stage][1][1] - .2
            print front_travel, front_error
            if abs(front_error) < .03:
                self.stage += 1
                self.substage = 0
                print 'BREAKINGGGGGGGGGGGGGGGGGGGG'
                
            self.actuation.actuate(.5 * front_error, 0)

        if self.stage == len(targets):
        
            save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/planner_of_agent_' + str(self.agent_id), log)
            self.subscriber.unregister()
            print '\033[92m' + '\033[1m' + 'AND DONE' + '\033[0m'
        elif self.stage > len(targets):  # just do nothing after that
            print "Stupid shit didn't unregister"
            sleep(100)


if __name__ == "__main__":
    nav = Navigation()
