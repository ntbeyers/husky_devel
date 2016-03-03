#!/usr/bin/env python

#Copyright: Nonlinear and Autonomous Systems Laboratory
#Michigan Technological University

#Author(s): Barzin Moridian, Nathan Beyers
#Created: 2016-02-20



import rospy
from numpy import arctan2, sqrt, arange, pi, sin, cos, save
from sys import path, argv
path.append('/home/naslab/husky_devel/src/microgrid_demo/modules')
from gpsLocalization import gpsLocalization
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder
import matplotlib.pyplot as plt
# from datetime import datetime

every_other = 3 #use one of every 3 readings
increment = pi * .5 / 180  #resolution for lidar angles
angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other] #select angles to use
kp = .4 / 1  #proportional gain
kd = .3      #derivative gain

targets = [[1, [2, 0]]]  #[TargetNumber,[ApproachDist, MinDist]]

# Update the new approach trajectory
def withDistance(x, y, theta, distance):
    new_x = x + distance * cos(theta)
    new_y = y + distance * sin(theta)
    return new_x, new_y


class Navigation(object):

    def __init__(self):
        # Set parameters
        self.flag = 1
        self.gap = .7   #space needed to pass through
        self.agent_id = 1   #Number for Robot $$change
        self.stage = 0      #initial target
        self.substage = 0   #initial task 0 = drive to objective

        self.topicConfig() #configure simulation namspaces or actual huskies

        self.connection = gpsLocalization(self.namespace, self.gpsTopic, self.imuTopic, self.xOffset, self.yOffset) #Connection which will give current position
        self.path_planner = GapFinder(self.gap) #Finds gaps that the robot can enter
        self.actuation = ROS2DimActuate(self.controlTopic)   #Controls the motion of the robot
        #Create a tracker which knows how to move the robot and get it's position
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)
        #Tell the tracker which robot to command
        self.tracker.setID(self.agent_id)

        #Countdown to start
        sleep_time = 3
        while sleep_time > 0:
            print "Mission starts in:", sleep_time
            sleep_time -= 1
            sleep(1)
        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        print 'Starting the Navigation'
        self.subscriber = rospy.Subscriber(self.lidarTopic, LaserScan, self.move, queue_size=1) #Call move for each laser scan


        rospy.spin()

    def topicConfig(self):
        if len(argv)>1:
            self.namespace = argv[3]
            self.lidarTopic = '/' + argv[3] + '/scan'
            self.gpsTopic = '/' + argv[3] + '/odometry/gps'
            self.imuTopic = '/' + argv[3] + '/imu/data'
            self.controlTopic = '/' + argv[3] + '/cmd_vel'
            self.xOffset = float(argv[1])
            self.yOffset = float(argv[2])

        else:
            self.namespace = ''
            self.lidarTopic ='/scan'
            self.gpsTopic = '/navsat/enu'
            self.imuTopic = '/imu/data'
            self.controlTopic = '/cmd_vel'
            self.xOffset = 0
            self.yOffset = 0


    def move(self, data):
        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id)
        print '-----------------------------'

        # extract distance data and analyze them
        distances = list(data.ranges)[0::every_other]

        #Drive to the setpoint for the target
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
            #get information about the current target
            #target_object = self.connection.getStates(targets[self.stage][0])
            if self.stage == 0:
                target_object = [0,-3,-3,0,0]
            # Find where to approach target from
            target = withDistance(target_object[1], target_object[2], target_object[4], targets[self.stage][1][0])
            print target
            
            target_x = target[0]
            target_y = target[1]
            diff_x = target_x - x
            diff_y = target_y - y
            self.distance = sqrt(diff_x**2 + diff_y**2)

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
                    desired_facing = 0
                    #desired_facing = self.connection.getStates(targets[self.stage][0])[4]
                else:
                    desired_facing = pi
                    #desired_facing = pi + self.connection.getStates(targets[self.stage][0])[4]
                self.tracker.faceDirection(desired_facing)
                self.substage = 1
                sleep(1)

            # save some of the variable needed for next iteration
            self.prev_closest_reading = closest_reading
            self.prev_time = time_now

            #Plot the results

            # xpol, ypol = self.path_planner.polarToCartesian()

            # nums = len(self.path_planner.possible_travel)
            # reading_x = [0] * nums
            # reading_y = [0] * nums
            # subgoal_x = []
            # subgoal_y = []
            # for i in range(len(self.path_planner.possible_travel)):
            #     reading_x[i] = x + self.path_planner.readings_polar[i][0] * cos(yaw - self.path_planner.readings_polar[i][1])
            #     reading_y[i] = y + self.path_planner.readings_polar[i][0] * sin(yaw - self.path_planner.readings_polar[i][1])
            #     xpol[i] = x + self.path_planner.possible_travel[i] * cos(-self.path_planner.readings_polar[i][1] + yaw)
            #     ypol[i] = y + self.path_planner.possible_travel[i] * sin(-self.path_planner.readings_polar[i][1] + yaw)
            #     if i in self.path_planner.subgoals:

            #         subgoal_x = subgoal_x + [xpol[i]]
            #         subgoal_y = subgoal_y + [ypol[i]]
            # if self.flag:
            #     self.f0 = plt.figure(1,figsize=(9,9))
            #     self.ax0 = self.f0.add_subplot(111)

            #     self.sgc, = self.ax0.plot(subgoal_x, subgoal_y, 'ko', markersize=20, label='Subgoal Candidate')
            #     self.bsg, = self.ax0.plot(x + subgoal_distance * cos(yaw - subgoal_angle),
            #              y + subgoal_distance * sin(yaw - subgoal_angle), 'go', markersize=20, label='Best Subgoal')
            #     self.ro, = self.ax0.plot(x, y, 'ms', markersize=10, label='Robot')
            #     self.dest, = self.ax0.plot(target_x, target_y, 'cs', markersize=10, label='Destination')
            #     self.pot, = self.ax0.plot(xpol, ypol, 'b.', markersize=10, label='Possible Travel')
            #     self.lidr, = self.ax0.plot(reading_x, reading_y, 'r.', markersize=10, label='Lidar Reading')
            #     self.ax0.set_xlabel('X (m)')
            #     self.ax0.set_ylabel('Y (m)')
            #     self.ax0.legend()
            #     self.ax0.axis('equal')
            #     plt.tight_layout()
            #     plt.draw()
            #     plt.pause(.1)
            #     self.flag = 0

            # else:
            #     self.sgc.set_xdata(subgoal_x)
            #     self.sgc.set_ydata(subgoal_y)
            #     self.bsg.set_xdata(x + subgoal_distance * cos(yaw - subgoal_angle))
            #     self.bsg.set_ydata(y + subgoal_distance * sin(yaw - subgoal_angle))
            #     self.ro.set_xdata(x)
            #     self.ro.set_ydata(y)
            #     self.dest.set_xdata(target_x)
            #     self.dest.set_ydata(target_y)
            #     self.pot.set_xdata(xpol)
            #     self.pot.set_ydata(ypol)
            #     self.lidr.set_xdata(reading_x)
            #     self.lidr.set_ydata(reading_y)
            #     plt.draw()



        #approach the target
        elif self.substage == 1:
            
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - targets[self.stage][1][1]
            print front_travel, front_error
            if abs(front_error) < .03:
                self.substage = 2
                print 'Connection made. Reversing.'
                sleep(5)
                
            self.actuation.actuate(.5 * front_error, 0)

        #reverse a set distance
        elif self.substage == 2:
            front_travel = self.path_planner.getFrontTravel(distances, angles)
            front_error = front_travel - targets[self.stage][1][1] - 1
            print front_travel, front_error
            if abs(front_error) < .03:
                self.stage += 1
                self.substage = 0
                print 'Connection complete. Proceeding to next operation.'
                
            self.actuation.actuate(.5 * front_error, 0)

        if self.stage == len(targets):
        
            save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/planner_of_agent_' + str(self.agent_id), log)
            self.subscriber.unregister()
            print '\033[92m' + '\033[1m' + 'AND DONE' + '\033[0m'
        elif self.stage > len(targets):  # just do nothing after that
            print "didn't unregister"
            sleep(100)


if __name__ == "__main__":
    try:
        if len(argv) > 1:
            if len(argv) != 4:
                print "Arguments: [X-destination], [Y-destination], [Namespace] "
            else:
                print "Robot type: simulated"
                print "Operating in Namespace: ", argv[3]
                print "Odometry Offset X:", argv[1]," Y: ", argv[2]
        nav = Navigation()
    finally:
        print "Done!"