#!/usr/bin/env python

#Code which goes from one point to another while avoiding obstacles


import rospy
from numpy import arctan2, sqrt, arange, pi, sin, cos, save
from sys import path, exit, argv
path.append('/modules/')
from gpsLocalization import gpsLocalization
from sensor_msgs.msg import LaserScan
from actuate import ROS2DimActuate
from tracker import PlanarTracker
from time import sleep, time
from path_planning import GapFinder


every_other = 3
#increment = pi * .5 / 180
increment = pi * .5 / 180
#angles = arange(-3 * pi / 4, 3 * pi / 4 + increment, increment)[0::every_other] #Actual Husky
angles = arange(-3*pi / 4, 3*pi / 4 + increment, increment)[0::every_other] #Simulated husky
kp = .4 / 1
kd = .3


log_length = 4096
log = [[]] * log_length
i = 0
stage = 0
finished_logging = False
temp_var = 1
temp_var_2 = temp_var * log_length


class Navigation(object):

    def __init__(self):

        #set parameters
        self.gap = .7 #space needed to pass through
        self.agent_id = 0
        self.topicConfig()

        self.connection = gpsLocalization(argv[3],self.gpsTopic,self.imuTopic)   #Connection which will give current position
        self.path_planner = GapFinder(self.gap)  #Finds gaps that the robot can enter
        self.actuation = ROS2DimActuate(self.controlTopic)   #Controls the motion of the robot
        self.actuation.setAngularVelocityLimit(.5)  #Sets the maximum velocity
        #Create a tracker which knows how to move the robot and get it's position
        self.tracker = PlanarTracker(self.actuation.actuate, self.connection.getStates)
        #Tell the tracker which robot to command
        self.tracker.setID(self.agent_id)

        self.distance = []
        self.prev_closest_reading = 0.0
        self.prev_time = time()
        self.crash_avert_velocity = 0.0

        print 'Starting the Navigation'
        sleep(2)
        self.subscriber = rospy.Subscriber(self.lidarTopic, LaserScan, self.move, queue_size=1) #Call move for each laser scan

        rospy.spin()

    def topicConfig(self):
        if len(argv)>1:
            self.lidarTopic = '/' + argv[3] + '/scan'
            self.gpsTopic = '/' + argv[3] + '/odometry/filtered'
            self.imuTopic = '/' + argv[3] + '/imu/data'
            self.controlTopic = '/' + argv[3] + '/cmd_vel'
            self.target_x = float(argv[1])
            self.target_y = float(argv[2])

        else:
            self.lidarTopic ='/scan'
            self.gpsTopic = '/navsat/enu'
            self.imuTopic = '/imu/data'
            self.controlTopic = '/cmd_vel'
            self.target_x = 5 #destination coordinates
            self.target_y = 4


    def move(self, data):
        agent_id, x, y, z, yaw, pitch, roll = self.connection.getStates(self.agent_id) #Get localization info
        #print 'x: ', x,'y: ', y,'theta: ', yaw

        print yaw
        print '-----------------------------'
        global i
        global stage
        global finished_logging
        distances = list(data.ranges)[0::every_other] #store the range readings from the lidar
        self.path_planner.filterReadings(distances, angles) #filter the results

        closest_reading, closest_reading_angle = self.path_planner.getMinimumReading()
        closest_reading = min(closest_reading, 2 * self.gap)
        time_now = time()
        self.crash_avert_velocity = (self.crash_avert_velocity + (closest_reading - self.prev_closest_reading) * kd / (time() - self.prev_time)) / 2
        self.crash_avert_velocity = min(0.0, self.crash_avert_velocity)

        controlled_velocity = (closest_reading) * kp + self.crash_avert_velocity
        controlled_velocity = max(0.0, min(controlled_velocity, 1.0))

        self.actuation.setTangentialVelocityLimit(min(.2, controlled_velocity))

        i += 1
        if i % temp_var is 0 and i < temp_var_2:
            log[i / temp_var] = [x, y, yaw, self.path_planner.readings_polar]
        print self.target_x, x
        print self.target_y, y
        diff_x = self.target_x - x
        diff_y = self.target_y - y
        self.distance = sqrt(diff_x**2 + diff_y**2)

        if self.distance < .1:
            stage += 1
            print 'ARRIVED!!!!!!!!!!'
            if finished_logging is False and i >= temp_var_2:
                self.tracker.saveLog()
                save('loginfo', log)
                finished_logging = True
            self.target_y = self.target_y * -1
            self.target_x = self.target_x * -1
            exit()

        angle = arctan2(diff_y, diff_x) - yaw
        #print 'dist: ', self.distance, 'angle: ', -angle
        subgoal_distance, subgoal_angle = self.path_planner.planPath(self.distance, -angle)
        subgoal_angle2 = -subgoal_angle

        self.tracker.moveTowardsDynamicPoint(subgoal_distance, subgoal_angle2)

        self.prev_closest_reading = closest_reading
        self.prev_time = time_now



if __name__ == "__main__":
    try:
        if len(argv) > 1:
            if len(argv) != 4:
                print "Arguments: [X-destination], [Y-destination], [Namespace] "
            else:
                print "Robot type: simulated"
                print "Operating in Namespace: ", argv[3]
                print "Destination Point X:", argv[1]," Y: ", argv[2]
        nav = Navigation()
    finally:
        if finished_logging is False:
            save('loginfo2', log)
