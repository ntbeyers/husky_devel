from numpy import pi, cos, append, save
from rotation import rotate2DimFrame, wrapAnglePi
from control_system import Proportional
from datetime import datetime  # remove later
from time import time


class PlanarTracker(object):

    "This class helps ground robots to navigate on different types of reference trajectories/paths, move to a point, or change their facing."

    def __init__(self, actuate_function, localization_function=[]):
        self.id = None
        self.actuate = actuate_function
        self.locate = localization_function

        self.long_ultimate_gain = 9
        self.long_ultimate_period = .55  # should be determined later
        self.lateral_ultimate_gain = 10
        self.lateral_ultimate_period = 999  # should be determined later
        self.angular_ultimate_gain = 8
        self.angular_ultimate_period = .7  # should be determined later

        self.long_control = Proportional()  # Proportional()
        self.lateral_control = Proportional()
        self.angular_control = Proportional()

        self.dynamic_long_control = Proportional()
        self.dynamic_angular_control = Proportional()
        self.dynamic_long_control.setGain(self.long_ultimate_gain / 4)
        self.dynamic_angular_control.setGain(self.lateral_ultimate_gain / 4)

        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    def setID(self, agent_id):
        self.agent_id = agent_id

    def goToPoint(self, x, y):
        print 'Going to static point:', x, y
        bound = .01
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)

        # def getError():
        #     agent_id, x_actual, y_actual,z, theta_actual, pitch, roll = self.locate(self.agent_id)
        #     x_error = x - x_actual
        #     y_error = y - y_actual
        #     long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
        #     return long_error, lateral_error
        pos = self.locate(self.agent_id)
        agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = pos
        x_error = x - x_actual
        y_error = y - y_actual
        long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
        # long_error, lateral_error = getError()
        while (abs(long_error) > bound or abs(lateral_error) > bound) and pos == pos:
            pos = self.locate(self.agent_id)
            agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = pos
            x_error = x - x_actual
            y_error = y - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
            feedback_linear = self.long_control.controllerOutput(long_error)
            # print '% 4.2f, % 4.2f' % (long_error, feedback_linear), x,y, x_actual,y_actual,theta_actual
            feedback_angular = self.lateral_control.controllerOutput(lateral_error)
            self.actuate(feedback_linear, feedback_angular)
            # self.logger = append(self.logger, [[long_error, lateral_error,
            #                                 x, y,
            #                                 x_actual, y_actual,
            #                                 theta_actual, time()]], axis=0)
        print 'Longitutional error:', long_error, 'm | Lateral error:', lateral_error, 'm'

    def moveTowardsDynamicPoint(self, distance, theta):
        agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = self.locate(self.agent_id)
        long_error = distance * cos(theta)
        angular_error = wrapAnglePi(theta)
        #print 'long error:', long_error, 'angle error: ', angular_error
        #print 'long error: ',long_error,'ang error: ',angular_error
        # self.logger = append(self.logger, [[0, 0, 0, 0, x_actual, y_actual, theta_actual, time(),0]], axis=0)
        feedback_linear = self.dynamic_long_control.controllerOutput(long_error)
        feedback_angular = self.dynamic_angular_control.controllerOutput(angular_error)
        feedback_linear = max(0, feedback_linear)
        #print 'cmd lin: ', feedback_linear, 'cmd ang: ', feedback_angular
        self.actuate(feedback_linear, feedback_angular)

    def saveLog(self):
        save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/tracker_of_agent_'+str(self.agent_id), self.logger)
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    def faceDirection(self, theta_desired):
        theta_desired = wrapAnglePi(theta_desired)
        #print 'Facing direction.'
        bound = .02
        self.angular_control.setGain(self.angular_ultimate_gain / 4)
        theta_error = wrapAnglePi(theta_desired - self.locate(self.agent_id)[4])
        while abs(theta_error) > bound:
            feedback_angular = self.angular_control.controllerOutput(theta_error)
            # print feedback_angular
            self.actuate(0, feedback_angular)
            # print theta_desired,self.locate(self.agent_id)[4]
            agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = self.locate(self.agent_id)
            theta_error = wrapAnglePi(theta_desired - theta_actual)
            self.logger = append(self.logger, [[0, 0,theta_error,
                                                0,0, theta_desired,
                                                x_actual, y_actual, theta_actual, time()]], axis=0)
        self.actuate(0, 0)
        agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = self.locate(self.agent_id)
        theta_error = wrapAnglePi(theta_desired - theta_actual)
        #print 'Angular error:', theta_error, 'radians =', theta_error * 180 / pi, 'degrees'

    def followTrajectory(self, trajectory, x_calibrate=0, y_calibrate=0):
        print 'Following Trajectory.'
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        # self.angular_control.setGain(self.angular_ultimate_gain / 2)
        reference_pos = trajectory.getPosition()
        # print 'ref pos 1',reference_pos
        while reference_pos == reference_pos:  # check to see if x_reference is not NaN
            agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = self.locate(self.agent_id)
            # x_actual, y_actual, theta_actual = self.locate()
            x_reference, y_reference, theta_reference = reference_pos
            x_error = x_reference + x_calibrate - x_actual
            y_error = y_reference + y_calibrate - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
            angular_error = theta_reference - theta_actual

            feedback_linear = self.long_control.controllerOutput(long_error)
            feedback_angular = self.lateral_control.controllerOutput(lateral_error)
            # feedback_angular = feedback_angular + self.angular_control.controllerOutput(angular_error)

            self.actuate(feedback_linear, feedback_angular)

            reference_pos = trajectory.getPosition()
            # print 'ref pos',reference_pos
            self.logger = append(self.logger, [[long_error, lateral_error,
                                                x_reference + x_calibrate, y_reference + y_calibrate,
                                                x_actual, y_actual,
                                                theta_actual, time(), 0]], axis=0)

        save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()) + ' followTrajectory', self.logger)
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        print 'Reached end of trajectory.'

    def followPath(self, path, x_calibrate=0, y_calibrate=0):
        'Following path.'
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        # self.angular_control.setGain(self.angular_ultimate_gain / 2)
        reference_pos = path.getPosition()
        # print 'ref pos 1',reference_pos
        while reference_pos == reference_pos:  # check to see if x_reference is not NaN
            agent_id, x_actual, y_actual, z, theta_actual, pitch, roll = self.locate(self.agent_id)
            # x_actual, y_actual, theta_actual = self.locate()
            x_reference, y_reference, theta_reference = reference_pos
            x_error = x_reference + x_calibrate - x_actual
            y_error = y_reference + y_calibrate - y_actual
            long_error, lateral_error = rotate2DimFrame(x_error, y_error, theta_actual)
            angular_error = theta_reference - theta_actual
            lateral_penalty = abs(lateral_error**2) * (-100)
            # angular_penalty = abs(pi/2-theta_actual) * -.4*18/pi
            vel = max(.5 + lateral_penalty, 0.05)
            print '% 4.2f, % 4.2f, % 4.2f' % (vel, abs(lateral_error), abs(pi / 2 - theta_actual))
            path.setVelocity(vel)
            feedback_linear = self.long_control.controllerOutput(long_error)
            feedback_lateral = self.lateral_control.controllerOutput(lateral_error)
            feedback_angular = self.angular_control.controllerOutput(angular_error)
            # print feedback_angular, feedback_lateral

            self.actuate(feedback_linear, feedback_lateral)

            reference_pos = path.getPosition()
            # print 'ref pos',reference_pos
            self.logger = append(self.logger, [[long_error, lateral_error,
                                                x_reference + x_calibrate, y_reference + y_calibrate,
                                                x_actual, y_actual,
                                                theta_actual, time(), vel]], axis=0)

        save('/home/administrator/barzin_catkin_ws/src/path_tracking/scripts/experimental_results/' + str(datetime.now()) + ' followPath', self.logger)
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        print 'Reached end of path.'

    def followLoop(self):
        self.long_control.setGain(self.long_ultimate_gain / 2)
        self.lateral_control.setGain(self.lateral_ultimate_gain / 2)
        pass
