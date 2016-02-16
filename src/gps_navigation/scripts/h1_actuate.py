import rospy
from geometry_msgs.msg import Twist

nan_var = float('nan')
stop_msg = Twist()


def saturate(value, limit=1):
    return min(limit, max(-limit, value))


class ActuationError(Exception):

    def __init__(self, message, errors=-1):

        # Call the base class constructor with the parameters it needs
        super(ActuationError, self).__init__(message)

        # Now for your custom code...
        self.errors = errors


class ROS2DimActuate(object):

    "This class controls wheels ground robots equipped with ROS."

    def __init__(self):
        # setup ROS node and topics
        #rospy.init_node('two_dim_actuation_control')
        self.pub_cmd = rospy.Publisher('/husky1/cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()

        self.rate = rospy.Rate(40)
        self.tangential_limit = 1
        self.angular_limit = 2

    def setRate(self, rate):
        self.rate = rospy.Rate(rate)

    def setTangentialVelocityLimit(self, limit):
        if limit < 0:
            raise ValueError('Saturation limit should be a positive value!')
        self.tangential_limit = limit

    def setAngularVelocityLimit(self, limit):
        if limit < 0:
            raise ValueError('Saturation limit should be a positive value!')
        self.angular_limit = limit

    def actuate(self, tangential_velocity, angular_velocity):
        if rospy.is_shutdown():
            raise ActuationError('rospy is shut down.')
        self.twist_msg.linear.x = saturate(tangential_velocity, self.tangential_limit)
        self.twist_msg.angular.z = saturate(angular_velocity, self.angular_limit)
        # print self.twist_msg.angular.z
        if tangential_velocity + angular_velocity is nan_var:
            self.pub_cmd.publish(stop_msg)
            raise ActuationError('Actuation command is NaN.')
        else:
            self.pub_cmd.publish(self.twist_msg)
        self.rate.sleep()
