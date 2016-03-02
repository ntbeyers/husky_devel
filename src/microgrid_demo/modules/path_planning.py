from numpy import cos, sin, pi, sqrt, arctan2
from time import sleep
from rotation import wrapAnglePi
# from time import time
# nan = float('nan')


class PathPlanningError(Exception):

    def __init__(self, message, errors=-1):
        super(PathPlanningError, self).__init__(message)


class GapFinder(object):

    def __init__(self, safe_radius):
        self.safe_radius = safe_radius * 1.0
        self.lidar_offset = .23
        self.safe_gap = self.safe_radius**2 * 4
        self.distance_range = 15.0
        self.readings_polar = []
        self.subgoals = []
        self.number_of_readings = 0

    def setDistanceRange(self, distance_range):
        if distance_range > 0:
            self.distance_range = distance_range
        else:
            raise PathPlanningError('Range of sensor should be greater than zero.')

    def polarToCartesian(self):
        x_cordinate = [reading[0] * cos(reading[1]) for reading in self.readings_polar]
        y_cordinate = [reading[0] * sin(reading[1]) for reading in self.readings_polar]

        return x_cordinate, y_cordinate

    def filterReadings(self, distances, angles):
        number_of_unfiltered_readings = len(distances)
        angles_trasfered = [[]] * number_of_unfiltered_readings
        distances_trasfered = [[]] * number_of_unfiltered_readings
        self.minimum_reading = []
        self.minimum_reading_angle = []

        inner_bound = .4

        for i in range(number_of_unfiltered_readings):
            if distances[i] < inner_bound:
                distances[i] = -1
            else:
                x = distances[i] * cos(angles[i]) + self.lidar_offset
                y = distances[i] * sin(angles[i])
                distances_trasfered[i] = sqrt(x**2 + y**2)
                angles_trasfered[i] = arctan2(y, x)
                # print angles_trasfered[40]
                if distances[i] >= 0 and distances[i] < self.minimum_reading:
                    self.minimum_reading = distances[i]
                    self.minimum_reading_angle = angles[i]

        self.readings_polar = [[distances_trasfered[i], angles_trasfered[i]] for i in range(number_of_unfiltered_readings) if distances[i] != -1]
        self.number_of_readings = len(self.readings_polar)
        # print self.readings_polar[40]
        # for i in range(self.number_of_readings):
        #     print self.readings_polar[i]

    def getMinimumReading(self):
        return self.minimum_reading, self.minimum_reading_angle

    def findGaps(self):
        d = self.safe_radius**2
        self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]

        for i in range(self.number_of_readings):

            for j in range(i - 1, -1, -1):
                angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:
                    if self.readings_polar[j][0] > self.safe_radius:
                        self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
                    else:
                        self.possible_travel[i] = 0

            for j in range(i + 1, self.number_of_readings):
                angular_difference = self.readings_polar[j][1] - self.readings_polar[i][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:
                    if self.readings_polar[j][0] > self.safe_radius:
                        self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))
                    else:
                        self.possible_travel[i] = 0

    def getFrontTravel(self, distances, angles):
        #initialize matrices
        number_of_unfiltered_readings = len(distances)
        angles_trasfered = [[]] * number_of_unfiltered_readings
        distances_trasfered = [[]] * number_of_unfiltered_readings
        self.minimum_reading = []
        self.minimum_reading_angle = []

        inner_bound = .05
        # eliminate readings below inner bound
        for i in range(number_of_unfiltered_readings):
            if distances[i] < inner_bound:
                distances[i] = -1
            else:
                x = distances[i] * cos(angles[i]) + self.lidar_offset
                y = distances[i] * sin(angles[i])
                distances_trasfered[i] = sqrt(x**2 + y**2)
                angles_trasfered[i] = arctan2(y, x)

                if distances[i] >= 0 and distances[i] < self.minimum_reading:
                    self.minimum_reading = distances[i]
                    self.minimum_reading_angle = angles[i]

        #create polar pair for readings above bound
        self.readings_polar = [[distances_trasfered[i], angles_trasfered[i]] for i in range(number_of_unfiltered_readings) if distances[i] != -1]
        self.number_of_readings = len(self.readings_polar)

        d = self.safe_radius**2
        #how far can robot safely travel in any direction
        self.possible_travel = [reading[0] - self.safe_radius for reading in self.readings_polar]

        for i in range(self.number_of_readings):
            #count down from i to -1
            for j in range(i - 1, -1, -1):
                angular_difference = self.readings_polar[i][1] - self.readings_polar[j][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:

                    self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))

            for j in range(i + 1, self.number_of_readings):
                angular_difference = self.readings_polar[j][1] - self.readings_polar[i][1]
                if angular_difference > pi / 2:
                    break
                if self.readings_polar[j][0] > self.readings_polar[i][0]:
                    continue
                c = (self.readings_polar[j][0] * sin(angular_difference))**2
                a = (self.readings_polar[j][0] * cos(angular_difference))
                if c < d:

                    self.possible_travel[i] = min(self.possible_travel[i], a - sqrt(d - c))

        for i in range(len(self.readings_polar)):
            if self.readings_polar[i][1]>0:
                return min(self.possible_travel[i],self.possible_travel[i-1])

    def findSubgoals(self):
        self.subgoals = [0, self.number_of_readings - 1]
        for i in range(self.number_of_readings - 1):
            possible_travel_change = self.possible_travel[i + 1] - self.possible_travel[i]
            if possible_travel_change > self.safe_radius:
                self.subgoals.append(i + 1)
            elif -possible_travel_change > self.safe_radius:
                self.subgoals.append(i)

        # return self.subgoals

    def selectSubgoal(self, distance, angle):
        # checks which subgoal is closer to the target and returns its index number
        best_subgoal = 0
        best_distance = distance**2 + self.possible_travel[0]**2 - 2 * distance * self.possible_travel[0] * cos(self.readings_polar[0][1] - angle)

        for subgoal in self.subgoals[1:]:
            distance_to_target_sq = distance**2 + self.possible_travel[subgoal]**2 - 2 * distance * \
                self.possible_travel[subgoal] * cos(self.readings_polar[subgoal][1] - angle)
            if distance_to_target_sq < best_distance and self.possible_travel[subgoal] > self.safe_radius:
                best_subgoal = subgoal
                best_distance = distance_to_target_sq

        return best_subgoal

    def isObstacleInTheWay(self, distance, angle):
        nearest_reading = 0
        for i in range(self.number_of_readings):
            if self.readings_polar[i][1] - angle > 0:
                nearest_reading = i
                break

        safe_travel = min(self.possible_travel[nearest_reading], self.possible_travel[nearest_reading - 1])
        # self.maximum_travel_to_target = safe_travel
        if distance < safe_travel:
            return 'safe',distance
        # elif distance < safe_travel + self.safe_radius and nearest_reading != 0:
        #     return 'close_to_obstacle',safe_travel
        else:
            return 'not_safe',safe_travel

    def planPath(self, distance, angle):
        self.findGaps()
        angle = wrapAnglePi(angle)
        environment_state, max_travel = self.isObstacleInTheWay(distance, angle)
        # print environment_state
        if environment_state is 'safe':
            return distance, angle
        elif environment_state is 'not_safe':
            self.findSubgoals()
            best_subgoal = self.selectSubgoal(distance, angle)
            return self.possible_travel[best_subgoal], self.readings_polar[best_subgoal][1]
        elif environment_state is 'close_to_obstacle':
            print '\033[93m'+'\033[1m'+'WARNING: Target is too close to an obstacle!!!'+'\033[0m'
            return max_travel, angle
        else:
            raise PathPlanningError.ImplementationError('Something wrong in planPath method of' + __name__)

    def pathToGlobalTarget(self, target_pos, agent_pos):
        # x, y, theta = localizer()
        diff_x = target_pos[0] - agent_pos[0]
        diff_y = target_pos[1] - agent_pos[1]
        distance = sqrt(diff_x**2 + diff_y**2)
        angle = arctan2(diff_y, diff_x) - agent_pos[2]
        subgoal_distance, subgoal_angle = self.obstacleAvoidance(distance, -angle)
