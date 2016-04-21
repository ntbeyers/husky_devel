#!/usr/bin/env python

import ConfigParser

#define needed information for targets
class Target(object): 
	def __init__(self):
		x = 0
		y = 0
		approach_dist = 0
		approach_angle = 0
		min_dist = 1

if __name__ == "__main__":
	config = ConfigParser.ConfigParser()
	config.read('/home/naslab/husky_devel/src/microgrid_demo/scripts/robotConfigure.cfg')

	namespace = config.get('setup', 'namespace')
	agent_type = config.get('setup', 'agent_type')
	x_position = config.getfloat('setup', 'x_position')
	y_position = config.getfloat('setup', 'y_position')
	starting_angle = config.getfloat('setup','theta')
	target_number = config.getint('setup','targets')

	targets = []

	for index in range(target_number):
		targets.append(Target())
		targets[index].x = config.getfloat('target_'+ str(index+1), 'x_target')
		targets[index].y = config.getfloat('target_'+ str(index+1), 'y_target')
		targets[index].approach_dist = config.getfloat('target_'+ str(index+1), 'approach_dist')
		targets[index].approach_angle = config.getfloat('target_'+ str(index+1), 'approach_angle')
		targets[index].min_dist = config.getfloat('target_'+ str(index+1), 'min_dist')


