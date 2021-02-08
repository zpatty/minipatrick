#!/usr/bin/python

# one_step_planner.py
# ROS node that takes messages from 2d_state_est
# and sends to britlestar_actions
# Copyright 2020 Soft Machines Lab

import sys
# This import allows us to interpret messages from the cv tracker
import numpy as np

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(1,'{}/../../soft_robot_agents/src/'.format(dir_path))
from brittlestar_agent import BrittlestarAgent
from datetime import date
from datetime import time
from datetime import datetime

class OneStepPlanner:

	def __init__(self):

		#Get the transition model and actions from the robot
		actions,transitions,self.transition_dist = BrittlestarAgent.get_actions()
		self.transition_model = transitions
		# self.log_file = f
		# with open(self.log_file,"w") as f:
		# 	f.write("actions: {}\ntransitions: {}\n".format(actions,transitions))
		# 	f.write("Timestamp (millisec since midnight today), CoM X (cm), CoM Y (cm), Theta (deg), Goal X (cm), Goal Y (cm), Action\n")

		self.plan = []
		self.started = False
		print("Planning over the actions: {}".format(str(actions.keys())))


	def get_time_since_midnight(self):
		# get the current time (clock ticks of computer)
		now = datetime.now()
		# subtract from midnight. Midnight is
		midnight = now.replace(hour=0, minute=0, second=0, microsecond=0)
		# total microseconds, according to system clock, since midnight.
		# Is a float.
		usec = (now - midnight).total_seconds()
		# Turn into a in, with microseconds only.
		msec = int(round(usec * 1000))
		return msec

	def dispatch(self,message):
		if message.data == 1 and len(self.plan)>0 and self.goal_dist > self.transition_dist*2: #TODO: replace hardcode with self.transition_dist
			
			print(self.plan)
			self.action_primitive_pub.publish(self.plan)
			with open(self.log_file,"a") as f:
				current_time = self.get_time_since_midnight()
				f.write("{0},{1},{2},{3},{4},{5},{6}\n".format(current_time,self.x,self.y,self.theta,self.gx,self.gy,self.plan))
			self.plan = []

	
	def got_a_state(self, state):
		# message is a TwoDStateWithGoal.
		self.x =  state[0]
		self.y = state[1]
		self.theta = state[2]
		self.gx = state[3]
		self.gy = state[4]

		x,y,theta,gx,gy = (self.x,self.y,self.theta,self.gx,self.gy)
		theta = theta*(np.pi/180)
		if theta != []:
			print("Got a state estimate with:")
			print("X= {0}, Y= {1}, Theta= {2}".format(x,y,theta))
			self.goal_dist = np.linalg.norm([gx-x,gy-y])
			print("Goal dist = {}".format(self.goal_dist))

			dists = []
			for action, state_change in self.transition_model.items():
				dx, dy, dtheta = [delta for delta in state_change]
				next_x = x + dx * np.cos(theta) - dy *np.sin(theta)
				next_y = y + dx * np.sin(theta) + dy * np.cos(theta)
				dists.append([action, np.linalg.norm([gx-next_x, gy-next_y])])
			
			sorted_actions = sorted(dists, key = lambda item : item[1],reverse=False) 
			closest_action  = sorted_actions[0][0]
		

			self.plan = closest_action
			if self.started == False:
				print("Starting planning messages.")
				self.started = True
			return closest_action


		# Otherwise, do nothing this round.

def main(args):
	if len(args) == 1:
		file_save_location = "{}.csv".format(raw_input("Where to save the log file?"))
	else:
		file_save_location = "{}.csv".format(sys.argv[1])
	# For now, hard-code the topic names.
	state_est_topic_name = "/2d_state_est"
	action_primitive_topic_name = "/brittlestar_actions"
	# start up a ROS node for this
	rospy.init_node('one_step_planner')
	# Initialize the object, that should be all
	planner = OneStepPlanner(state_est_topic_name, action_primitive_topic_name,file_save_location)
	# wait until keyboard interrupt.
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Caught Ctrl-C, shutting down one_step_planner.")

# ...and if called as an executable,
if __name__ == '__main__':
	main(sys.argv)