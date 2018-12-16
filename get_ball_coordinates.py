#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
import numpy as np
import time

class Block:
	"""docstring for Block"""
	def __init__(self, model_name, link_name):
		self._model_name = model_name
		self._link_name = link_name

class GetBallCor:

	_ballDict = {
		'ball_0': Block('ball_0', 'link'),
		'ball_1': Block('ball_1', 'link'),
		'ball_2': Block('ball_2', 'link'),
	}

	def get_ball_coordinates(self):
		try:
			ball_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			print('connect success!!')
			# 3D matrix (4 set of coordinates in 4 second, 3 balls, coordinate of each ball)
			coordinates = np.zeros((4, 3, 3), dtype = np.float)

			for i in range(4):
				for block, j in zip(self._ballDict.itervalues(), range(3)):
					resp_coordinates = ball_coordinates(block._model_name, block._link_name)
					coordinates[i, j, 0] = resp_coordinates.pose.position.x
					coordinates[i, j, 1] = resp_coordinates.pose.position.y
					coordinates[i, j, 2] = resp_coordinates.pose.position.z
					print(resp_coordinates.twist.linear.x)
					#coor = [pos_x, pos_y, pos_z]
					#coordinates[i].append(coor)
				time.sleep(1)
			print(coordinates)
			return coordinates				

		except rospy.ServiceException as e:
			ROS_INFO("nono..")
			rospy.loginfo("Get model state service call failed: {0}".format(e))

if __name__ == '__main__':
	get_coor = GetBallCor()
	get_coor.get_ball_coordinates()