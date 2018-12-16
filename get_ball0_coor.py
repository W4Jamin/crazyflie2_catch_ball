#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Vector3;

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
	}

	def get_ball_coordinates(self):
		try:
			ball_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			pub = rospy.Publisher('ball_0_coor', Vector3, queue_size=10)
			rospy.init_node('ball_0_pub', anonymous=True)
			print('connect success!!')
			# 3D matrix (4 set of coordinates in 4 second, 3 balls, coordinate of each ball)
			coordinates = np.zeros((4, 3), dtype = np.float)
			while(1):
				ball0_coordinates = ball_coordinates('ball_0', 'link')
				if ball0_coordinates.pose.position.x > 9.5:
					time.sleep(1)
					for i in range(4):
						for j in range(3):
							resp_coordinates = ball_coordinates('ball_0', 'link')
							coordinates[i, 0] = resp_coordinates.pose.position.x
							coordinates[i, 1] = resp_coordinates.pose.position.y
							coordinates[i, 2] = resp_coordinates.pose.position.z
						time.sleep(2)
					print(coordinates)

					f_y = coordinates[2, 1] - coordinates[1][1]
					f_x = coordinates[2, 0] - coordinates[1][0]
					# x_target = 0
					# y_target = -(9.5 * f_y - 2.5 * f_x) / f_x
					# z_target = 1

					vec_msg = Vector3()
					vec_msg.x = 0
					vec_msg.y = coordinates[1][1]-(coordinates[1][1] - coordinates[2][1]) * coordinates[1][0] / (coordinates[1][0] - coordinates[2][0])
					vec_msg.z = 1.5
					pub.publish(vec_msg)
				time.sleep(0.1)

		except rospy.ServiceException as e:
			ROS_INFO("nono..")
			rospy.loginfo("Get model state service call failed: {0}".format(e))

if __name__ == '__main__':
	get_coor = GetBallCor()
	get_coor.get_ball_coordinates()