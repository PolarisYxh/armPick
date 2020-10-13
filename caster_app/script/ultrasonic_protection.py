#!/usr/bin/env python

import time
import rospy
import sys
import threading
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class UltrasonicProtection():
	"""docstring for Log"""
	def __init__(self):
		self._sub_ultrasonic_front_left = rospy.Subscriber("/dauxi_ks106_node/ultrasonic_front_left", Range, self.ultrasonicFrontLeftCallback)
		self._sub_ultrasonic_front_right = rospy.Subscriber("/dauxi_ks106_node/ultrasonic_front_right", Range, self.ultrasonicFrontRightCallback)
		self._sub_ultrasonic_rear_left = rospy.Subscriber("/dauxi_ks106_node/ultrasonic_rear_left", Range, self.ultrasonicRearLeftCallback)
		self._sub_ultrasonic_rear_right = rospy.Subscriber("/dauxi_ks106_node/ultrasonic_rear_right", Range, self.ultrasonicRearLeftCallback)
		# self._sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmdvelCallback)
		self._pub_protection_cmd_vel = rospy.Publisher("/yocs_cmd_vel_mux/input/safety_cmd", Twist, queue_size=1)
		
		self.front_left_obstacle = 0
		self.front_right_obstacle = 0
		self.rear_left_obstacle = 0
		self.rear_right_obstacle = 0
		self.protection_stop = False

	def ultrasonicFrontLeftCallback(self, msg):
		if msg.range < 0.3 or msg.range == -1.0:
			self.front_left_obstacle += 1
		else:
			self.front_left_obstacle = 0

	def ultrasonicFrontRightCallback(self, msg):
		if msg.range < 0.3 or msg.range == -1.0:
			self.front_right_obstacle += 1
		else:
			self.front_right_obstacle = 0
	def ultrasonicRearLeftCallback(self, msg):
		if msg.range < 0.3 or msg.range == -1.0:
			self.rear_left_obstacle += 1
		else:
			self.rear_left_obstacle = 0
	def ultrasonicRearLeftCallback(self, msg):
		if msg.range < 0.3 or msg.range == -1.0:
			self.rear_right_obstacle += 1
		else:
			self.rear_right_obstacle = 0

	def protectionCmdVelPub(self):

		cmd_vel = Twist()
		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.linear.z = 0
		cmd_vel.angular.x = 0
		cmd_vel.angular.y = 0
		cmd_vel.angular.z = 0
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			if self.protection_stop == True:
				self._pub_protection_cmd_vel.publish(cmd_vel)
			rate.sleep()
			
	def statusMonitor(self):
		# rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.front_left_obstacle == 3 or self.front_right_obstacle == 3 or self.rear_left_obstacle == 3 or self.rear_right_obstacle == 3:
				# if self.front_left_obstacle == 3:
				# 	rospy.loginfo("worning: front left obstacle too close")
				# if self.front_right_obstacle == 3:
				# 	rospy.loginfo("worning: front right obstacle too close")
				# if self.rear_left_obstacle == 3:
				# 	rospy.loginfo("worning: rear left obstacle too close")
				# if self.rear_right_obstacle == 3:
				# 	rospy.loginfo("worning: rear right obstacle too close")
				self.protection_stop = True
			elif self.front_left_obstacle == 0 and self.front_right_obstacle == 0 and self.rear_left_obstacle == 0 and self.rear_right_obstacle == 0: 
				self.protection_stop = False
			time.sleep(0.01)
			# rate.sleep()


def main():
	rospy.init_node('ultrasonic_protection')
	UP = UltrasonicProtection()
	# rospy.sleep(1.0)
	t_status_monitor = threading.Thread(target=UP.statusMonitor, args=())
	t_status_monitor.start()
	t_cmd_vel = threading.Thread(target=UP.protectionCmdVelPub, args=())
	t_cmd_vel.start()
	rospy.spin()

if __name__ == "__main__":
  try:
	main()
  except rospy.ROSInterruptException:
	pass