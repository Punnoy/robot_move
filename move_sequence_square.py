#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Twist


class move_robot:
	
	def __init__(self):
		self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
		self.cmd_vel_val = Twist()
		self.detect = False
		self.data_id = 0
		self.target = 0
		self.k = True
		self.Hz = 10 
		self.rate = rospy.Rate(self.Hz)

	def callback(self,data):
		if data.markers != []: 
			self.detect = True
			self.data_id = data.markers[0].id
			self.distance_z = data.markers[0].pose.pose.position.z
			self.distance_x = data.markers[0].pose.pose.position.x
			self.move_until_close()
		else:
			self.detect=False
			self.move_until_close()
		

	def move_until_close(self):
		if self.detect == True and self.data_id == self.target:
			rospy.loginfo("id = %d, distance z = %f, x = %f", self.data_id, self.distance_z, self.distance_x)
			if self.distance_z<0.3:
				#stop
				self.cmd_vel_val.linear.x = 0
				self.pub.publish(self.cmd_vel_val)
				rospy.loginfo("Finish, target %d",self.target)
				
				#moveback
				self.move_back()
				rospy.sleep(1)

				#turn_90
				self.turn_right_90()
				rospy.sleep(1)

				self.target += 1
				if self.target == 4:
					self.target = 0
				rospy.loginfo("start  next target %d",self.target)
				

			else:
				if self.distance_x<=0.05 and self.distance_x>=-0.05:
					self.cmd_vel_val.linear.x = 0.3
					
				elif self.distance_x>0.05:
					self.cmd_vel_val.linear.x = 0.3
					self.cmd_vel_val.angular.z = -0.2
				else:
					self.cmd_vel_val.linear.x = 0.3
					self.cmd_vel_val.angular.z = 0.2

				self.pub.publish(self.cmd_vel_val)

		else:
			self.cmd_vel_val.linear.x = 0
			self.pub.publish(self.cmd_vel_val)
			rospy.loginfo("No detect QR-code number : %d",self.target)
	
	def move_back(self):
		self.cmd_vel_val.linear.x = -0.1
		self.cmd_vel_val.angular.z = 0
		time = 3*self.Hz
		i = 0
		while i <= time:
			self.pub.publish(self.cmd_vel_val)
			i += 1
			self.rate.sleep()
		
	
	def turn_right_90(self):
		self.cmd_vel_val.linear.x = 0.1
		self.cmd_vel_val.angular.z = -0.4
		time = 4.5*self.Hz
		i = 0
		while i <= time:
			if sdata.markers != []:
				break

			self.pub.publish(self.cmd_vel_val)
			i += 1
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node("robot_move")
	x = move_robot()
	rospy.spin()
