#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import assignment_2_2024.msg

from std_srvs.srv import SetBool
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2024_client.msg import Pos_and_vel #custom .msg file inside msg directory
from actionlib_msgs.msg import GoalStatus

class GoalHandler:
	def __init__(self):
		rospy.init_node('target_client')
		#No target
		self.remove_target = True
		
		#Create SimpleAction Client
		self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
		self.client.wait_for_server()
		
		#subscribe Odometry
		self.check_odometry = rospy.Subscriber('/odom', Odometry, self.pub_vel_pos)
		
		#Publish on custom message
		self.pub = rospy.Publisher('/pos_and_vel', Pos_and_vel, queue_size=1)
		
	#Function that sets new target and delete the previous one	
	def set_goal(self):
		
		while not rospy.is_shutdown():
			#get current pose of the target
			target_pos_x = rospy.get_param('/des_pos_x')
			target_pos_y = rospy.get_param('/des_pos_y')
			#Create Goal Object
			target_goal = assignment_2_2024.msg.PlanningGoal()
			#pass it to action file
			target_goal.target_pose.pose.position.x = target_pos_x
			target_goal.target_pose.pose.position.y = target_pos_y
			rospy.loginfo ("Current target position: x = %f, y = %f", target_pos_x, target_pos_y)
			
			choice = input("Do you want to change the target? Press -> 't'\nDo you want to cancel the target? Press -> 'c'\n\nYour choice: ")
			if choice == 't':
				self.change_target(target_goal)
			elif choice == 'c' :
				self.cancel_target()
			else:
				rospy.logwarn("Invalid input, please try again...\n")
	
	#Change target method			
	def change_target(self, target_goal):
		#input new target position
		valid_input=False
		while not valid_input:
			try:
				new_target_pos_x = float(input("New target x position: "))
				new_target_pos_y = float (input("New target y position: "))
				valid_input = True
			except ValueError:
				rospy.logwarn("Invalid input. Please enter numeric value")
						
		#set new target position
		rospy.set_param ('/des_pos_x', new_target_pos_x)
		rospy.set_param('/des_pos_y', new_target_pos_y)
		
		#Update Target position to send it to the server
		target_goal.target_pose.pose.position.x=new_target_pos_x
		target_goal.target_pose.pose.position.y=new_target_pos_y
		
		#Send it to action server
		self.client.send_goal(target_goal)
		self.remove_target = False
		rospy.loginfo("New target set: x = %f, y = %f", new_target_pos_x, new_target_pos_y)
		rospy.logwarn("Invalid input. Please enter a numeric values."
		
	#Method for cancelling target goal
	def cancel_target(self):
		rospy.loginfo("Cancelling target...")
		if (not self.remove_target):
			self.client.cancel_goal()
			self.remove_target = True
			rospy.loginfo ("Target has been removed correctly")
		else:
			rospy.loginfo ("There's no target to remove.\nInsert a target...\n")
				
						
	def pub_vel_pos (self,data):
		msg = Pos_and_vel()
		msg.pos_x=data.pose.pose.position.x
		msg.pos_y=data.pose.pose.position.y
		msg.vel_x=data.twist.twist.linear.x
		msg.vel_y=data.twist.twist.linear.y
		
		self.pub.publish(msg)

def main():
	handler = GoalHandler()
	handler.set_goal()
	
if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
				
