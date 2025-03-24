#!/usr/bin/env python

"""
.. module:: Target_client
   :platform: Unix
   :synopsis: Python module for target client functionality.

.. moduleauthor:: Francesca Amato <s7827998@studenti.unige.it>

This module defines a ROS client for setting and managing goal targets
using `SimpleActionClient`. It also handles publishing positional
and velocity data using a custom message type.

Publisher:
    - **/pos_and_vel** (:class:`assignment_2_2024_client.msg.Pos_and_vel`)
      Publishes the current position and velocity of the robot.

Subscriber:
    - **/odom** (:class:`nav_msgs.msg.Odometry`)
      Subscribes to odometry data for position and velocity tracking.

Action Client:
    - **/reaching_goal** (:class:`assignment_2_2024.msg.PlanningAction`)
      Sends goals to the action server to manage target positions.

Services:
    None.
"""


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
	"""
	A class to handle goal-setting, goal cancellation, and publishing robot position and velocity in a ROS-based system.

	:ivar remove_target: A flag to indicate whether the current target should be removed.
	:vartype remove_target: bool
	:ivar client: The SimpleActionClient for interacting with the goal action server.
	:vartype client: :class:`actionlib.SimpleActionClient`
	:ivar check_odometry: Subscriber for odometry updates.
	:vartype check_odometry: :class:`rospy.Subscriber`
	:ivar pub: Publisher for position and velocity messages.
	:vartype pub: :class:`rospy.Publisher`
	"""
	def __init__(self):
		""" 
		Initializes the GoalHandler node and sets up the clients, subscriber, and publisher.
		
		:raises ROSInterruptException: If ROS initialization fails.
		"""
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
		"""
		Prompts the user to set a new target or cancel the current target.

		This function interacts with the ROS parameter server to fetch and modify target positions. It allows users to provide input dynamically.

		:return: None
		"""
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
		"""
		Changes the target goal by requesting new target positions from the user.

		:param target_goal: The current goal object to be updated.
		:type target_goal: :class:`assignment_2_2024.msg.PlanningGoal`
		
				    
		Return:
		    None

		"""
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
		
	#Method for cancelling target goal
	def cancel_target(self):
		"""
		Cancel the current goal if one is set.
		
		Args:
		    None
		    
		Return:
		    None

		"""
		rospy.loginfo("Cancelling target...")
		if (not self.remove_target):
			self.client.cancel_goal()
			self.remove_target = True
			rospy.loginfo ("Target has been removed correctly")
		else:
			rospy.loginfo ("There's no target to remove.\nInsert a target...\n")
				
						
	def pub_vel_pos (self,data):
		"""
		Publishes the robot's current position and velocity to a custom topic.
		
		:param data: Odometry data used to extract position and velocity.
		:type data: :class:`nav_msgs.msg.Odometry`
		
		:return: None
		
		"""
		msg = Pos_and_vel()
		msg.pos_x=data.pose.pose.position.x
		msg.pos_y=data.pose.pose.position.y
		msg.vel_x=data.twist.twist.linear.x
		msg.vel_y=data.twist.twist.linear.y
		
		self.pub.publish(msg)

def main():
	"""
	Entry point for the script. Initializes the GoalHandler instance and calls the set_goal method.
	"""
	handler = GoalHandler()
	handler.set_goal()
	
if __name__== '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
				
