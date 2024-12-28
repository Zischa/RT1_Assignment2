#!/usr/bin/env python 3

import rospy
import actionlib
import actionlib.msg
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import Vel, PlanningAction, PlanningGoal
from actionlib_msgs.msg import GoalStatus

class GoalHandler:
	def __init__(self):
		rospy.init_node('target_client')
		self.pub = rospy.Publisher ("/pos_vel", Vel, queue_size = 1)
		self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
		self.client.wait_for_server()

		
	def set_goal(self):
		rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
		while not rospy.is_shutdown():
			#get current pose of the target
			target_pos_x = rospy.get_param('/des_pos_x')
			target_pos_y = rospy.get_param('/des_pos_y')
			#Create Goal Object
			target_goal = PlanningGoal()
			#pass it to action file
			target_goal.target_pose.pose.position.x = target_pos_x
			target_goal.target_pose.pose.position.y = target_pos_y
			rospy.loginfo ("Current target position: x = %f, y = %f", target_pos_x, target_pos_y)
			choice = input("Do you want to change the target? Press -> 't'\n Do you want to cancel the target? Press -> 'c'")
			if choice == 't':
				#input new target position
				new_target_pos_x = float(input("New target x position: "))
				new_target_pos_y = float (input("New target y position: "))
				
				
				#aggiungi possibilità di errore in input....
				
				
				#set new target position
				rospy.set_param ('/des_pos_x', new_target_pos_x)
				rospy.set_param('/des_pos_y', new_target_pos_y)
				#Update Target position to send it to the server
				target_goal.target_pose.pose.position.x=new_target_pos_x
				target_goal.target_pose.pose.position.y=new_target_pos_y
				#Send it to action server
				self.client.send_goal(target_goal)
				self.delete_goal = False
			if choice == 'c' :
				rospy.loginfo("Cancelling target...")
				self.client.cancel_goal()
				
				#E se non ci sono target??
				
			#aggiungi errore in caso di choice diverso da c e t
			
#aggiungi pub and sub per velocità e posizione

def main():
	handler = GoalHandler()
	handler.set_goal()
	
if __name== '__main__':
	main()	
				
