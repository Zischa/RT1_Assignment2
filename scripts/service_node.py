#!/usr/bin/env python

import rospy
from assignment_2_2024_client.srv import Target_srv, Target_srvResponse

class FindLastTarget:
	def __init__(self):
		rospy.init_node("service_node")
		
		#Creating Service
		rospy.Service("target_srv", Target_srv, self.handle_target_srv)
		rospy.loginfo("Service ready...\n")
		
		#Responding to service by giving the user the last position of the target
	def handle_target_srv(sel, request):
		last_coordinate = Target_srvResponse()
		last_coordinate.target_pos_x = rospy.get_param('/des_pos_x')
		last_coordinate.target_pos_y = rospy.get_param('/des_pos_y')
		
		return last_coordinate

if __name__ == '__main__':
	try:
		node = FindLastTarget ()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Error has occured, shutting down Target Service...")
