#!/usr/bin/env python
"""
.. module:: service_node
   :platform: Unix
   :synopsis: Python service node for retrieving the last target position.

.. moduleauthor:: Francesca Amato <s7827998@studenti.unige.it>

This module implements a ROS service node that provides the last target position to clients
via a custom service message, `Target_srv`. The service retrieves the target position
stored in ROS parameters (`/des_pos_x` and `/des_pos_y`).

Service:
    - **/target_srv** (:class:`assignment_2_2024_client.srv.Target_srv`)
      Responds with the most recent target position.

Custom Service:
    - **Target_srv**: A service defined in the package `assignment_2_2024_client`. It includes the following fields:
        - **Request**: No fields (empty request).
        - **Response**:
          - **target_pos_x**: Float, representing the X coordinate of the last target.
          - **target_pos_y**: Float, representing the Y coordinate of the last target.
"""

import rospy
from assignment_2_2024_client.srv import Target_srv, Target_srvResponse

class FindLastTarget:
	"""
	A class that provides the last target position through a service.

	This node initializes a ROS service `/target_srv`, which uses the custom
	service type `Target_srv`. It retrieves the last target's X and Y positions
	from ROS parameters and sends them back to the client as a response.

	:ivar service: The ROS service object.
	:vartype service: :class:`rospy.Service`

    	"""
	def __init__(self):
		"""
        	Initializes the `FindLastTarget` node and sets up the `/target_srv` service.

        	:raises ROSInterruptException: If there is an issue with ROS initialization.
        	"""
		rospy.init_node("service_node")
		
		#Creating Service
		rospy.Service("target_srv", Target_srv, self.handle_target_srv)
		rospy.loginfo("Service ready...\n")
		
		
		#Responding to service by giving the user the last position of the target
	def handle_target_srv(self, request):
		"""
		Handles requests to the `/target_srv` service by returning the last target's position.

		:param request: The service request (unused as no fields are defined in the request).
		:type request: :class:`assignment_2_2024_client.srv.Target_srvRequest`
		:return: The response containing the X and Y coordinates of the last target.
		:rtype: :class:`assignment_2_2024_client.srv.Target_srvResponse`
		"""
		last_coordinate = Target_srvResponse()
		last_coordinate.target_pos_x = rospy.get_param('/des_pos_x')
		last_coordinate.target_pos_y = rospy.get_param('/des_pos_y')
		
		return last_coordinate
		
		#Printing on screen the coordinate of the target
	

if __name__ == '__main__':
	"""
	Entry point for the service node. Initializes the `FindLastTarget` class
	and starts the ROS spin loop to keep the service running.

	:return: None
	"""
	try:
		node = FindLastTarget ()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Error has occured, shutting down Target Service...")
