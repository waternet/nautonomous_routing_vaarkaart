#!/usr/bin/env python

import rospy

import vaarkaart_helper
import vaarkaart_service

debug_visualization = False # change this to see visualization
test_client = False # change this to see a test run.

if __name__ == "__main__":
	rospy.init_node('routing_vaarkaart_node')

	debug_visualization = rospy.get_param('~debug', False)
	test_client = rospy.get_param('~test', False)

	vaarkaart_graph = vaarkaart_helper.load_vaarkaart()

	vaarkaart_service.vaarkaart_routing_service_server(vaarkaart_graph, debug_visualization)

	if test_client:
		vaarkaart_service.vaarkaart_routing_service_client()

	rospy.spin()