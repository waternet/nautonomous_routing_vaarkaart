#!/usr/bin/env python

import sys
# Prevent pyc files being generated!
sys.dont_write_bytecode = True

import rospy

import vaarkaart_helper
import vaarkaart_service

test_client = False # change this to see a test run.

if __name__ == "__main__":
	rospy.init_node('routing_vaarkaart_node')

	test_client = rospy.get_param('~test', False)

	vaarkaart_graph = vaarkaart_helper.load_vaarkaart()

	vaarkaart_service.vaarkaart_routing_service_server(vaarkaart_graph)

	if test_client:
		vaarkaart_service.vaarkaart_routing_service_client()
		
	rospy.spin()