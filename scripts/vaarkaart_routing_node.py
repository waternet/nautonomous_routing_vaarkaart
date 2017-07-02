#!/usr/bin/env python

import sys
import os
# Prevent pyc files being generated!
sys.dont_write_bytecode = True

import rospy

sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/vaarkaart'))
sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/astar'))

print sys.path

import vaarkaart_helper
from vaarkaart_service import VaarkaartService
from vaarkaart_client import vaarkaart_routing_service_client

test_client = False # change this to see a test run.

if __name__ == "__main__":
	rospy.init_node('routing_vaarkaart_node')

	test_client = rospy.get_param('~test', False)

	vaarkaart_graph = vaarkaart_helper.load_vaarkaart()

	vaarkaart_service = VaarkaartService(vaarkaart_graph)

	if test_client:
		vaarkaart_routing_service_client(vaarkaart_graph)
		
	rospy.spin()