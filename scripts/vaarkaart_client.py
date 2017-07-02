import rospy
from vaarkaart_service import routing_vaarkaart_service_namespace
import random
import utm_helper

from geometry_msgs.msg import Pose2D
from nautonomous_routing_msgs.srv import Route

def vaarkaart_routing_service_client(vaarkaart_graph):

	rospy.wait_for_service(routing_vaarkaart_service_namespace)

	# Setup vaarkaart routing client
	vaarkaart_routing_service_client = rospy.ServiceProxy(routing_vaarkaart_service_namespace, Route)

	# Try to make a request to the vaarkaart routing service with a randomly chosen start and destination position.
	try:
		start_edge = random.choice(vaarkaart_graph.edges())
		start_easting, start_northing = utm_helper.map_float_split(start_edge[0])

		destination_edge = random.choice(vaarkaart_graph.edges())
		destination_easting, destination_northing = utm_helper.map_float_split(destination_edge[0])

		print "Requesting vaarkaart routing"

		vaarkaart_routing_service_response = vaarkaart_routing_service_client(Pose2D(start_easting, start_northing, 0), Pose2D(destination_easting, destination_northing, 0))

		print "Success vaarkaart routing service client response: " + str(vaarkaart_routing_service_response.route)

	except rospy.ServiceException, e:
		print "Service call for vaarkaart routing failed: %s" % e
