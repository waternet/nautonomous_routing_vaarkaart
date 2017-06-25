
import random
import rospy
import sys

from geometry_msgs.msg import Pose2D
from nautonomous_routing_msgs.srv import Routing, RoutingResponse

import astar_route
import graph_helper
import vaarkaart_visualizer

routing_vaarkaart_service_namespace = "/routing/vaarkaart/request"
pose_utm_fix_topic = "/pose/utm/fix"

vaarkaart_graph = None
debug_visualization = False

def vaarkaart_routing_service_server(routing_graph, debug):
	global vaarkaart_graph, debug_visualization
	vaarkaart_graph = routing_graph
	debug_visualization = debug

	# Setup vaarkaart routing service
	vaarkaart_routing_service_server = rospy.Service(routing_vaarkaart_service_namespace, Routing, routing_vaarkaart_service)

# Find shortest route
def find_shortest_route(start_edge, destination_edge):
	global vaarkaart_graph
	# Temporarily variables to remember the shortest route
	shortest_distance = sys.maxint
	shortest_route = None

	# Check all permutations to check the shortest route.
	for i in range(0, 4):
		route, distance = astar_route.astar_search(vaarkaart_graph, start_edge[(i/2)], destination_edge[not (i%2)])
		if distance < shortest_distance:
			shortest_distance = distance
			shortest_route = route
	
	return shortest_distance, shortest_route

# Find the route to the destination.
def routing_vaarkaart_service(request):
	global vaarkaart_graph

	# Create the start coordinate and edge
	start_coordinate = str(request.start.x) + "," + str(request.start.y)
	start_edge = graph_helper.get_closest_edge(vaarkaart_graph, request.start.x, request.start.y)

	# Create the destination coordinate and edge
	destination_coordinate = str(request.destination.x) + "," + str(request.destination.y)
	destination_edge = graph_helper.get_closest_edge(vaarkaart_graph, request.destination.x, request.destination.y)

	# Temporarily variables to remember the shortest route
	shortest_distance, shortest_route = find_shortest_route(start_edge, destination_edge)

	# UTM route
	route_poses = []

	# Convert the route array from astar to a pose2d array for ros.
	for current_route in shortest_route:
		route_coordinate = map(float, current_route.split(','))
		# TODO give correct theta pose for next route segment
		route_poses.append(Pose2D(route_coordinate[0], route_coordinate[1], 0))

	# Add the start and destination to the route.
	start_pose = Pose2D(request.start.x, request.start.y, 0)
	if start_pose not in route_poses:
		route_poses.insert(0, start_pose)
	
	destination_pose = Pose2D(request.destination.x, request.destination.y, 0)
	if destination_pose not in route_poses:
		route_poses.append(destination_pose)

	# Visualize kaart route
	if debug_visualization:
		vaarkaart_visualizer.visualize_vaarkaart_route(vaarkaart_graph, start_coordinate, destination_coordinate, route_poses)

	return RoutingResponse(route_poses)

def vaarkaart_routing_service_client():
	global vaarkaart_graph

	rospy.wait_for_service(routing_vaarkaart_service_namespace)

	# Setup vaarkaart routing client
	vaarkaart_routing_service_client = rospy.ServiceProxy(routing_vaarkaart_service_namespace, Routing)

	# Try to make a request to the vaarkaart routing service with a randomly chosen start and destination position.
	try:
		start_edge = random.choice(vaarkaart_graph.edges())
		start_easting, start_northing = map(float,start_edge[0].split(","))

		destination_edge = random.choice(vaarkaart_graph.edges())
		destination_easting, destination_northing = map(float,destination_edge[0].split(","))

		print "Requesting vaarkaart routing"

		vaarkaart_routing_service_response = vaarkaart_routing_service_client(Pose2D(start_easting, start_northing, 0), Pose2D(destination_easting, destination_northing, 0))

		print "Success vaarkaart routing service client response: " + str(vaarkaart_routing_service_response.route)

	except rospy.ServiceException, e:
		print "Service call for vaarkaart routing failed: %s" % e
