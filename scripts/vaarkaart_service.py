
import random
import rospy
import sys

from geometry_msgs.msg import Pose2D
from nautonomous_routing_msgs.srv import Route, RouteResponse

import astar_route
import graph_helper
import vaarkaart_visualizer

routing_vaarkaart_service_namespace = "route_service"

class VaarkaartService:

	def __init__(self, routing_graph):
		self.vaarkaart_graph = routing_graph

		self.debug_visualization = rospy.get_param('~debug', False)
		
		# Setup vaarkaart routing service
		vaarkaart_routing_service_server = rospy.Service(routing_vaarkaart_service_namespace, Route, self.routing_vaarkaart_service)

	# Find shortest route
	def find_shortest_route(self, start_edge, destination_edge):
		# Temporarily variables to remember the shortest route
		shortest_distance = sys.maxint
		shortest_route = None

		# Check all permutations to check the shortest route.
		for i in range(0, 4):
			route, distance = astar_route.astar_search(self.vaarkaart_graph, start_edge[(i/2)], destination_edge[not (i%2)])
			if distance < shortest_distance:
				shortest_distance = distance
				shortest_route = route
		
		return shortest_distance, shortest_route

	# Find the route to the destination.
	def routing_vaarkaart_service(self, request):
		# Create the start coordinate and edge
		start_coordinate = str(request.start.x) + "," + str(request.start.y)
		start_edge = graph_helper.get_closest_edge(self.vaarkaart_graph, request.start.x, request.start.y)

		# Create the destination coordinate and edge
		destination_coordinate = str(request.destination.x) + "," + str(request.destination.y)
		destination_edge = graph_helper.get_closest_edge(self.vaarkaart_graph, request.destination.x, request.destination.y)

		# Temporarily variables to remember the shortest route
		shortest_distance, shortest_route = self.find_shortest_route(start_edge, destination_edge)

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
		if self.debug_visualization:
			vaarkaart_visualizer.visualize_vaarkaart_route(self.vaarkaart_graph, start_coordinate, destination_coordinate, route_poses)

		return RouteResponse(route_poses)

