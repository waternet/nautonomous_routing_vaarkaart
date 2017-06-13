#!/usr/bin/env python

import rospy
import pyproj
import json
import os
import math

import utm

import time
import random

import networkx as nx
import matplotlib.pyplot as plt

from vertex import Vertex
from geometry_msgs.msg import Point, Quaternion
from nautonomous_navigation_pathfinder.srv import *
from astarSearch import *
from crop import *
from orientation import orientation
from sensor_msgs.msg import NavSatFix

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

positionNorthing = 0
positionEasting = 0

debug = False # change this to see example application

G = nx.Graph()
v = []

def gpsFixCallback(data):
	global positionNorthing, positionEasting
	positionNorthing = data.northing
	positionEasting = data.easting

	print (positionNorthing, positionEasting)

# def findClosestWaypoints(waypoint):
# 	closestNode = None
# 	closestDistance = sys.maxint
    
# 	# Check each node and see which one is closest to the given waypoint.
# 	for node in G.nodes():
# 		distance = euclideanDistance(waypoint, node)
# 		if distance == 0.0:
# 			return waypoint
# 		elif(distance < closestDistance):
# 			closestDistance = distance
# 			closestNode = node

# 	# When no closest node can be found, then return false.
# 	if not closestNode:
# 		return False

# 	# Keep a array of the closest nodes.
# 	closestNodes = [closestNode]

# 	nextNode = None    
# 	nextDistance = sys.maxint

# 	# For each edge check if it is connected to the closest node
# 	for edge in G.edges():

# 		if(closestNode in edge):
# 			edgeNode = None
# 			if(closestNode == edge[0]):
# 				edgeNode = edge[1]
# 			elif(closestNode == edge[1]):
# 				edgeNode = edge[0]              

# 			distance = euclideanDistance(edgeNode, waypoint)
# 			if(distance < nextDistance):
# 				nextDistance = distance
# 				nextNode = edgeNode

# 	if(nextNode == 0):
# 		return False

# 	closestNodes.append(nextNode)

# 	return closestNodes

def get_closest_edge(goal_x, goal_y):
	
	# Variables to keep track of the closest edge and its distance.
	closestEdge = None
	closestDistance = float("inf")

	for edge in G.edges():
		# Get the easting and northing of the utm coordinate from the lat-lon position
		start_position = map(float, edge[0].split(","))
		end_position = map(float, edge[1].split(","))

		start_x = start_position[0]
		start_y = start_position[1]
		end_x = end_position[0]
		end_y = end_position[1]

		# Transposed coordinate of the path direction of the edge
		path_x = start_x - end_x
		path_y = start_y - end_y 

		# Transposed coordinate of the goal location of the boat
		boat_x = goal_x - end_x
		boat_y = goal_y - end_y

		# Calculate the quotient for the projection (y dot u) / (u dot u)
		top_quotient_projection = (boat_x * path_x + boat_y * path_y)
		bottom_quotient_projection = (path_x * path_x + path_y * path_y)

		# Calculate the quotient of the projection
		quotient_projection = (top_quotient_projection / bottom_quotient_projection)

		# Project u to get projected y
		projected_path_x = quotient_projection * path_x
		projected_path_y = quotient_projection * path_y

		# Calculate the total distance using the euclidean distance 
		distance_x = 0
		distance_y = 0
		# When the constraint of the projection is not onto the line, adjust the calculation.
		if top_quotient_projection <= 0:
			# the distance is from the current position of the boat to the origin of the frame.
			distance_x = boat_x
			distance_y = boat_y
		elif bottom_quotient_projection <= top_quotient_projection:
			# the distance is from the current position of the boat to the end of the path
			distance_x = boat_x - path_x
			distance_y = boat_y - path_y
		else:
			# Reduce y by project y to get z (distance)
			distance_x = boat_x - projected_path_x
			distance_y = boat_y - projected_path_y

		totalDistance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))

		if(totalDistance < closestDistance):
			closestEdge = edge
			closestDistance = totalDistance

	print "ClosestEdge: " + str(closestEdge)
	print "ClosestDistance: " + str(closestDistance)

	return closestEdge


def find_path(request):


	if debug:
		randomStart = map(float,random.choice(G.nodes()).split(","))
		positionEasting = randomStart[0]
		positionNorthing = randomStart[1]

	start = str(positionEasting) + "," + str(positionNorthing)

	boatEdge = get_closest_edge(positionEasting, positionNorthing)

	goal = str(request.goalEasting) + "," + str(request.goalNorthing)

	goalEdge = get_closest_edge(request.goalEasting, request.goalNorthing)

	print "Find path at " + start + " to " + goal

	searchResult = []
	shortestDistance = sys.maxint
	shortestResult = []

	#
	for i in range(0, 4):
		result,distance = aStarSearch(G, boatEdge[(i/2)], goalEdge[not (i%2)])
		if distance < shortestDistance:
			shortestDistance = distance
			shortestResult = result

			shortestResult.insert(0, start)
			shortestResult.insert(1, boatEdge[(i/2)])
			
			shortestResult.append(goalEdge[not (i%2)])
			shortestResult.append(goal)

	# #pin = cropResultingPath(shortestResult)

	route = [] # route containing utm coordinates

	for j in range(len(shortestResult)):
		pathCoordinate = shortestResult[j].split(',')

		coordinateGPSnorthing = float(pathCoordinate[0])
		coordinateGPSeasting = float(pathCoordinate[1])
		route.append(Point(coordinateGPSnorthing,coordinateGPSeasting, 0))

		if debug:
			print route[-1] # print last added gps coordinate
	
	#os.system("rosrun map_server map_server " + project_path + "/config/amsterdam.yaml&")
	#os.system("")

	#z, w = orientation(utm_route)
	#print "x:", utm_route[1][0] - 121000, "y:", utm_route[1][1] - 486500, "z:", z, "w:", w

	if debug:
		# add start and goal nodes
		G.add_node(shortestResult[0], pos=(route[0].x, route[0].y))
		G.add_node(shortestResult[-1], pos=(route[-1].x, route[-1].y))

		node_colors = []
		for n in G.nodes():
			if n in shortestResult:
				if n == start or n == goal:
					node_colors.append("red")
					print "found goal would color red"
				else:
					node_colors.append("green")
			else:
				node_colors.append("white")

		pos = nx.get_node_attributes(G,'pos')
		nx.draw_networkx_nodes(G, pos = pos, node_color = node_colors, node_size = 50)
		nx.draw_networkx_edges(G, pos = pos)
		plt.show()

	return FindPathAmsterdamCanalsResponse(route)

def path_finder_server():
	rospy.init_node('path_finder_server')
	sub = rospy.Subscriber("/utm/fix", NavSatFix, gpsFixCallback)
	rate = rospy.Rate(10)
	while positionEasting == 0 and not rospy.is_shutdown():
		rospy.loginfo("waiting fix")
		rate.sleep()

	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)
	print "Waiting for coordinates..."
	rospy.spin()

def test_path_finder():
	
	sub = rospy.Subscriber("/utm/fix/", NavSatFix, gpsFixCallback)
	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)

	try:
		service_client = rospy.ServiceProxy("find_path_amsterdam_canals", FindPathAmsterdamCanals)
		randomGoal = map(float,random.choice(G.nodes()).split(","))
		srv_response = service_client(randomGoal[0], randomGoal[1])
		print "Success " + str(srv_response.pathCoordinates)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e
	print "Finished testing"
	rospy.spin()

def convertGPSStringToUTM(gpsString):
	gpsPosition = map(float, gpsString.split(",", 1))
	utmPosition = utm.from_latlon(gpsPosition[0], gpsPosition[1])[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	utmString = str(utmPosition[0]) + "," + str(utmPosition[1])
	return utmString, utmPosition[0], utmPosition[1]

def convertGPSArrayToUTM(gpsArray):
	result = []
	for gpsString in gpsArray:
		result.append(convertGPSStringToUTM(gpsString)[0])
	return result

def construct_graph():
	global v
	print "Constructing Graph"
	with open(project_path + '/data/intersections.json') as data_file:    
		intersection_data = json.load(data_file)
		for node in intersection_data:
			node_id_string, node_id_easting, node_id_northing = convertGPSStringToUTM(node["id"])
			nodes_connected = convertGPSArrayToUTM(node["connected"])
			v.append(Vertex(node_id_string, node["adjacent"], nodes_connected))
			G.add_node(node_id_string, pos = (node_id_easting, node_id_northing))

		for node in v:
			for connected in node.connected:
				G.add_edge(node.id, connected, weight = (euclideanDistance(node.id,connected)))

		v = []
if __name__ == "__main__":
	rospy.init_node('path_finder_server')
	debug = rospy.get_param('~debug')
	print debug
	construct_graph()
	if debug:
		test_path_finder()
	else:
		path_finder_server()

