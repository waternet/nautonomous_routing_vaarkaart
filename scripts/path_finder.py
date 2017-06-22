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
from geometry_msgs.msg import Pose2D
from nautonomous_navigation_pathfinder.srv import *
from astarSearch import *
from orientation import orientation
from sensor_msgs.msg import NavSatFix

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

positionNorthing = 0
positionEasting = 0

debug = False # change this to see example application
test = False

G = nx.Graph()
v = []

def gpsFixCallback(data):
	global positionNorthing, positionEasting
	positionNorthing = data.northing
	positionEasting = data.easting

	print (positionNorthing, positionEasting)

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

	#print "ClosestEdge: " + str(closestEdge)
	#print "ClosestDistance: " + str(closestDistance)

	return closestEdge

def getTrafficId(edge):
	#print "edge: " + str(edge)
	l1 = 0
	l2 = 0
	for node in v:
		if node.id == edge[0]:
			l1 = node.adjacent
		elif node.id == edge[1]:
			l2 = node.adjacent
	#print str(l1) + " "  + str(l2)
	return list(set(l1).intersection(l2))[0]

def find_path(request):
	global positionEasting
	global positionNorthing

	if debug:
		randomStartEdge = random.choice(G.edges())
		firstRandomStartNode = map(float,randomStartEdge[0].split(","))
		secondRandomStartNode = map(float,randomStartEdge[1].split(","))

		positionEasting = firstRandomStartNode[0] #(firstRandomStartNode[0] + secondRandomStartNode[0]) / 2
		positionNorthing = firstRandomStartNode[1] #(firstRandomStartNode[1] + secondRandomStartNode[1]) / 2
	elif test:
		positionEasting = request.testStartEasting
		positionNorthing = request.testStartNorthing

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

			#print "RESULT: " + str(result)
			#print ""

			# if shortestResult[0] != start:
			# 	shortestResult.insert(0, start)
			# 	if shortestResult[0] != boatEdge[(i/2)]:
			# 		print "added boat edge1"
			# 		shortestResult.insert(1, boatEdge[(i/2)])
			# else:
			if shortestResult[0] != boatEdge[(i/2)]:
				#print "added boat edge2"
				shortestResult.insert(0, boatEdge[(i/2)])
			
			#if shortestResult[-1] != goalEdge[not (i%2)]:
			#	shortestResult.append(goalEdge[not (i%2)])
			#if shortestResult[-1] != goalEdge[not (i%2)]:
			#	print "added goal edge"
			#	shortestResult.append(goal)

	# #pin = cropResultingPath(shortestResult)

	routePose = [] # route containing utm coordinates
	routeIds = [getTrafficId(boatEdge)]

	for i, item in enumerate(shortestResult):
		pathCoordinate = map(float,item.split(','))
		if i != 0:
			routeIds.append(getTrafficId([shortestResult[i-1],item]))
		
		# todo give correct theta pose for next path segment
		routePose.append(Pose2D(pathCoordinate[0], pathCoordinate[1], 0))

	routeIds.append(getTrafficId(goalEdge))
	
	routePose.insert(0, Pose2D(positionEasting, positionNorthing, 0))
	routePose.append(Pose2D(request.goalEasting, request.goalNorthing, 0))

	#print "Start: " + str(start)
	#print "Route poses: " + str(routePose) # print last added gps coordinate
	#print "Route ids: " + str(routeIds)
	#print "End: " + str(goal)
	
	#os.system("rosrun map_server map_server " + project_path + "/config/amsterdam.yaml&")
	#os.system("")

	print "Route: " + str(routePose)

	#z, w = orientation(utm_route)
	#print "x:", utm_route[1][0] - 121000, "y:", utm_route[1][1] - 486500, "z:", z, "w:", w

	if debug:
		# add start and goal nodes
		#G.add_node(start, pos=(positionEasting, positionNorthing))
		#G.add_node(goal, pos=(request.goalEasting, request.goalNorthing))

		node_colors = []
		for n in G.nodes():
			if n == start:
				node_colors.append("blue")
				#print "found start"
			elif n == goal:
				node_colors.append("red")
				#print "found goal"
			else:
				found = False
				utmCoordinate = map(float, n.split(","))
				for p in routePose:
					#print str(p.x) + " " + str(utmCoordinate[0]) + " " + str(p.y) + " " + str(utmCoordinate[1])  
					if(p.x == utmCoordinate[0] and p.y == utmCoordinate[1]):
						node_colors.append("green")
						found = True
						break
				if not found:
					node_colors.append("white")	
				
			

		pos = nx.get_node_attributes(G,'pos')
		nx.draw_networkx_nodes(G, pos = pos, node_color = node_colors, node_size = 50)
		nx.draw_networkx_edges(G, pos = pos)
		plt.show()

	# rospy.wait_for_service('map_cropper')
    # try:
    #     cropMapPoints = rospy.ServiceProxy('map_cropper', CropMapPoints)
	# 	cropMapPointsResponse = cropMapPoints(routePose, routeIds)
    #     print cropMapPointsResponse.
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e

	return FindPathAmsterdamCanalsResponse(routePose, routeIds)

def path_finder_server():
	global positionEasting, positionNorthing
	rospy.init_node('path_finder_server')
	sub = rospy.Subscriber("/utm/fix", NavSatFix, gpsFixCallback)
	rate = rospy.Rate(10)
	if not test:
		while positionEasting == 0 and not rospy.is_shutdown():
			rospy.loginfo("waiting fix")
			rate.sleep()
	#else: 
		#randomStartEdge = random.choice(G.edges())
		#firstRandomStartNode = map(float,randomStartEdge[0].split(","))
		#secondRandomStartNode = map(float,randomStartEdge[1].split(","))

		#positionEasting = (firstRandomStartNode[0] + secondRandomStartNode[0]) / 2
		#positionNorthing = (firstRandomStartNode[1] + secondRandomStartNode[1]) / 2

	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)
	print "Waiting for coordinates..."
	rospy.spin()

def debug_path_finder():
	
	sub = rospy.Subscriber("/utm/fix/", NavSatFix, gpsFixCallback)
	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)

	service_client = rospy.ServiceProxy("find_path_amsterdam_canals", FindPathAmsterdamCanals)
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		try:
			randomEdge = random.choice(G.edges())
			firstRandomNode = map(float,randomEdge[0].split(","))
			secondRandomNode = map(float,randomEdge[1].split(","))

			random_easting = firstRandomNode[0] #(firstRandomNode[0] + secondRandomNode[0]) / 2
			random_northing = firstRandomNode[1] #(firstRandomNode[1] + secondRandomNode[1]) / 2

			srv_response = service_client(random_easting, random_northing,0,0)
			# srv_response = service_client(628532, 5805045)
			print "Success " + str(srv_response.pathLocations) + " " + str(srv_response.pathIds)
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
		rate.sleep()
		print "Finished testing"

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

if __name__ == "__main__":
	rospy.init_node('path_finder_server')
	debug = rospy.get_param('~debug', False)
	test = rospy.get_param('~test', False)
	print str(debug) + " " + str(test)
	construct_graph()
	if debug:
		debug_path_finder()
	else:
		path_finder_server()

