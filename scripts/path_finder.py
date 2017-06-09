#!/usr/bin/env python

import rospy
import pyproj
import json
import os
import math

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

latitude = 0
longitude = 0

debug = False # change this to see example application

if debug:
	latitude = 52.36727
	longitude = 4.93093

def gpsFixCallback(data):
	global latitude, longitude
	latitude = data.latitude
	longitude = data.longitude

	print (latitude, longitude)

def findClosestWaypoints(waypoint):
	closestNode = None
	closestDistance = sys.maxint
    
	# Check each node and see which one is closest to the given waypoint.
	for node in G.nodes():
		distance = euclideanDistance(waypoint, node)
		if distance == 0.0:
			return waypoint
		elif(distance < closestDistance):
			closestDistance = distance
			closestNode = node

	# When no closest node can be found, then return false.
	if not closestNode:
		return False

	# Keep a array of the closest nodes.
	closestNodes = [closestNode]

	nextNode = None    
	nextDistance = sys.maxint

	# For each edge check if it is connected to the closest node
	for edge in G.edges():

		if(closestNode in edge):
			edgeNode = None
			if(closestNode == edge[0]):
				edgeNode = edge[1]
			elif(closestNode == edge[1]):
				edgeNode = edge[0]              

			distance = euclideanDistance(edgeNode, waypoint)
			if(distance < nextDistance):
				nextDistance = distance
				nextNode = edgeNode

	if(nextNode == 0):
		return False

	closestNodes.append(nextNode)

	return closestNodes

def find_path(request):

	start = str(latitude) + "," + str(longitude)
	goal = str(request.goalLatitude) + "," + str(request.goalLongitude)

	print "Find path at " + start + " to " + goal

	closestStart = findClosestWaypoints(start)
	closestGoal = findClosestWaypoints(goal)

	searchResult = []

	shortestDistance = sys.maxint

	shortestResult = []
	#
	for i in range(0, 4):
		result,distance = aStarSearch(G, closestStart[(i/2)], closestGoal[not (i%2)])
		if distance < shortestDistance:
			shortestDistance = distance
			shortestResult = result
			shortestResult.insert(0, closestStart[(i/2)])
			shortestResult.insert(0, start)
			shortestResult.append(closestGoal[not (i%2)])
			shortestResult.append(goal)

	#pin = cropResultingPath(shortestResult)

	gps_route = [] # route containing gps coordinates
	utm_route = [] # route containing utm coordinates

	p = pyproj.Proj(proj='utm', zone=31, ellps='WGS84')

	for j in range(len(shortestResult)):
		pathCoordinate = shortestResult[j].split(',')

		coordinateGPSLatitude = float(pathCoordinate[0])
		coordinateGPSLongitude = float(pathCoordinate[1])
		gps_route.append(Point(coordinateGPSLatitude,coordinateGPSLongitude, 0))

		coordinateUTMLatitude, coordinateUTMLongitude = p(coordinateGPSLatitude, coordinateGPSLongitude)
		utm_route.append(Point(coordinateUTMLatitude, coordinateUTMLongitude, 0))

		if debug:
			print gps_route[-1] # print last added gps coordinate
			print utm_route[-1] # print last added utm coordinate
	
	#os.system("rosrun map_server map_server " + project_path + "/config/amsterdam.yaml&")
	#os.system("")

	#z, w = orientation(utm_route)
	#print "x:", utm_route[1][0] - 121000, "y:", utm_route[1][1] - 486500, "z:", z, "w:", w

	if debug:
		node_colors = ["green" if n in shortestResult else "white" for n in G.nodes()]
		pos = nx.get_node_attributes(G,'pos')
		nx.draw_networkx_nodes(G, pos = pos, node_color = node_colors, node_size = 50)
		nx.draw_networkx_edges(G, pos = pos)
		plt.show()

	return FindPathAmsterdamCanalsResponse(gps_route)

def path_finder_server():
	rospy.init_node('path_finder_server')
	sub = rospy.Subscriber("gps/fix", NavSatFix, gpsFixCallback)
	rate = rospy.Rate(10)
	while longitude == 0 and not rospy.is_shutdown():
		rospy.loginfo("waiting fix")
		rate.sleep()

	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)
	print "Waiting for coordinates..."
	rospy.spin()

def test_path_finder():
	rospy.init_node('path_finder_server')
	sub = rospy.Subscriber("gps/fix", NavSatFix, gpsFixCallback)
	s = rospy.Service('find_path_amsterdam_canals', FindPathAmsterdamCanals, find_path)

	try:
		service_client = rospy.ServiceProxy("find_path_amsterdam_canals", FindPathAmsterdamCanals)
		srv_response = service_client(52.36905, 4.89248)
		print "Success " + str(srv_response.pathCoordinates)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e
	print "Finished testing"
	rospy.spin()

if __name__ == "__main__":
	print "Constructing Graph"
	G = nx.Graph()
	v = []
	with open(project_path + '/data/intersections.json') as data_file:    
		data = json.load(data_file)
		for node in data:
			v.append(Vertex(node["id"], node["adjacent"], node["connected"]))
			position = node["id"].split(",")
			positionPoints = [float(position[1]), float(position[0])]                      
			G.add_node(node["id"], pos = positionPoints)

		for node in data:
			connectedNodes = node["connected"] 
			for connected in connectedNodes:
				G.add_edge(node["id"], connected, weight = (euclideanDistance(node["id"],connected)*1000))
	if debug:
		test_path_finder()
	else:
		path_finder_server()

