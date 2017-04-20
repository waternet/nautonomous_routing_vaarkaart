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

longitude = 0
latitude = 0

def gpsFixCallback(data):
    global longitude, latitude
    longitude = data.longitude
    latitude = data.latitude

def findClosestWaypoints(waypoint):
	closestNode = 0
	closestDistance = sys.maxint
    
	for node in G.nodes():
		distance = euclideanDistance(waypoint, node)
		if distance == 0.0:
			return waypoint
		elif(distance < closestDistance):
			closestDistance = distance
			closestNode = node

	if(closestNode == 0):
		return False

	closestNodes = [closestNode]

	nextNode = 0    
	nextDistance = sys.maxint

	for edge in G.edges():

		if(closestNode in edge):
			edgeNode = 0
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

def handle_navigation(request):
    start = str(latitude) + "," + str(longitude)
    goal = str(request.c) + "," + str(request.d)

    closestStart = findClosestWaypoints(start)
    closestGoal = findClosestWaypoints(goal)

    searchResult = []

    shortestDistance = sys.maxint
    shortestResult = []
    for i in range(0, 4):
        result,distance = aStarSearch(G, closestStart[(i/2)], closestGoal[not (i%2)])
        if distance < shortestDistance:
            shortestDistance = distance
            shortestResult = result
            shortestResult.insert(0, closestStart[(i/2)])
            shortestResult.insert(0, start)
            shortestResult.append(closestGoal[not (i%2)])
            shortestResult.append(goal)

    pin = cropResultingPath(shortestResult)

    route = []
    for j in range(0, 4):
        short = shortestResult[j]
        dots = short.split(',')
        route.append([float(dots[0]), float(dots[1])])

    p = pyproj.Proj(proj='utm', zone=31, ellps='WGS84')

    rd_route = []
    for coord in route:
        x,y = p(coord[1], coord[0])
        rd_route.append([x, y])

    os.system("rosrun map_server map_server " + project_path + "/config/amsterdam.yaml&")
    os.system("")

    z, w = orientation(rd_route)
	#print "x:", rd_route[1][0] - 121000, "y:", rd_route[1][1] - 486500, "z:", z, "w:", w

	#return pin[0][0], pin[0][1], z, w

	#node_colors = ["green" if n in shortestResult else "white" for n in G.nodes()]

	#pos=nx.get_node_attributes(G,'pos')
	#nx.draw_networkx_nodes(G, pos=pos, node_color=node_colors)
	#nx.draw_networkx_edges(G, pos=pos)
	#plt.show()
    #print "Dit is een test..."
    #print rd_route[1][0]
    #return rd_route[1][0] - 628550, rd_route[1][1] - 5803351, z, w

def navigation_server():
	rospy.init_node('navigation_server')
	sub = rospy.Subscriber("gps/fix", NavSatFix, gpsFixCallback)
	rate = rospy.Rate(10)
	while longitude == 0 and not rospy.is_shutdown():
		rospy.loginfo("waiting fix")
		rate.sleep()

	s = rospy.Service('add_two_ints', AddTwoInts, handle_navigation)
	print "Waiting for coordinates..."
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
			positionPoints = [float(position[1]),float(position[0])]                      
			G.add_node(node["id"],pos=positionPoints)

		for node in data:
			connectedNodes = node["connected"] 
			for connected in connectedNodes:
				G.add_edge(node["id"],connected, weight=(euclideanDistance(node["id"],connected)*1000))
	
	navigation_server()
