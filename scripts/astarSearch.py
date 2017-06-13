# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 21:19:10 2015

@author: dzeeuwe
"""

import math
import heapq
import json
import urllib
import os

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))
 
with open(project_path + '/data/intersections.json') as data_file:    
    intersection_data = json.load(data_file)
#with open('/home/wiebe/ros_workspace/src/WaternetNautonomous/WaternetNautonomousNavigation/navigator/scripts/traffic.json') as data_file:    
#    traffic = json.load(data_file)

url = "https://grachten.waternet.nl/api/v2/trafficdata"
response = urllib.urlopen(url)
traffic = json.loads(response.read())

#in first en second adressen van twee gekoppelde nodes. 
# out traffic waarde.
def amount(first, second):
    
    for item in data:
        if item["id"] == first:
            l1 = item["adjacent"]
        elif item["id"] == second:
            l2 = item["adjacent"]
    traffic_id = list(set(l1)&set(l2))

    for item in traffic["data"]:
        if(item["trafficlink_id"] == traffic_id[0]):
            return euclideanDistance(first, second) * (item["traffic"] / 4 + 1)
    return 0;

def euclideanDistance(start, end):
    startString = start.split(",")
    endString = end.split(",")
    return math.sqrt(math.pow((float(startString[0])-float(endString[0])), 2)+math.pow((float(startString[1])-float(endString[1])),2))
    
def isGoalState(node, goal):
    if(node.state == goal):
        return True
    return False

class Node:
    def __init__(self, stateValue, parentValue, costValue):
        self.state = stateValue
        self.parent = parentValue
        self.pathCost = costValue

def SOLUTION(node):
    actionList = []
    cost = node.pathCost
    while(node.parent is not None):
        
        actionList.insert(0, node.state)
        node = node.parent
    
    return actionList, cost

class PriorityQueue:
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        # FIXME: restored old behaviour to check against old results better
        # FIXED: restored to stable behaviour
        entry = (priority, self.count, item)
        # entry = (priority, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        #  (_, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0
#Astar
def aStarSearch(G, problem, goal):
    """Search the node that has the lowest combined cost and heuristic first."""
    node = Node(problem, None, 0)

    #Check if the problem is the goal state
    goalTest = isGoalState(node, goal)
    if(goalTest):
        return SOLUTION(node)
    		
    #Priority queue initialised with node and empty explored list
    frontier = PriorityQueue() #priority doesn't matter, right?
    frontier.push(node, 0);
    explored = []
     
    #Loop do
    loop = False
    while(True):
        if frontier.isEmpty():
            return False
            
        node = frontier.pop()

        goalTest = isGoalState(node, goal)
        if(goalTest):
            return SOLUTION(node)     
        
        explored.append(node)
    
       
        #raw_input("Press Enter to continue...")
        for option in G.neighbors(node.state):
            #value = euclideanDistance(node.state, option)*1000
            # removed traffic cost temp. TODO : amount(option, node.state)
            child = Node(option, node, ((node.pathCost + euclideanDistance(option, goal)))) #initialise node with actual distance, successor pathcost + entire path cost

            foundExplored = False
            for closed_list_item in explored:
                if(closed_list_item.state == child.state):
                    foundExplored = True

            if(foundExplored):
                continue;

            foundFrontier = False
            frontierPathCost = 0
            for open_list_item in frontier.heap:

                if(open_list_item[2].state == child.state):
                    foundFrontier = True
                    #  + amount(open_list_item[2].state, open_list_item[2].parent.state)
                    frontierPathCost = ((open_list_item[2].pathCost + euclideanDistance(open_list_item[2].state, goal)))
                    break

            #  + amount(child.state, child.parent.state)
            distance = ((child.pathCost + euclideanDistance(child.state, goal)))

            if(not foundFrontier):
                frontier.push(child, distance)
                
            elif (foundFrontier and child.pathCost < frontierPathCost):
                #replace in frontier heap
                for open_list_item in frontier.heap:
                    if(open_list_item[2].state == child.state):
                        frontier.heap.remove(open_list_item)
                        frontier.push(child, distance)
                        break
