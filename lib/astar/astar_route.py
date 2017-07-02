# -*- coding: utf-8 -*-
import astar_node
import graph_helper
import priority_queue

# Check if the node is the goal state
def is_goal_state(current_node, goal_node):
    return current_node.state == goal_node

# Construct the route based on the node parent chain.
def construct_route(node):
    actionList = []
    total_route_cost = node.route_cost

    # While there is a node left.
    while(node is not None):
        
        actionList.insert(0, node.state)
        node = node.parent
    
    return actionList, total_route_cost

# Check if the node is in the explored node list.
def is_explored_node(current_node, explored_nodes):
    found_explored = False

    for explored_node in explored_nodes:
        if explored_node.state == current_node.state:
            found_explored = True

    return found_explored

# Check if the node is in the unexplored node list, and get the route cost if it is in the unexplored node list.
def is_unexplored_node(current_node, goal_node, unexplored_nodes):
    is_existing_unexplored_node = False
    current_unexplored_route_cost = 0

    for (_, _, unexplored_node) in unexplored_nodes.heap:
        if unexplored_node.state == current_node.state:
            is_existing_unexplored_node = True
            current_unexplored_route_cost = ((unexplored_node.route_cost + graph_helper.euclidean_distance(unexplored_node.state, goal_node))) #  + amount(open_list_item[2].state, open_list_item[2].parent.state)
            break

    return is_existing_unexplored_node, current_unexplored_route_cost

# Find the route using the vaarkaart graph, 
def astar_search(vaarkaart_graph, start_node, goal_node):

    current_node = astar_node.AStarNode(start_node, None, 0)

    # Check if the start is the goal state
    if is_goal_state(current_node, goal_node):
        return construct_route(current_node)
    		
    # Priority queue initialised with node and empty explored list
    unexplored_nodes = priority_queue.PriorityQueue() #priority doesn't matter, right?
    unexplored_nodes.push(current_node, 0)

    explored_nodes = []
    
    # Perform AStar search until the unexplored nodes is empty.
    while not unexplored_nodes.isEmpty():
        
        # Pop the current_node of the unexplored node list.
        current_node = unexplored_nodes.pop()
        explored_nodes.append(current_node)

        # Check if this current node is the goal node
        if is_goal_state(current_node, goal_node):
            return construct_route(current_node)            
       
        # Get the neighbours of the current_node
        for current_neighbour in vaarkaart_graph.neighbors(current_node.state):
            #value = graph_helper.euclidean_distance(node.state, current_neighbour)*1000
            # removed traffic cost temp. TODO : amount(current_neighbour, node.state)
            
            current_neighbour = astar_node.AStarNode(current_neighbour, current_node, ((current_node.route_cost + graph_helper.euclidean_distance(current_node.state, current_neighbour)))) #initialise node with actual distance, successor pathcost + entire path cost

            # Check if the neighbour is in the explored explored nodes
            if is_explored_node(current_neighbour, explored_nodes):
                continue

            # Check if the neighbour already exists in the explored nodes list.
            is_unexplored_neighbour, current_unexplored_route_cost = is_unexplored_node(current_neighbour, goal_node, unexplored_nodes)

            #  + amount(child.state, child.parent.state)
            distance = (current_neighbour.route_cost + graph_helper.euclidean_distance(current_neighbour.state, goal_node))

            # Check if it is not an unexplored node
            if not is_unexplored_neighbour:
                unexplored_nodes.push(current_neighbour, distance)

            elif current_neighbour.route_cost < current_unexplored_route_cost:
                # Replace in frontier heap only if the 
                for unexplored_node in unexplored_nodes.heap:
                    # Only check if the state of the unexplored node is equal to the current neighbour state.
                    if(unexplored_node[2].state == current_neighbour.state):
                        unexplored_nodes.heap.remove(unexplored_node)
                        unexplored_nodes.push(current_neighbour, distance)
                        break

    return False
