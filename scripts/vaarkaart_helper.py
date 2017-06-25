
import json
import networkx as nx
import os
import pyproj
import urllib

import graph_helper
import utm_helper
import vaarkaart_vertex

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

vaarkaart_traffic_url = "https://grachten.waternet.nl/api/v2/trafficdata"
vaarkaart_intersection_url = project_path + '/data/intersections.json'

vaarkaart_graph = None
vaarkaart_vertices = []

# Get the traffic id from the edge by comparing the traffic ids of both ends of the edge nodes.
def get_vaarkaart_node_id(first_node, second_node):
	for node in vaarkaart_vertices:
		if node.id == first_node:
			l1 = node.adjacent
		elif node.id == second_node:
			l2 = node.adjacent
	return list(set(l1).intersection(l2))[0]

# Calculate the cost function for astar searching
def vaarkaart_cost_function(first, second):

    traffic_id = get_vaarkaart_node_id(first, second)

    for item in traffic["data"]:
        if(item["trafficlink_id"] == traffic_id[0]):
            return graph_helper.euclidean_distance(first, second) * (item["traffic"] / 4 + 1)

    return 0

# Load the vaarkaart intersections data file.
def load_vaarkaart_intersections():
    global vaarkaart_intersection_url

    with open(vaarkaart_intersection_url) as vaarkaart_intersections_data:    
       return json.load(vaarkaart_intersections_data)

    return None

# Load the vaarkaart traffic url.
def load_vaarkaart_traffic():
    global vaarkaart_traffic_url

    vaarkaart_traffic_response = urllib.urlopen(vaarkaart_traffic_url)

    return json.loads(vaarkaart_traffic_response.read())

# Load the vaarkaart graph
def load_vaarkaart_graph(vaarkaart_intersections, vaarkaart_traffic):
    global vaarkaart_vertices

    vaarkaart_graph = nx.Graph()

    # For each intersection create a vaarkaart vertices and vaarkaart graph based on utm coordinates.
    for current_intersection in vaarkaart_intersections:
        # Get the UTM id, easting and northing from the gps coordinate in the current intersection id.
        intersection_id, intersection_easting, intersection_northing = utm_helper.convert_GPS_string_to_UTM(current_intersection["id"])
        
        # Get the connected intersections of the current intersection. 
        connected_intersections = utm_helper.convert_GPS_array_to_UTM(current_intersection["connected"])

        # Add the intersection as a vaarkaart vertice and as a graph node.
        vaarkaart_vertices.append(vaarkaart_vertex.VaarkaartVertex(intersection_id, connected_intersections))
        vaarkaart_graph.add_node(intersection_id, pos = (intersection_easting, intersection_northing))

    # Connect the vaartkaart graph nodes by edges based on the vaarkaart vertice connected vertices.
    for current_vertice in vaarkaart_vertices:
        for connected_vertice in current_vertice.connected:
            vaarkaart_graph.add_edge(current_vertice.id, connected_vertice, weight = graph_helper.euclidean_distance(current_vertice.id, connected_vertice))

    return vaarkaart_graph

# Load the vaarkaart and return the graph
def load_vaarkaart():
    vaarkaart_intersections = load_vaarkaart_intersections()
    vaarkaart_traffic = load_vaarkaart_traffic()

    vaarkaart_graph = load_vaarkaart_graph(vaarkaart_intersections, vaarkaart_traffic)

    return vaarkaart_graph