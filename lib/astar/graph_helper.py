import math

# Calculate the euclidean distance between the start and destination of the 
def euclidean_distance(start, destination):
	start_node = map(float, start.split(","))
	destination_node = map(float, destination.split(","))

	distance = math.sqrt(math.pow(start_node[0] - destination_node[0], 2) + math.pow(start_node[1] - destination_node[1], 2))

	return distance

# Get the closest edge using point to line projection.
def get_closest_edge(vaarkaart_graph, goal_x, goal_y):

	# Variables to keep track of the closest edge and its distance.
	closest_edge = None
	closest_distance = float("inf")

	for current_edge in vaarkaart_graph.edges():
		# Get the easting and northing of the utm coordinate from the lat-lon position
		first_node = map(float, current_edge[0].split(","))
		second_node = map(float, current_edge[1].split(","))

		# Rename variables
		start_x = first_node[0]
		start_y = first_node[1]
		destination_x = second_node[0]
		destination_y = second_node[1]

		# Transposed coordinate of the route direction of the edge
		route_x = start_x - destination_x
		route_y = start_y - destination_y 

		# Transposed coordinate of the goal location of the boat
		boat_x = goal_x - destination_x
		boat_y = goal_y - destination_y

		# Calculate the quotient for the projection (y dot u) / (u dot u)
		top_quotient_projection = (boat_x * route_x + boat_y * route_y)
		bottom_quotient_projection = (route_x * route_x + route_y * route_y)

		# Calculate the quotient of the projection
		quotient_projection = (top_quotient_projection / bottom_quotient_projection)

		# Project u to get projected y
		projected_route_x = quotient_projection * route_x
		projected_route_y = quotient_projection * route_y

		# Calculate the total distance using the euclidean distance 
		distance_x = 0
		distance_y = 0

		# When the constraint of the projection is not onto the line, adjust the calculation.
		if top_quotient_projection <= 0:
			# the distance is from the current position of the boat to the origin of the frame.
			distance_x = boat_x
			distance_y = boat_y

		elif bottom_quotient_projection <= top_quotient_projection:
			# the distance is from the current position of the boat to the end of the route
			distance_x = boat_x - route_x
			distance_y = boat_y - route_y

		else:
			# Reduce y by project y to get z (distance)
			distance_x = boat_x - projected_route_x
			distance_y = boat_y - projected_route_y

		total_distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))

		# Set the closest edge and distance if the total distance is smaller than the current closest distance.
		if(total_distance < closest_distance):
			closest_edge = current_edge
			closest_distance = total_distance

	return closest_edge