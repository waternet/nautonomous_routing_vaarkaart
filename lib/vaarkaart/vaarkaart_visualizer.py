
import matplotlib.pyplot as plt

import networkx as nx

# Visualize vaarkaart based on the graph
def visualize_vaarkaart_route(vaarkaart_graph, start_coordinate, destination_coordinate, route_poses):
    node_colors = []
		
    for n in vaarkaart_graph.nodes():
        if n == start_coordinate:
            node_colors.append("blue")
        elif n == destination_coordinate:
            node_colors.append("red")
        else:
            found = False
            utm_coordinate = map(float, n.split(","))

            for p in route_poses:
                if(p.x == utm_coordinate[0] and p.y == utm_coordinate[1]):
                    found = True
                    break

            if not found:
                node_colors.append("white")	
            else: 
                node_colors.append("green")

    pos = nx.get_node_attributes(vaarkaart_graph, 'pos')

    nx.draw_networkx_nodes(vaarkaart_graph, pos = pos, node_color = node_colors, node_size = 50)
    nx.draw_networkx_edges(vaarkaart_graph, pos = pos)

    plt.show()