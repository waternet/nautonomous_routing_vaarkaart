
# AStar node
class AStarNode:
    def __init__(self, state_value, parent_value, cost_value):
        self.state = state_value
        self.parent = parent_value
        self.route_cost = cost_value