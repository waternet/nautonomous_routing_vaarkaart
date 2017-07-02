
# Vaarkaart Vertex to store the node, adjacent nodes and the connected nodes.
class VaarkaartVertex:
    def __init__(self, node, connected = None):
        self.id = node
            
        if(connected != None):
            self.connected = connected
        else:
            self.connected = []