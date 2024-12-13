from IGraph import IGraph

class Graph(IGraph):
    def __init__(self, file):
        nodes, edges = load_graph_from_file(file)
        self.nodes = nodes
        self.edges = edges

    def get_node_from_position(self, x, y):
        pass

    def get_edge_from_positions(self, x1, y1, x2, y2):
        pass

def load_graph_from_file(file):
    nodes = []
    edges = []

    with open(file, "r") as f:
        content = f.read()
        print(content)

    return nodes, edges
