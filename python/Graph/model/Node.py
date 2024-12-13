from INode import INode

class Node(INode):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []

    def get_edges(self):
        return self.edges

    def get_node(self):
        return self

    def get_degree(self):
        return len(self.edges)

    def add_edge(self, edge):
        # TODO: verify that the current edge we are adding is already inside self.edges
        self.edges.append(edge)