from .IEdge import IEdge
from .Node import Node
from math import sqrt

class Edge(IEdge):
    def __init__(self, node1 : Node, node2 : Node):
        self.node1 = node1
        self.node2 = node2
        self.weight = sqrt((node2.x - node1.x)**2 + (node2.y - node1.y)**2)

    def get_opposite(self, node : Node):
        if node == self.node1:
            return self.node2
        else:
            return self.node1

    def __str__(self):
        return f"Edge({self.node1} -> {self.node2}, weight={self.weight:.2f})"


