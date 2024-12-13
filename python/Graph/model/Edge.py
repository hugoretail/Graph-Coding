from IEdge import IEdge
from Node import Node
from math import sqrt

class Edge(IEdge):
    def __init__(self, node1 : Node, node2 : Node):
        self.node1 = node1
        self.node2 = node2
        self.weight = sqrt((node2.x - node1.x)**2 + (node2.y - node1.y)**2)




