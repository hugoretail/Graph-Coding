from .INode import INode
from math import sqrt

class Node(INode):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.mark = False # used for algorithms
        self.selected = False
        self.edges = []

    def get_degree(self):
        return len(self.edges)

    def add_edge(self, edge):
        if edge not in self.edges:
            self.edges.append(edge)
            return True
        else:
            return False

    def heuristic(self, target):
        return sqrt((target.x - self.x)**2 + (target.y - self.y)**2)

    def __str__(self):
        degree = self.get_degree()
        return f"Node(x={self.x}, y={self.y}, degree={degree})"