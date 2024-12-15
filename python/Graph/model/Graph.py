from .IGraph import IGraph
from .Node import Node
from .Edge import Edge
from typing import List

class Graph(IGraph):
    def __init__(self):
        self.nodes = []
        self.edges = []

    def set_nodes(self, nodes: List[Node]) -> None:
        self.nodes = nodes

    def set_edges(self, edges: List[Edge]) -> None:
        self.edges = edges
        for e in edges:
            e.node1.add_edge(e)
            e.node2.add_edge(e)

    def get_node_from_position(self, x, y) -> Node:
        for n in self.nodes:
            if n.x == x and n.y == y :
                return n


    def get_edge_from_positions(self, x1, y1, x2, y2) -> Edge:
        for e in self.edges:
            if e.node1.x == x1 and e.node1.y == y1 and e.node2.x == x2 and e.node2.y == y2:
                return e

    def load_nodes_from_file(self, file):
        nodes = []

        with open(file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                elements = line.split(",")
                if elements[0] == "N":
                    try:
                        x = int(elements[1])
                        y = int(elements[2])
                        node = Node(x, y)
                        nodes.append(node)
                    except (ValueError, IndexError):
                        print(f"Error while loading the node : {line}")

        self.set_nodes(nodes)

    def load_edges_from_file(self, file):
        edges_list = []

        with open(file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                elements = line.split(",")
                if elements[0] == "E":
                    try:
                        x1 = int(elements[1])
                        y1 = int(elements[2])
                        x2 = int(elements[3])
                        y2 = int(elements[4])
                        node1 = self.get_node_from_position(x1, y1)
                        node2 = self.get_node_from_position(x2, y2)
                        edge = Edge(node1, node2)
                        edges_list.append(edge)
                    except (ValueError, IndexError):
                        print(f"Error while loading the edge : {line}")

        self.set_edges(edges_list)

    def __str__(self):
        nodes_str = "\n  ".join(str(node) for node in self.nodes)
        edges_str = "\n  ".join(str(edge) for edge in self.edges)
        return f"Graph:\nNodes:\n  {nodes_str}\nEdges:\n  {edges_str}"