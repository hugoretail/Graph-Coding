from .IGraph import IGraph
from .Node import Node
from .Edge import Edge
from typing import List
from queue import Queue
from collections import deque

class Graph(IGraph):
    def __init__(self):
        self.view = None
        self.path = ""
        self.nodes = []
        self.edges = []
        self.selected_nodes = []
        self.selected_nodes_counter = 0

    def get_neighbors(self, node : Node):
        neighbors = []
        for e in node.edges:
            opposite_node = e.get_opposite(node)
            neighbors.append(opposite_node)
        return neighbors

    def reset_selected_nodes(self):
        self.selected_nodes = []
        self.selected_nodes_counter = 0

    def reset_algorithms_button(self):
        self.view.disable_algorithms_menu_button()

    def select_node(self, node : Node):
        for n, count in self.selected_nodes:
            if n == node:
                self.deselect_node(node)
                return

        if len(self.selected_nodes) >= 2:
            oldest_node, _ = min(self.selected_nodes, key=lambda x:x[1])
            self.selected_nodes = [(n,c - 1) for n, c in self.selected_nodes if n != oldest_node]
            oldest_node.selected = False

        self.selected_nodes.append((node, len(self.selected_nodes) + 1))
        self.selected_nodes_counter = len(self.selected_nodes)
        node.selected = True

    def deselect_node(self, node : Node):
        self.selected_nodes = [(n, count - 1) for n, count in self.selected_nodes if n != node]
        self.selected_nodes_counter = len(self.selected_nodes)
        node.selected = False
        self.selected_nodes = [(n, i + 1) for i, (n, _) in enumerate(self.selected_nodes)]

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

    def get_edge_from_nodes(self, node1, node2) -> Edge:
        for e in self.edges:
            if e.node1 == node1 and e.node2 == node2:
                return e
            elif e.node1 == node2 and e.node2 == node1:
                return e

    def get_edge_from_positions(self, x1, y1, x2, y2) -> Edge:
        for e in self.edges:
            if e.node1.x == x1 and e.node1.y == y1 and e.node2.x == x2 and e.node2.y == y2:
                return e

    def load_graph(self, path):
        self.path = path
        self.load_nodes_from_file(path)
        self.load_edges_from_file(path)
        self.reset_selected_nodes()
        self.update_view()

    def reload_graph(self):
        self.load_nodes_from_file(self.path)
        self.load_edges_from_file(self.path)
        self.reset_selected_nodes()
        self.update_view()

    def update_view(self):
        self.view.update_graph(self.nodes, self.edges)

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

    def bfs(self):
        start = self.selected_nodes[0][0]
        nodes, edges = [], []

        f = Queue()
        f.put(start)
        start.mark = True
        while not f.empty():
            s = f.get()
            # add node here
            if s not in nodes:
                nodes.append(s)

            for t in self.get_neighbors(s):
                if t.mark is False:
                    f.put(t)
                    t.mark = True
                    # add edge here
                    edge = self.get_edge_from_nodes(s, t)
                    if edge not in edges:
                        edges.append(edge)

        self.set_nodes(nodes)
        self.set_edges(edges)
        self.view.update_graph(nodes,edges)
        self.reset_selected_nodes()
        self.reset_algorithms_button()
        self.view.enable_reset_button()

    def dfs(self):
        start = self.selected_nodes[0][0]
        nodes, edges = [], []

        p = deque()
        p.append(start)
        while len(p) > 0:
            s = p.pop()
            if s.mark is False:
                s.mark = True
                # add node here
                if s not in nodes:
                    nodes.append(s)

                for t in self.get_neighbors(s):
                    if t.mark is False:
                        p.append(t)
                        # add edge here
                        edge = self.get_edge_from_nodes(s, t)
                        if edge not in edges:
                            edges.append(edge)

        self.set_nodes(nodes)
        self.set_edges(edges)
        self.view.update_graph(nodes,edges)
        self.reset_selected_nodes()
        self.reset_algorithms_button()
        self.view.enable_reset_button()

    def ucs(self):
        pass

    def greedy_best_first(self):
        pass

    def a_star(self):
        pass

    def dijkstra(self):
        pass

    def bellman_ford(self):
        pass

    def floyd_warshall(self):
        pass

    def prim(self):
        pass

    def kruskal(self):
        pass

    def __str__(self):
        nodes_str = "\n  ".join(str(node) for node in self.nodes)
        edges_str = "\n  ".join(str(edge) for edge in self.edges)
        return f"Graph:\nNodes:\n  {nodes_str}\nEdges:\n  {edges_str}"