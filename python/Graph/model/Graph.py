import heapq

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

    def apply_algorithm_result(self, nodes, edges):
        self.set_nodes(nodes)
        self.set_edges(edges)
        self.view.update_graph(nodes, edges)
        self.reset_selected_nodes()
        self.reset_algorithms_button()
        self.view.enable_reset_button()

    def get_start_end_nodes(self):
        start = None
        end = None
        for n, p in self.selected_nodes:
            if p == 1:
                start = n
            elif p == 2:
                end = n
        return start, end

    def get_start_node(self):
        for n, p in self.selected_nodes:
            if p == 1:
                return n

    def get_end_node(self):
        for n, p in self.selected_nodes:
            if p == 2:
                return n

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

        self.apply_algorithm_result(nodes, edges)


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

        self.apply_algorithm_result(nodes, edges)

    def ucs(self):
        start, end = self.get_start_end_nodes()

        priority_queue = [(0, start)]
        visited = {start: (0, None)}
        reached = False

        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)
            if current_node == end:
                reached = True
                break

            for edge in current_node.edges:
                cost = edge.weight
                neighbor = edge.get_opposite(current_node)
                total_cost = current_cost + cost
                if neighbor not in visited or total_cost < visited[neighbor][0]:
                    visited[neighbor] = (total_cost, current_node)
                    heapq.heappush(priority_queue, (total_cost, neighbor))

        if reached:
            nodes_taken = []
            edges_taken = []
            current = end
            nodes_taken.append(current)
            current = visited[current][1]
            i = 0
            while current is not None:
                nodes_taken.append(current)
                edges_taken.append(self.get_edge_from_nodes(current, nodes_taken[i]))
                current = visited[current][1]
                i += 1
            nodes_taken.reverse()
            self.apply_algorithm_result(nodes_taken, edges_taken)


    def greedy_best_first(self):
        """NOT WORKING"""
        start, end = self.get_start_end_nodes()

        visited = set()
        priority_queue = []
        heapq.heappush(priority_queue, (start.heuristic(end), start))

        nodes_taken = []
        edges_taken = []

        while priority_queue:
            _, current_node = heapq.heappop(priority_queue)
            if current_node == end:
                nodes_taken.append(current_node)
                break

            if current_node not in visited:
                visited.add(current_node)
                nodes_taken.append(current_node)
                for neighbor in self.get_neighbors(current_node):
                    if neighbor not in visited and neighbor != end:
                        heapq.heappush(priority_queue,
                                       (neighbor.heuristic(end), neighbor))
                        edge = self.get_edge_from_nodes(current_node, neighbor)
                        if edge not in edges_taken and edge:
                            edges_taken.append(edge)

        self.apply_algorithm_result(nodes_taken, edges_taken)

    def a_star(self):
        start, end = self.get_start_end_nodes()

        if not start or not end:
            print("Start or end node is not selected.")
            return

        open_set = [(start.heuristic(end), start)]
        g_score = {node: float('inf') for node in self.nodes}
        g_score[start] = 0
        came_from = {}

        while open_set:
            f_score, current = heapq.heappop(open_set)

            if current == end:
                nodes_taken = []
                edges_taken = []
                while current in came_from:
                    nodes_taken.append(current)
                    previous = came_from[current]
                    edges_taken.append(self.get_edge_from_nodes(previous, current))
                    current = previous

                nodes_taken.append(start)
                nodes_taken.reverse()
                self.apply_algorithm_result(nodes_taken, edges_taken)
                return

            for edge in current.edges:
                neighbor = edge.get_opposite(current)
                tentative_g_score = g_score[current] + edge.weight

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + neighbor.heuristic(end)
                    heapq.heappush(open_set, (f_score, neighbor))

        print("No path found")

    def dijkstra(self):
        start = self.get_start_node()
        if not start:
            print("Start node is not selected.")
            return

        dist = {node: float('inf') for node in self.nodes}
        prev = {node: None for node in self.nodes}
        dist[start] = 0
        priority_queue = [(0, start)]

        nodes_taken = []
        edges_taken = []

        while priority_queue:
            current_dist, current_node = heapq.heappop(priority_queue)

            if current_node.mark:
                continue
            current_node.mark = True
            nodes_taken.append(current_node)

            for edge in current_node.edges:
                neighbor = edge.get_opposite(current_node)
                new_dist = current_dist + edge.weight

                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = current_node
                    heapq.heappush(priority_queue, (new_dist, neighbor))

        # Reconstruct the path
        end = self.get_end_node()
        if end:
            current = end
            while prev[current] is not None:
                edges_taken.append(self.get_edge_from_nodes(prev[current], current))
                current = prev[current]

        self.apply_algorithm_result(nodes_taken, edges_taken)

    def bellman_ford(self):
        start = self.get_start_node()
        if not start:
            print("Start node is not selected.")
            return

        dist = {node: float('inf') for node in self.nodes}
        prev = {node: None for node in self.nodes}
        dist[start] = 0

        for _ in range(len(self.nodes) - 1):
            for edge in self.edges:
                u = edge.node1
                v = edge.node2
                weight = edge.weight

                if dist[u] != float('inf') and dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight
                    prev[v] = u

                if dist[v] != float('inf') and dist[v] + weight < dist[u]:
                    dist[u] = dist[v] + weight
                    prev[u] = v

        for edge in self.edges:
            u = edge.node1
            v = edge.node2
            weight = edge.weight

            if dist[u] != float('inf') and dist[u] + weight < dist[v]:
                print("Graph contains a negative-weight cycle.")
                return

        nodes_taken = []
        edges_taken = []
        end = self.get_end_node()

        if end:
            current = end
            while current:
                nodes_taken.append(current)
                if prev[current]:
                    edges_taken.append(self.get_edge_from_nodes(prev[current], current))
                current = prev[current]

            nodes_taken.reverse()

        self.apply_algorithm_result(nodes_taken, edges_taken)

    def floyd_warshall(self):
        if not self.nodes:
            print("Le graphe ne contient aucun nœud.")
            return

        n = len(self.nodes)
        dist = [[float('inf')] * n for _ in range(n)]
        next_node = [[None] * n for _ in range(n)]

        for i, node in enumerate(self.nodes):
            dist[i][i] = 0
            for edge in node.edges:
                neighbor = edge.get_opposite(node)
                j = self.nodes.index(neighbor)
                dist[i][j] = edge.weight
                next_node[i][j] = j

        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if dist[i][k] + dist[k][j] < dist[i][j]:
                        dist[i][j] = dist[i][k] + dist[k][j]
                        next_node[i][j] = next_node[i][k]

        nodes_taken = set()
        edges_taken = set()

        for i in range(n):
            for j in range(n):
                if i != j and dist[i][j] != float('inf'):
                    current = i
                    while current != j:
                        node1 = self.nodes[current]
                        node2 = self.nodes[next_node[current][j]]
                        edge = self.get_edge_from_nodes(node1, node2)
                        nodes_taken.add(node1)
                        nodes_taken.add(node2)
                        if edge:
                            edges_taken.add(edge)
                        current = next_node[current][j]

        self.apply_algorithm_result(list(nodes_taken), list(edges_taken))

    def prim(self):
        if not self.nodes:
            print("The graph has no nodes.")
            return

        start = self.get_start_node()
        if not start:
            print("Start node is not selected.")
            return

        priority_queue = []
        visited = set()
        node_costs = {node: float('inf') for node in self.nodes}
        prev_edges = {node: None for node in self.nodes}
        node_costs[start] = 0
        heapq.heappush(priority_queue, (0, start))

        edges_taken = []
        nodes_taken = []

        while priority_queue:
            _, current = heapq.heappop(priority_queue)
            if current in visited:
                continue

            visited.add(current)
            nodes_taken.append(current)
            if prev_edges[current]:
                edges_taken.append(prev_edges[current])

            for edge in current.edges:
                neighbor = edge.get_opposite(current)
                if neighbor not in visited and edge.weight < node_costs[neighbor]:
                    node_costs[neighbor] = edge.weight
                    prev_edges[neighbor] = edge
                    heapq.heappush(priority_queue, (edge.weight, neighbor))

        self.apply_algorithm_result(nodes_taken, edges_taken)

    def kruskal(self):
        # Kruskal's Algorithm implementation
        if not self.edges or not self.nodes:
            print("The graph is empty.")
            return

        parent = {node: node for node in self.nodes}
        rank = {node: 0 for node in self.nodes}

        def find(node):
            if parent[node] != node:
                parent[node] = find(parent[node])
            return parent[node]

        def union(node1, node2):
            root1 = find(node1)
            root2 = find(node2)
            if root1 != root2:
                if rank[root1] > rank[root2]:
                    parent[root2] = root1
                elif rank[root1] < rank[root2]:
                    parent[root1] = root2
                else:
                    parent[root2] = root1
                    rank[root1] += 1

        edges_taken = []
        nodes_taken = set()
        sorted_edges = sorted(self.edges, key=lambda edge: edge.weight)

        for edge in sorted_edges:
            u, v = edge.node1, edge.node2
            if find(u) != find(v):
                union(u, v)
                edges_taken.append(edge)
                nodes_taken.add(u)
                nodes_taken.add(v)

        self.apply_algorithm_result(list(nodes_taken), edges_taken)

    def __str__(self):
        nodes_str = "\n  ".join(str(node) for node in self.nodes)
        edges_str = "\n  ".join(str(edge) for edge in self.edges)
        return f"Graph:\nNodes:\n  {nodes_str}\nEdges:\n  {edges_str}"