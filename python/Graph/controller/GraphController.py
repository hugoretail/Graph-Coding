class GraphController:
    def __init__(self, model, view):
        self.model = model
        self.view = view

    def load_graph(self, filename):
        self.model.load_nodes_from_file(filename)
        self.model.load_edges_from_file(filename)

        self.update_view()

    def update_view(self):
        self.view.update_graph(self.model.nodes, self.model.edges)

    def apply_algorithm(self, algorithm : str):
        if len(self.model.selected_nodes) != 2:
            print("Error! Please select 2 nodes to apply the algorithm.")

        switch = {
            "BFS": self.model.bfs(),
            "DFS": self.model.dfs(),
            "UCS": self.model.ucs(),
            "Greedy Best-First": self.model.greedy_best_first(),
            "A*": self.model.a_star(),
            "Dijkstra": self.model.dijkstra(),
            "Bellman-Ford": self.model.bellman_ford(),
            "Floyd-Warshall": self.model.floyd_warshall(),
            "Prim": self.model.prim(),
            "Kruskal": self.model.kruskal()
        }

    def graph_chosen_event(self, path):
        self.view.set_selected_graph(path)
        self.load_graph(path)

    def node_clicked_event(self, node):
        """TODO"""
        pass