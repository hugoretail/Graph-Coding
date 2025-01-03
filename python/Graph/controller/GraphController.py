class GraphController:
    def __init__(self, model, view):
        self.model = model
        self.view = view

    def load_graph(self, path):
        self.model.load_graph(path)

    def update_view(self):
        self.view.update_graph(self.model.nodes, self.model.edges)

    def apply_algorithm(self, algorithm : str):
        switch = {
            "BFS": self.model.bfs,
            "DFS": self.model.dfs,
            "UCS": self.model.ucs,
            "Greedy Best-First": self.model.greedy_best_first,
            "A*": self.model.a_star,
            "Dijkstra": self.model.dijkstra,
            "Bellman-Ford": self.model.bellman_ford,
            "Floyd-Warshall": self.model.floyd_warshall,
            "Prim": self.model.prim,
            "Kruskal": self.model.kruskal
        }
        try:
            return switch[algorithm]()
        except KeyError:
            print(f"Algorithm '{algorithm}' not found.")

    def graph_chosen_event(self, path):
        self.view.set_selected_graph(path)
        self.view.enable_algorithms_menu_button()
        self.view.disable_algorithms_menu()
        self.view.disable_reset_button()
        self.load_graph(path)

    def node_clicked_event(self, node):
        self.model.select_node(node)
        self.update_view()
        self.view.update_node_styles(self.model.selected_nodes)

        if self.model.selected_nodes_counter == 1:
            self.view.toggle_algorithms_menu(["BFS", "DFS", "Bellman-Ford"])
        elif self.model.selected_nodes_counter == 2:
            self.view.toggle_algorithms_menu(["UCS", "A*", "Dijkstra", "Bellman-Ford", "Floyd-Warshall"])
        else:
            self.view.disable_algorithms_menu()

    def reset_current_graph(self):
        self.model.reload_graph()
        self.view.disable_reset_button()
        self.view.enable_algorithms_menu_button()
        self.view.disable_algorithms_menu()

    def reset_everything(self):
        """TODO"""
        pass