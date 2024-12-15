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