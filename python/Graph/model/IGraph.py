from abc import ABC, abstractmethod

class IGraph(ABC):
    @abstractmethod
    def get_node_from_position(self, x, y):
        """TODO"""
        pass

    @abstractmethod
    def set_nodes(self, nodes):
        """TODO"""
        pass

    @abstractmethod
    def set_edges(self, edges):
        """TODO"""
        pass

    @abstractmethod
    def get_edge_from_positions(self, x1, y1, x2, y2):
        """TODO"""
        pass

    @abstractmethod
    def get_neighbors(self, node):
        """TODO"""
        pass