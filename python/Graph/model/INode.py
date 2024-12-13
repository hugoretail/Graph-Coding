from abc import ABC, abstractmethod

class INode(ABC):
    @abstractmethod
    def get_node(self):
        """TODO"""
        pass

    @abstractmethod
    def get_edges(self):
        """TODO"""
        pass

    def get_degree(self):
        """TODO"""
        pass

    def add_edge(self, edge):
        """TODO"""
        pass
