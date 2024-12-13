from abc import ABC, abstractmethod

class IGraph(ABC):

    @abstractmethod
    def getNodes(self):
        """TODO"""
        pass

    @abstractmethod
    def getEdges(self):
        """TODO"""
        pass

    @abstractmethod
    def getNodeFromPosition(self, x, y):
        """TODO"""
        pass

    @abstractmethod
    def getEdgeFromPositions(self, x1, y1, x2, y2):
        """TODO"""
        pass

    def getNeighbours(self, node):
        """TODO"""
        pass

