from abc import ABC, abstractmethod

class IEdge(ABC):
    @abstractmethod
    def getEdge(self):
        """TODO"""
        pass

    @abstractmethod
    def setEdge(self):
        """TODO"""
        pass

    @abstractmethod
    def getNodes(self):
        """TODO"""
        pass

    @abstractmethod
    def getWeight(self):
        """TODO"""
        pass

    @abstractmethod
    def setWeight(self, n1, n2):
        """TODO"""
        pass