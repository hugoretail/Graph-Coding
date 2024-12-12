from abc import ABC, abstractmethod

class INode(ABC):
    @abstractmethod
    def getNode(self):
        """TODO"""
        pass

    @abstractmethod
    def setNode(self):
        """TODO"""
        pass

    @abstractmethod
    def getEdges(self):
        """TODO"""
        pass

    def getDegree(self):
        """TODO"""
        pass