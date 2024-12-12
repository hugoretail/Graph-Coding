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

    