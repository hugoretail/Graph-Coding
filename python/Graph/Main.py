from model.Graph import Graph
from view.GraphView import GraphView
from controller.GraphController import GraphController
from PyQt5.QtWidgets import QApplication
import sys

if __name__ == '__main__':
    if len(sys.argv) != 1:
        print("Usage : python Main.py")
        sys.exit(1)

    app = QApplication(sys.argv)

    model = Graph()
    view = GraphView()
    controller = GraphController(model, view)

    view.controller = controller
    model.view = view

    view.show()

    sys.exit(app.exec_())
    