from PyQt5.QtWidgets import QMainWindow, QGraphicsScene, QGraphicsView
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt
from .design_view import Ui_MainWindow

class GraphView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.scene = QGraphicsScene(self)
        self.graphView = QGraphicsView(self.scene, self)
        self.graphView.setGeometry(0, 0, 600, 600)

        self.ui.centralwidget.layout().addWidget(self.graphView)

    def update_graph(self, nodes, edges):
        self.scene.clear()
        for node in nodes:
            x, y = node.x, node.y
            self.scene.addEllipse(x-5,y-5,10,10, pen=QPen(Qt.black))

        for edge in edges:
            self.scene.addLine(edge.node1.x,
                               edge.node1.y,
                               edge.node2.x,
                               edge.node2.y,
                               pen=QPen(Qt.blue))

