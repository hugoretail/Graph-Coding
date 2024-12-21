from PyQt5.QtWidgets import QMainWindow, QGraphicsScene, QGraphicsView, QAction
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt
from .design_view import Ui_MainWindow
import os

class GraphView(QMainWindow):
    def __init__(self, controller = None):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.scene = QGraphicsScene(self)
        self.graphView = QGraphicsView(self.scene, self)
        self.graphView.setGeometry(0, 0, 600, 600)

        self.ui.centralwidget.layout().addWidget(self.graphView)

        self.controller = controller

        self.selected_graph_path = None
        self.populate_default_graphs_menu()

    def populate_default_graphs_menu(self):
        graph_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'graphGenerator', 'generated')

        if os.path.exists(graph_folder):
            for file_name in os.listdir(graph_folder):
                if file_name.endswith(".txt"):
                    filepath = os.path.join(graph_folder, file_name)
                    action = QAction(file_name, self)
                    action.triggered.connect(lambda checked, path=filepath: self.set_selected_graph(path))
                    self.ui.menuDefault_Graphs.addAction(action)

    def set_selected_graph(self, path):
        self.selected_graph_path = path

        if self.controller:
            self.controller.load_graph(path)

        self.ui.label.setText(f"Selected graph: {os.path.basename(path)}")
        print(f"Graph selected: {path}")

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

