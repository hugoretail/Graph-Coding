from PyQt5.QtWidgets import QMainWindow, QGraphicsScene, QGraphicsView, QAction, QFileDialog, QGraphicsEllipseItem, \
    QGraphicsTextItem
from PyQt5.QtGui import QPen, QBrush, QColor, QLinearGradient
from PyQt5.QtCore import Qt
from .design_view import Ui_MainWindow
from typing import List
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

        self.ui.actionLocal_File.triggered.connect(self.open_file_dialog)

        self.apply_algorithms_events()

    def update_node_styles(self, clicked_nodes):
        for node, number in clicked_nodes:
            gradient = QLinearGradient(node.x, node.y, node.x + 10, node.y + 10)
            gradient.setColorAt(0, QColor(255, 102, 102))
            gradient.setColorAt(1, QColor(153, 0, 0))

            # styles
            pen = QPen(Qt.black)
            pen.setWidth(1)
            brush = QBrush(gradient)

            ellipse = QGraphicsEllipseItem(node.x - 5, node.y - 5, 10, 10)
            ellipse.setPen(pen)
            ellipse.setBrush(brush)

            self.scene.addItem(ellipse)

            # number
            text = QGraphicsTextItem(str(number))
            text.setDefaultTextColor(Qt.white)
            text.setFont(text.font())
            text.setZValue(1)
            text_width = text.boundingRect().width()
            text_height = text.boundingRect().height()
            text.setPos(node.x - text_width / 2, node.y - text_height / 2)

            self.scene.addItem(text)

    def toggle_algorithms_menu(self, algorithms: List[str]):
        for action in self.ui.menuSearch_Algorithms.actions():
            if action.text() in algorithms:
                action.setEnabled(True)
            else:
                action.setDisabled(True)

    def enable_algorithms_menu(self):
        for action in self.ui.menuSearch_Algorithms.actions():
            action.setEnabled(True) # clickable

    def disable_algorithms_menu(self):
        for action in self.ui.menuSearch_Algorithms.actions():
            action.setDisabled(True) # not clickable

    def disable_algorithms_menu_button(self):
        self.ui.menuSearch_Algorithms.setDisabled(True)

    def enable_algorithms_menu_button(self):
        self.ui.menuSearch_Algorithms.setEnabled(True)

    def apply_algorithms_events(self):
        for action in self.ui.menuSearch_Algorithms.actions():
            action.setDisabled(True) # not clickable
            action.triggered.connect(lambda checked, alg=action.text(): self.controller.apply_algorithm(alg))

    def populate_default_graphs_menu(self):
        graph_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'graphGenerator', 'generated')

        if os.path.exists(graph_folder):
            for file_name in os.listdir(graph_folder):
                if file_name.endswith(".txt"):
                    filepath = os.path.join(graph_folder, file_name)
                    action = QAction(file_name, self)
                    action.triggered.connect(lambda checked, path=filepath: self.controller.graph_chosen_event(path))
                    self.ui.menuDefault_Graphs.addAction(action)

    def open_file_dialog(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open file", "", "Text Files (*.txt);; All Files (*)")

        if file_path:
            self.controller.graph_chosen_event(file_path)

    def set_selected_graph(self, path):
        self.selected_graph_path = path
        self.ui.label.setText(f"Selected graph: {os.path.basename(path)}")
        print(f"Graph selected: {path}")

    def update_graph(self, nodes, edges):
        self.scene.clear()

        edge_pen = QPen(QColor(50,50,50)) # dark gray
        edge_pen.setWidth(1)

        for edge in edges:
            self.scene.addLine(edge.node1.x,
                               edge.node1.y,
                               edge.node2.x,
                               edge.node2.y,
                               pen=edge_pen)

        for node in nodes:
            x, y = node.x, node.y

            # styles
            gradient = QLinearGradient(x,y,x+10,y+10)
            gradient.setColorAt(0, QColor(135,206,250)) # light blue
            gradient.setColorAt(1,QColor(0,90,180)) # dark blue
            pen = QPen(Qt.black)
            pen.setWidth(1)
            brush = QBrush(gradient)

            # events
            ellipse = QGraphicsEllipseItem(x-5,y-5,10,10)
            ellipse.setPen(pen)
            ellipse.setBrush(brush)
            ellipse.setFlags(QGraphicsEllipseItem.ItemIsSelectable | QGraphicsEllipseItem.ItemIsFocusable)
            ellipse.mousePressEvent = lambda event, n= node: self.controller.node_clicked_event(n)

            self.scene.addItem(ellipse)

