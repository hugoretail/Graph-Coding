from model import Graph
from model import Node
from model import Edge
import os
import sys

def load_nodes_from_file(file):
    nodes = []

    with open(file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            elements = line.split(",")
            if elements[0] == "N":
                try:
                    x = int(elements[1])
                    y = int(elements[2])
                    node = Node(x,y)
                    nodes.append(node)
                except (ValueError, IndexError):
                    print(f"Error while loading the node : {line}")

    return nodes

def load_edges_from_file(file, graph_object : Graph):
    edges_list = []

    with open(file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            elements = line.split(",")
            if elements[0] == "E":
                try:
                    x1 = int(elements[1])
                    y1 = int(elements[2])
                    x2 = int(elements[3])
                    y2 = int(elements[4])
                    node1 = graph_object.get_node_from_position(x1, y1)
                    node2 = graph_object.get_node_from_position(x2, y2)
                    edge = Edge(node1, node2)
                    edges_list.append(edge)
                except (ValueError, IndexError):
                    print(f"Error while loading the edge : {line}")

    return edges_list

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage : python Main.py <filename>")
        sys.exit(1)

    filename = sys.argv[1]
    script_dir = os.path.dirname(os.path.abspath(__file__))
    graph_folder = os.path.join(script_dir, "graphGenerator", "generated")
    graph_file = os.path.join(graph_folder, filename)

    if not os.path.isfile(graph_file):
        print(f"Error: the file '{filename}' doesn't exist in folder '{graph_folder}'.")
        sys.exit(1)

    graph = Graph()

    nodes = load_nodes_from_file(graph_file)
    graph.set_nodes(nodes)

    edges = load_edges_from_file(graph_file, graph)
    graph.set_edges(edges)

    print(graph)

    # display