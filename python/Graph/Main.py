from model import Graph
import os
import sys

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

    graph = Graph(graph_file)