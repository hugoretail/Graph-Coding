import sys
import random
import os

def __main__():
    if len(sys.argv) != 3:
        print("Usage : python generator.py <nb_nodes> <nb_edges>")
        sys.exit(1)

    nb_nodes = int(sys.argv[1])
    nb_edges = int(sys.argv[2])

    if (nb_nodes*(nb_nodes-1))/2 < nb_edges:
        print("Error : too many edges")
        sys.exit(1)

    x_min = 0
    y_min = 0
    x_max = 500
    y_max = 500

    available_positions = []
    for x in range(x_max):
        for y in range(y_max):
            available_positions.append((x,y))

    nodes = []
    for _ in range(nb_nodes):
        rdm_pos_index = random.randint(0, len(available_positions)-1)
        x, y = available_positions[rdm_pos_index]
        nodes.append((x,y))
        available_positions.remove((x,y))

    available_nodes = []
    for i in range(nb_nodes - 1):
        for j in range(i + 1, nb_nodes):
            available_nodes.append( ( (nodes[i][0],nodes[i][1]),(nodes[j][0],nodes[j][1]) ) ) # [ (n1, n2) ] with n1 = (x,y) and n2 = (x,y)

    edges = []
    for _ in range(nb_edges):
        rdm_pos_index = random.randint(0, len(available_nodes)-1)
        edges.append(available_nodes[rdm_pos_index])
        available_nodes.remove(available_nodes[rdm_pos_index])

    script_dir = os.path.dirname(os.path.abspath(__file__))
    folder = os.path.join(script_dir, "generated")
    if not os.path.exists(folder):
        os.makedirs(folder)
    content = os.listdir(folder)
    files = [file for file in content if os.path.isfile(os.path.join(folder,file))]
    nb_files = len(files)

    with open(os.path.join(folder, f"graph_{nb_files}.txt"), "w") as f:
        # write nodes
        for n in nodes:
            f.write("N," + str(n[0]) + "," + str(n[1]) + "\n") # N,x,y

        # write edges
        for e in edges:
            f.write("E,"
                    + str(e[0][0]) + ","
                    + str(e[0][1]) + ","
                    + str(e[1][0]) + ","
                    + str(e[1][1])
                    + "\n") # E,x1,y1,x2,y2

if __name__ == "__main__":
    __main__()