# Graph Algorithms and Application
 
## Table of Contents
- [Overview](#overview)
- [Todo](#todo)
- [Algorithms](#algorithms)
    - [BFS (Breadth-First Search)](#bfs-breadth-first-search)
        - [Requirements](#requirements)
        - [Pseudo Code](#pseudo-code)
    - [DFS (Depth-First Search)](#dfs-depth-first-search)
        - [Requirements](#requirements-1)
        - [Pseudo Code](#pseudo-code-1)
    - [UCS (Uniform Cost Search)](#ucs-uniform-cost-search)

---

## Overview

A Python application for graph-related operations and algorithm implementations. This includes visualizing graphs and running various search and shortest path algorithms.

---

## Todo

- [ ] Keep the display of the selected node(s) after applying an algorithm
- [ ] Add documentation (interfaces, usage, etc.)
- [ ] Implement all the algorithms
  - [ ] Greedy Best-First
  - [ ] A*
  - [ ] Dijkstra
  - [ ] Bellman-Ford
  - [ ] Floyd-Warshall
  - [ ] Prim
  - [ ] Kruskal

---

## Algorithms

### BFS (Breadth-First Search)

#### Requirements
- A single node must be selected to begin the search.

#### Pseudo Code

```plaintext
BreadthFirstSearch(Graph G, Vertex s):
    q = CreateQueue();
    q.enqueue(s);
    mark(s);
    while the queue is not empty:
        s = q.dequeue();
        print(s);
        for each neighbor t of s in G:
            if t is not marked:
                q.enqueue(t);
                mark(t);
```
Source: https://en.wikipedia.org/wiki/Breadth-first_search

---

### DFS (Depth-First Search)

#### Requirements
- A single node must be selected to begin the search.

#### Pseudo Code

```plaintext
DepthFirstSearchIterative(Graph G, Vertex s):
    stack = CreateStack()
    stack.push(s)
    while stack is not empty:
        s = stack.pop()
        if s is not marked:
            mark vertex s
            print vertex s
            for each vertex t adjacent to vertex s:
                if t is not marked:
                    stack.push(t)
```
Adapted from: https://en.wikipedia.org/wiki/Depth-first_search

---

### UCS (Uniform Cost Search)

#### Requirements
- A graph with weighted edges.
- A defined starting node and goal node.

#### Pseudo Code

```plaintext
UniformCostSearch(Graph G, Vertex start, Vertex goal):
    priorityQueue = CreateMinHeap();
    priorityQueue.insert((0, start)); // (cost, vertex)
    visited = CreateDictionary();
    visited[start] = (0, None); // (cost, parent)

    while priorityQueue is not empty:
        (currentCost, currentNode) = priorityQueue.removeMin();

        if currentNode == goal:
            return (currentCost, ReconstructPath(visited, start, goal));

        for each (neighbor, edgeCost) in G[currentNode]:
            totalCost = currentCost + edgeCost;

            if neighbor not in visited OR totalCost < visited[neighbor][0]:
                visited[neighbor] = (totalCost, currentNode);
                priorityQueue.insert((totalCost, neighbor));

    return None; // Goal is not reachable

ReconstructPath(Dictionary visited, Vertex start, Vertex goal):
    path = CreateEmptyList();
    current = goal;

    while current is not None:
        path.append(current);
        current = visited[current][1];

    path.reverse();
    return path;
```
Adaptated from: https://www.geeksforgeeks.org/uniform-cost-search-ucs-in-ai/

---

## License

Feel free to modify and use this project.