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

---

## Overview

A Python application for graph-related operations and algorithm implementations. This includes visualizing graphs and running various search and shortest path algorithms.

---

## Todo

- [ ] Add documentation (interfaces, usage, etc.)
- [ ] Implement all the algorithms
  - [X] BFS
  - [ ] DFS
  - [ ] UCS
  - [ ] Greedy Best-First
  - [ ] A*
  - [ ] Dijkstra
  - [ ] Bellman-Ford
  - [ ] Floyd-Warshall
  - [ ] Prim
  - [ ] Kruskal
- [ ] Add a button to reset (restore) the current graph to its initial state.

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
Source : https://en.wikipedia.org/wiki/Breadth-first_search

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
Source : Adapted from https://en.wikipedia.org/wiki/Depth-first_search

---

## License

Feel free to modify and use this project.