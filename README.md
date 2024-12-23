# Graph Algorithms and Application

## Table of Contents
- [Overview](#overview)
- [Todo](#todo)
- [Algorithms](#algorithms)
    - [BFS (Breadth-First Search)](#bfs-breadth-first-search)
        - [Requirements](#requirements)
        - [Pseudo Code](#pseudo-code)

---

## Overview

A Python application for graph-related operations and algorithm implementations. This includes visualizing graphs and running various search and shortest path algorithms.

---

## Todo

- [ ] Add documentation (interfaces, usage, etc.)
- [ ] Implement all the algorithms

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

### DFS

[TODO]

---

## License

Feel free to modify and use this project.