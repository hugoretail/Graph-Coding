# Graph Algorithms and Application
 
## Table of Contents
- [Overview](#overview)
- [Todo](#todo)
- [Algorithms](#algorithms)
    - [BFS (Breadth-First Search)](#bfs-breadth-first-search)
    - [DFS (Depth-First Search)](#dfs-depth-first-search)
    - [UCS (Uniform Cost Search)](#ucs-uniform-cost-search)
    - [A* (AStar Search)](#a-astar-search)
    - TODO

---

## Overview

A Python application for graph-related operations and algorithm implementations. This includes visualizing graphs and running various search and shortest path algorithms.

---

## Todo

- [ ] Keep the display of the selected node(s) after applying an algorithm
- [ ] Add documentation (interfaces, usage, etc.)
- [ ] Implement all the algorithms
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
   procedure BreadthFirstSearch(Graph G, Vertex s):
       queue = CreateQueue()
       queue.enqueue(s)
       mark(s)
       while queue is not empty do:
           s = queue.dequeue()
           for each neighbor t of s do:
               if t is not marked then:
                   queue.enqueue(t)
                   mark(t)
```
Source: https://en.wikipedia.org/wiki/Breadth-first_search

---

### DFS (Depth-First Search)

#### Requirements
- A single node must be selected to begin the search.

#### Pseudo Code

```plaintext
   procedure DepthFirstSearchIterative(Graph G, Vertex s):
       stack = CreateStack()
       stack.push(s)
       while stack is not empty do:
           s = stack.pop()
           if s is not marked then:
               mark(s)
               for each neighbor t of s do:
                   if t is not marked then:
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
   procedure UniformCostSearch(Graph G, Vertex start, Vertex goal):
       priorityQueue = CreateMinHeap()
       priorityQueue.insert((0, start)) // (cost, vertex)
       visited = CreateDictionary()
       visited[start] = (0, None) // (cost, parent)

       while priorityQueue is not empty do:
           (currentCost, currentNode) = priorityQueue.removeMin()
           if currentNode == goal then:
               return ReconstructPath(visited, start, goal)

           for each (neighbor, edgeCost) in Neighbors(currentNode) do:
               totalCost = currentCost + edgeCost
               if neighbor not in visited OR totalCost < visited[neighbor][0] then:
                   visited[neighbor] = (totalCost, currentNode)
                   priorityQueue.insert((totalCost, neighbor))
       return failure
```
Adapted from: https://www.geeksforgeeks.org/uniform-cost-search-ucs-in-ai/

---

### A* (AStar Search)

#### Requirements
- A defined starting node and target node.
- A heuristic function `h(n)` to estimate the cost to reach the goal from node `n`.

#### Pseudo Code

```plaintext
   procedure AStar(Graph G, Vertex start, Vertex goal, Function h):
       openSet = CreatePriorityQueue()
       openSet.insert((h(start), start)) // (fScore, node)

       cameFrom = Dictionary
       gScore = Dictionary with default value INFINITY
       gScore[start] = 0

       while openSet is not empty do:
           (f, current) = openSet.removeMin()

           if current == goal then:
               return ReconstructPath(cameFrom, goal)

           for each neighbor in Neighbors(current) do:
               tentative_gScore = gScore[current] + Cost(current, neighbor)

               if tentative_gScore < gScore[neighbor] then:
                   cameFrom[neighbor] = current
                   gScore[neighbor] = tentative_gScore
                   f = tentative_gScore + h(neighbor)
                   openSet.insert((f, neighbor))
       return failure
```
Source: https://en.wikipedia.org/wiki/A*_search_algorithm

### GBS (Greedy Best-First Search)

#### Requirements
- A defined starting node and target node.
- A heuristic function to estimate the "distance" from the current node to the target.

---

#### Pseudo Code

```plaintext
    procedure GBS(start, target) is:
      mark start as visited
      add start to queue
      while queue is not empty do:
        current_node ← vertex of queue with min distance to target
        remove current_node from queue
        foreach neighbor n of current_node do:
          if n not in visited then:
            if n is target:
              return n
            else:
              mark n as visited
              add n to queue
      return failure
```
Source: https://en.wikipedia.org/wiki/Best-first_search

---

### A* (A-Star Search)

#### Requirements
- A graph with weighted edges.
- A defined starting node and goal node.
- A heuristic function to estimate the cost from a node to the goal.

#### Pseudo Code

```plaintext
AStar(Graph G, Vertex start, Vertex goal, Function h):
    openSet ← CreatePriorityQueue()
    openSet.insert((h(start), start)) // (fScore, node)

    cameFrom ← CreateDictionary()
    gScore ← CreateDictionaryWithDefaultValue(Infinity)
    gScore[start] ← 0

    path ← CreateEmptyList()

    while openSet is not empty do:
        (f, current) ← openSet.removeMin()

        if current = goal then:
            current ← goal
            while current is not None do:
                path.prepend(current)
                current ← cameFrom[current] if current in cameFrom else None
            return path

        for each (neighbor, edgeCost) in G[current] do:
            tentative_gScore ← gScore[current] + edgeCost

            if tentative_gScore < gScore[neighbor] then:
                cameFrom[neighbor] ← current
                gScore[neighbor] ← tentative_gScore
                f ← tentative_gScore + h(neighbor)

                if neighbor not in openSet then:
                    openSet.insert((f, neighbor))

    return failure
``` 
Adapted from: https://en.wikipedia.org/wiki/Best-first_search

---

### Dijkstra’s Algorithm

#### Requirements
- A graph with weighted edges.
- A defined starting and end node.

#### Pseudo Code

```plaintext
procedure Dijkstra(Graph G, Vertex source):
    dist = Dictionary with default value INFINITY
    prev = Dictionary
    dist[source] = 0
    Q = All vertices in G

    while Q is not empty do:
        u = Vertex in Q with minimum dist[u]
        remove u from Q
        for each neighbor v of u still in Q do:
            alt = dist[u] + Cost(u, v)
            if alt < dist[v] then:
                dist[v] = alt
                prev[v] = u
    return dist, prev
```
Source: https://en.wikipedia.org/wiki/Dijkstra's_algorithm

---



## License

Feel free to modify and use this project.
