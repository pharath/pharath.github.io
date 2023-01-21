---
title: "Introduction to Artificial Intelligence - Notes"
read_time: false
excerpt: "For learning Introduction to AI; content mostly from RWTH lecture Introduction to Artificial Intelligence by G. Lakemeyer."
header:
  teaser: /assets/images/C_logo.png
  overlay_image: /assets/images/C_logo.png
  overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
  - Notes
tags:
  - ai
  - notes
  - lecture

---

# Org

- search
    - learn space and time complexities later
    - focus on basic properties

# Search

- **frontier** of the search tree: set of unexpanded nodes
- **redundant** path: a sub-optimal way to get to the same state, e.g. a path containing a cycle
- graph search: 
    - checks for redundant paths (keeps a table of reached states)
- tree-like search: 
    - does not check for redundant paths (does **not** keep a table of reached states)

## Best-first Search

- chooses a node (from the frontier), $n$, with **minimum** value of some **evaluation function**, $f(n)$

## Breadth-first Search (BFS)

- waves of uniform depth
- best-first search where $f(n)$ is the depth of the node
- **complete**: always finds a solution with a minimal number of actions (shallowest solution)
- **cost-optimal**: for problems where all actions have the identical non-negative cost
- time **and** space complexity: $\mathcal{O}(b^{d+1})$ 
    - The memory requirements are a bigger problem than time

## Uniform Cost Search, Dijkstra's Algorithm

- waves of uniform path-cost
- best-first search where $f(n)$ is the path-cost of the node
- recall Figure 3.10 example: tests for goals only when it expands a node, not when it generates a node
- always finds cheapest solution, if non-negative action cost for all n
- $\mathcal{O}(b^{1 + \lfloor C^\ast/\epsilon \rfloor})$ time and space complexity, $\epsilon$ cost of cheapest action, $C^\ast$ cost of the cheapest solution
- When all action costs are equal, uniform cost search is similar to breadth-first search
- Uniform-cost search is **complete** and is **cost-optimal**, if non-negative action cost for all n

## Depth-first Search

- expands the deepest node in the frontier first
- usually implemented as a **tree-like search** (s.o.)
- space: $\mathcal{O}(b \cdot m)$, where $m$: max depth
- time: $\mathcal{O}(b^m)$
- **not cost-optimal**; it returns the first solution it finds, even if it is not cheapest
- **incomplete**: can get stuck going down an infinite path ($m=\infty$), even if there are no cycles
- then why use it?
    - much smaller needs for memory than BFS
    - reason why adopted as the basic workhorse of many areas of AI

## Depth-limited Search

- to keep depth-first search from wandering down an infinite path
- depth-first search in which we supply a **depth limit**, $l$, and treat all nodes at depth $l$ as if they had no successors
- space: $\mathcal{O}(bl)$
- time: $\mathcal{O}(b^l)$
- **incomplete**: if we make a poor choice for $l$ the algorithm will fail to reach the solution, making it incomplete again
- **not cost-optimal**; it returns the first solution it finds, even if it is not cheapest (like depth-first search)
- how to find a good depth limit?
    - **diameter** of the state-space graph
        - e.g. any city can be reached from any other city in at most **9** actions
        - use it as a better depth limit, which leads to a more efficient depth-limited search
    - however, for most problems we will not know a good depth limit until we have solved the problem

## Iterative Deepening Search

- combines depth-first search and BFS
- solves the problem of picking a good value for $l$ by trying all values: first 0, then 1, then 2, and so on - until either a solution is found, or the depth-limited search returns the `failure` value rather than the `cutoff` value
- space: $\mathcal{O}(bd)$ (better than BFS)
- time: $\mathcal{O}(b^d)$, when there is a solution (asymptotically the same as BFS)
- **cost-optimal**: for problems where all actions have the identical non-negative cost (like BFS)
- **complete**, on any finite acyclic state space (like BFS)
- trade-off: breadth-first stores all nodes in memory, while iterative deepening repeats the previous levels, thereby **saving memory at the cost of more time**
- wasteful because **states** near the top of the search tree are **re-generated multiple times**? 
    - No, because for many state spaces, most of the nodes are in the bottom level, so it does not matter much that the upper levels are repeated
- preferred method when 
    - the search state space is larger than can fit in memory and 
    - the depth of the solution is not known
- lecture: iterative deepening is better than BFS (not because of time, but because of space!)

## Bidirectional Search

- Assuming that forward and backward search are symmetric
- time: $\mathcal{O}(b^{d/2})$ ideally
- problems:
    - Operators not always reversible
    - Sometimes there are very **many goal states**, e.g. chess

## Greedy Best-first Search

- evaluation function $f(n)=h(n)$
