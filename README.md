# VLSI Global Router

## Overview
A high-performance global routing engine engineered in C++ for VLSI (Very Large Scale Integration) chip design. The router connects multi-pin components across a heavily congested 2D grid while actively minimizing wirelength, avoiding routing blockages, and resolving capacity overflows.

## Architecture & Algorithmic Strategy
* **Minimum Spanning Trees (MST):** Multi-pin nets (3+ pins) are decomposed into optimal 2-pin sub-nets using Prim's algorithm to minimize baseline wirelength before routing begins.
* **A* Heuristic Search:** Utilizes a highly optimized A* pathfinding algorithm. To maximize speed, the search is constrained within dynamically expanding bounding boxes, forcing fast Z-shape routes first and only expanding the search perimeter when blockages occur.
* **History-Based Rip-Up and Reroute (RRR):** Implements a time-bound optimization loop. Edges that experience traffic overflow are penalized with a historical weight multiplier. The engine identifies the most congested nets, "rips up" their specific overflowing segments, and reroutes them through newly penalized paths to naturally disperse traffic jams.

## Measurable Performance
The implementation of the Rip-Up and Reroute optimization loop resulted in massive reductions in routing congestion (Q-Score) across standard Adaptec benchmarks over a 5-minute compute window:

| Benchmark | Pre-Optimization Q-Score | Post-Optimization Q-Score | Improvement |
| :--- | :--- | :--- | :--- |
| **Adaptec 1** | 492,140 | 213,913 | **56.5% Reduction** |
| **Adaptec 2** | 46,650 | 774 | **98.3% Reduction** |
| **Adaptec 3** | 706,927 | 133,803 | **81.0% Reduction** |

## Build & Execute
Compile the routing engine using the provided `Makefile`:

```bash
make
```

Execute the router by specifying the pin-ordering flag (`-d`), the Rip-Up and Reroute flag (`-n`), the input benchmark file, and the desired output file:
```bash
./ROUTE.exe -d=1 -n=1 input_benchmark.gr output_route.txt
```