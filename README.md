# A* Path Planning Algorithm in C++

## Description
This project implements the _A pathfinding algorithm*_ to find the shortest path between a start (`'S'`) and a goal (`'G'`) in a grid-based environment. The grid contains obstacles (`'|'`), and the algorithm marks the optimal path with dots (`'.'`).

## Features
Uses the _A algorithm*_, combining cost-based search with a heuristic function (Manhattan distance).
Processes a fixed-size grid where paths are calculated dynamically.
Outputs the grid with a visual representation of the path.

## How It Works
The grid is initialized with obstacles, a start position, and a goal.
The _A algorithm*_ explores paths based on cost and heuristic.
If a path is found, it is marked with `'.'` from `'S'` to `'G'`.
The updated grid is printed to the console.

## Example Output
```
 S   |    
 . ||     
 .   |    
 ..| |  G 
 |.. |  . 
   .||||.
|| ......
```
## How to Run
Compile with `g++ -o path-planning path-planning.cpp`
Run the program: `./path-planning`