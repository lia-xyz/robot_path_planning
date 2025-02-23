/*
A* Path Planning Algorithm in C++
Author: Natalia Semjonova
Date: 23.02.2025
Description:
This program finds the shortest path from 'S' (start) to 'G' (goal) in a grid-based environment using
the A* algorithm. The found path is marked with '.'.
*/

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <unordered_map>

using namespace std;

// Grid dimensions
const int WIDTH = 10, HEIGHT = 7;

// Movement directions (left, right, up, down)
const int dx[4] = {-1, 1, 0, 0};
const int dy[4] = {0, 0, -1, 1};

// Initial grid representation ('S' = start position, 'G' = goal position, '|' = obstacle)
vector<string> grid = {
    " S   |    ",
    "   ||     ",
    "     |    ",
    "   | |  G ",
    " |   |    ",
    "    ||||  ",
    "||        "
};

// Node structure for priority queue (A* algorithm)
struct Node{
    int x, y, cost, heuristic;
    Node(int new_x, int new_y, int new_cost, int new_heuristic)
        : x(new_x), y(new_y), cost(new_cost), heuristic(new_heuristic) {}

    // Comparison operator for priority queue (min-heap)
    bool operator>(const Node &other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

// Finds start and goal positions in the grid
pair<int, int> findPosition(char c){
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH; x++){
            if(grid[y][x] == c){
                return {x, y};
            }
        }
    }
    return {-1, -1}; // Return invalid position if character is not found
}

// Manhattan distance heuristic function
int findHeuristic(int x, int y, int gx, int gy) {
    return abs(gx - x) + abs(gy - y);
}

// A* pathfinding function
void findPath(int sx, int sy, int gx, int gy){
    priority_queue<Node, vector<Node>, greater<Node>> nodes;
    unordered_map<int, pair<int, int>> previous_positions;
    vector<vector<int>> costs(HEIGHT, vector<int>(WIDTH, 1e9));

    // Start node
    nodes.push(Node(sx, sy, 0, findHeuristic(sx, sy, gx, gy)));
    costs[sy][sx] = 0;

    while(!nodes.empty()){
        Node current_position = nodes.top();
        nodes.pop();

        // If goal is reached, reconstruct path
        if(current_position.x == gx && current_position.y == gy){
            int x = gx;
            int y = gy;

            while(!(x == sx && y == sy)){
                grid[y][x] = '.'; // Mark path
                tie(x, y) = previous_positions[y * WIDTH + x];
            }
            grid[sy][sx] = 'S'; // Ensure start remains marked
            grid[gy][gx] = 'G'; // Ensure goal remains marked

            return;
        }

        // Explore map
        for(int i = 0; i < 4; i++){
            int nx = current_position.x + dx[i];
            int ny = current_position.y + dy[i];

            // Check if the move is within bounds and not an obstacle
            if(nx < 0 || ny < 0 || nx >= WIDTH || ny >= HEIGHT || grid[ny][nx] == '|')
                continue;
            
            int new_cost = current_position.cost + 1;
            if(new_cost < costs[ny][nx]){
                costs[ny][nx] = new_cost;
                nodes.push(Node(nx, ny, new_cost, findHeuristic(nx, ny, gx, gy)));
                previous_positions[ny * WIDTH + nx] = {current_position.x, current_position.y};
            }

        }
    }
    cout << "No path found!\n";
}

int main(){
    auto [sx, sy] = findPosition('S'); // Get start position
    auto [gx, gy] = findPosition('G'); // Get goal position

    findPath(sx, sy, gx, gy); // Run A* pathfinding algorithm

    // Print the updated grid with the found path
    for (const auto &row : grid){
        cout << row << '\n';
    }

    return 0;
}