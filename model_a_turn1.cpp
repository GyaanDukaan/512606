#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

struct Node {
    int x, y;
    double g, h; // g: cost from start, h: heuristic cost to goal
    Node* parent;

    Node(int x, int y, double g = 0, double h = 0, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const { return g + h; }

    bool operator>(const Node& other) const { return f() > other.f(); }
};


class AStar {
public:
    AStar(const std::vector<std::vector<int>>& grid)
        : grid(grid), rows(grid.size()), cols(grid[0].size()) {}

    std::vector<Node> search(const Node& start, const Node& goal) {
        std::vector<Node*> openSet;
        std::unordered_map<int, Node*> allNodes;

        openSet.push_back(new Node(start.x, start.y, 0, heuristic(start, goal)));
        allNodes[hashPos(start.x, start.y)] = openSet.back();

        while (!openSet.empty()) {
            // Sort openSet to find node with the lowest f value
            std::sort(openSet.begin(), openSet.end(), [](Node* a, Node* b) { return *a > *b; });
            Node* current = openSet.back();
            openSet.pop_back();

            if (current->x == goal.x && current->y == goal.y) {
                return reconstructPath(*current);
            }

            for (auto& neighbor : getNeighbors(*current)) {
                double tentative_g = current->g + distance(*current, neighbor);

                if (allNodes.find(hashPos(neighbor.x, neighbor.y)) == allNodes.end() ||
                    tentative_g < allNodes[hashPos(neighbor.x, neighbor.y)]->g) {

                    Node* neighborNode = new Node(neighbor.x, neighbor.y, tentative_g, heuristic(neighbor, goal), current);
                    allNodes[hashPos(neighbor.x, neighbor.y)] = neighborNode;

                    openSet.push_back(neighborNode);
                }
            }

            delete current; // Free memory of the current node
        }

        return {}; // Return empty path if no solution found
    }

private:
    std::vector<std::vector<int>> grid;
    size_t rows, cols;

    double heuristic(const Node& a, const Node& b) {
        // Using Manhattan distance as heuristic
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    double distance(const Node& a, const Node& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    std::vector<Node> reconstructPath(const Node& current) {
        std::vector<Node> path;
        const Node* node = &current;
        while (node) {
            path.push_back(*node);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;
        const std::vector<std::pair<int, int>> directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0} };

        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] == 0) {
                neighbors.emplace_back(nx, ny);
            }
        }

        return neighbors;
    }

    int hashPos(int x, int y) const {
        return x * cols + y; // Simple hash based on grid position
    }
};

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0},
    };

    AStar astar(grid);
    Node start(0, 0);
    Node goal(4, 4);

    std::vector<Node> path = astar.search(start, goal);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ") ";
        }
    }
    else {
        std::cout << "No path found!";
    }

    return 0;
}
