#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

using namespace std;

struct Node {
    int x;
    int y;
    double g;
    double h;
    Node* parent;

    Node() : g(INT_MAX), h(INT_MAX) {}

    Node(int x, int y) : x(x), y(y), g(INT_MAX), h(INT_MAX) {}

    double f() const { return g + h; }
};

class AStar {
public:
    AStar(const vector<vector<int>>& grid) : grid(grid), rows(grid.size()), cols(grid[0].size()) {}

    vector<Node> search(const Node& start, const Node& goal) {
        priority_queue<Node, vector<Node>, greater<>> openSet;
        unordered_map<long long, Node> nodeMap;
        unordered_set<long long> closedSet;

        long long keyStart = make_llong_key(start);
        nodeMap[keyStart] = start;
        openSet.emplace(start);

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            if (current.x == goal.x && current.y == goal.y) {
                return reconstructPath(current);
            }

            closedSet.insert(make_llong_key(current));

            for (const auto& dirs : { {0, 1}, {1, 0}, {0, -1}, {-1, 0} }) {
                int nx = current.x + dirs[0];
                int ny = current.y + dirs[1];

                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] == 0) {
                    long long key = make_llong_key(Node(nx, ny));

                    if (closedSet.find(key) != closedSet.end()) {
                        continue;
                    }

                    double tentativeG = current.g + 1.0; // Assuming each step has distance 1
                    if (nodeMap.find(key) == nodeMap.end() || tentativeG < nodeMap[key].g) {
                        Node& neighbor = nodeMap[key];
                        neighbor.g = tentativeG;
                        neighbor.h = distance(neighbor, goal);
                        neighbor.parent = &current;
                        openSet.emplace(neighbor);
                    }
                }
            }
        }

        return {};
    }

    double distance(const Node& a, const Node& b) {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    vector<Node> reconstructPath(const Node& current) {
        vector<Node> path;
        while (current.parent) {
            path.push_back(current);
            current = *current.parent;
        }
        path.push_back(current);
        reverse(path.begin(), path.end());
        return path;
    }

    // Hybrid key to store integers and update efficiently based on integer values.
    long long make_llong_key(const Node& n) const {
        return static_cast<long long>(n.x) << 32 | static_cast<long long>(n.y);
    }

private:
    const vector<vector<int>>& grid;
    size_t rows;
    size_t cols;
};

int main() {
    // Huge sample grid
    int n = 1000; // Customize dataset size for testing
    vector<vector<int>> grid(n, vector<int>(n, 0));
    int obstacleCount = n * n * 0.1; // Customize obstacle count
    for (int i = 0; i < obstacleCount; ++i) {
        grid[rand() % n][rand() % n] = 1;
    }
