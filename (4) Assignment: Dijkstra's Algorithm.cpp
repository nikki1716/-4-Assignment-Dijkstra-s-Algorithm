#include <iostream>
#include <vector>
#include <limits>
#include <set>
#include <unordered_map>

using namespace std;

const int INF = numeric_limits<int>::max();

class Graph {
public:
    unordered_map<int, vector<pair<int, int>>> nodes;

    void addEdge(int from, int to, int weight) {
        nodes[from].push_back({to, weight});
        nodes[to].push_back({from, weight}); // For undirected graph
    }
};

void dijkstra(const Graph& graph, int start, int end) {
    set<pair<int, int>> pq; // Priority queue to store {distance, node}
    unordered_map<int, int> distance; // To store the shortest distance to each node
    unordered_map<int, int> parent; // To store the parent node in the shortest path

    for (const auto& node : graph.nodes) {
        distance[node.first] = INF;
        parent[node.first] = -1;
    }

    distance[start] = 0;
    pq.insert({0, start});

    while (!pq.empty()) {
        int current = pq.begin()->second;
        pq.erase(pq.begin());

        for (const auto& neighbor : graph.nodes.at(current)) {
            int adjNode = neighbor.first;
            int weight = neighbor.second;

            if (distance[current] + weight < distance[adjNode]) {
                pq.erase({distance[adjNode], adjNode});
                distance[adjNode] = distance[current] + weight;
                parent[adjNode] = current;
                pq.insert({distance[adjNode], adjNode});
            }
        }
    }

    if (distance[end] == INF) {
        cout << "No path exists between the nodes.\n";
    } else {
        cout << "Shortest distance from node " << start << " to node " << end << " is: " << distance[end] << endl;
        cout << "Shortest path: ";
        printPath(parent, end);
        cout << endl;
    }
}

void printPath(const unordered_map<int, int>& parent, int node) {
    if (node == -1) return;
    printPath(parent, parent.at(node));
    cout << node << " ";
}

int main() {
    Graph graph;

    // You can customize the graph by adding edges here
    // Example:
    // graph.addEdge(1, 2, 5);
    // graph.addEdge(2, 3, 2);
    // ...

    int start, end;
    cout << "Enter the starting node: ";
    cin >> start;
    cout << "Enter the ending node: ";
    cin >> end;

    dijkstra(graph, start, end);

    return 0;
}
