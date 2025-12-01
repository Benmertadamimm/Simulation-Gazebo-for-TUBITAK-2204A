// skyshard_core/include/skyshard_core/Graph.hpp

#ifndef SKYSHARD_GRAPH_HPP
#define SKYSHARD_GRAPH_HPP

#include <vector>
#include <map>
#include <limits>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std;

namespace skyshard {

const double INF_D = numeric_limits<double>::infinity();
const double EPS = 1e-9;

// Kenar Bilgisi
struct Edge {
    int32_t to;
    double bw;
    double latency;
};

// Düğüm (Drone) Bilgisi
struct Node {
    int32_t id;
    int32_t storage_free_mb;
    double node_bw;
    double reliability;
    double cpu_units;
    
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 velocity;
    
    bool is_active = true;
};

// Yol Hesaplama Bilgisi
struct PathInfo {
    double latency_sum;
    double bottleneck_bw;
    vector<int32_t> path;
};

// Dijkstra için Priority Queue Elemanı
struct PQElement {
    double latency;
    double neg_bottleneck;
    int32_t node;

    bool operator>(const PQElement& other) const {
        if (abs(latency - other.latency) > EPS) {
            return latency > other.latency;
        }
        return neg_bottleneck > other.neg_bottleneck;
    }
};

class Graph {
public:
    int32_t N;
    vector<Node> nodes;
    vector<vector<Edge>> adj;
    
    Graph(int32_t drone_count) : N(drone_count) {
        nodes.resize(N + 1);
        adj.resize(N + 1);
    }
    
    void add_node_initial_data(int32_t id, int32_t storage, double bw, double rel, double cpu) {
        if (id > 0 && id <= N) {
            nodes[id] = {id, storage, bw, rel, cpu, geometry_msgs::msg::Point(), geometry_msgs::msg::Vector3()};
        }
    }
    
    void add_edge(int32_t u, int32_t v, double bw, double latency) {
        if (u > 0 && u <= N && v > 0 && v <= N) {
            adj[u].push_back({v, bw, latency});
            adj[v].push_back({u, bw, latency});
        }
    }
    
    void remove_edge(int32_t u, int32_t v) {
        auto remove_link = [&](int32_t start, int32_t end) {
            auto& list = adj[start];
            list.erase(remove_if(list.begin(), list.end(), 
                [end](const Edge& e) { return e.to == end; }), list.end());
        };
        remove_link(u, v);
        remove_link(v, u);
    }

    void update_node_mobility(int32_t id, const geometry_msgs::msg::Point& pos, const geometry_msgs::msg::Vector3& vel) {
        if (id > 0 && id <= N && nodes[id].is_active) {
            nodes[id].position = pos;
            nodes[id].velocity = vel;
        }
    }

    void set_node_active_status(int32_t id, bool status) {
        if (id > 0 && id <= N) {
            nodes[id].is_active = status;
        }
    }
};

} // namespace skyshard

#endif // SKYSHARD_GRAPH_HPP
