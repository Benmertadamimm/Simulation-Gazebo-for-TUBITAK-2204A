// skyshard_core/include/skyshard_core/Dijkstra.hpp

#ifndef SKYSHARD_DIJKSTRA_HPP
#define SKYSHARD_DIJKSTRA_HPP

#include "Graph.hpp"
#include <queue>

using namespace std;

namespace skyshard {

// En düşük gecikme ve en yüksek bottleneck BW yolunu bulan Dijkstra
map<int32_t, PathInfo> dijkstra_latency_and_bottleneck(const Graph& graph, int32_t src) {
    
    const int32_t n = graph.N;
    const auto& adj = graph.adj;
    
    vector<double> dist(n + 1, INF_D);
    vector<double> bottleneck(n + 1, 0.0);
    vector<int32_t> parent(n + 1, -1);
    
    priority_queue<PQElement, vector<PQElement>, greater<PQElement>> pq;

    dist[src] = 0.0;
    bottleneck[src] = INF_D;
    pq.push({0.0, -bottleneck[src], src});

    while (!pq.empty()) {
        PQElement current = pq.top();
        pq.pop();
        
        double d = current.latency;
        int32_t u = current.node;

        if (d > dist[u] + EPS || !graph.nodes[u].is_active) {
            continue;
        }
        
        for (const auto& e : adj[u]) {
            int32_t v = e.to;
            if (!graph.nodes[v].is_active) continue;

            double nd = d + e.latency;
            double nb = min(bottleneck[u], e.bw);

            // Relax: latency (min) ve bottleneck (max)
            if (nd < dist[v] - EPS || (abs(nd - dist[v]) < EPS && nb > bottleneck[v] + EPS)) {
                dist[v] = nd;
                bottleneck[v] = nb;
                parent[v] = u;
                pq.push({dist[v], -bottleneck[v], v});
            }
        }
    }

    map<int32_t, PathInfo> paths;
    for (int32_t t = 1; t <= n; ++t) {
        if (dist[t] == INF_D || !graph.nodes[t].is_active) {
            paths[t] = {INF_D, 0.0, {}};
            continue;
        }
        
        vector<int32_t> path;
        int32_t cur = t;
        
        while (cur != -1 && cur != 0) {
            path.push_back(cur);
            if (cur == src) break;
            cur = parent[cur];
            if (cur == -1 && t != src) break;
        }
        
        if (path.empty() || path.back() != src) {
             paths[t] = {INF_D, 0.0, {}};
             continue;
        }

        reverse(path.begin(), path.end());

        double final_bottleneck = bottleneck[t];
        if (final_bottleneck == INF_D) final_bottleneck = 0.0;
        
        paths[t] = {dist[t], final_bottleneck, path};
    }
    return paths;
}

} // namespace skyshard

#endif // SKYSHARD_DIJKSTRA_HPP
