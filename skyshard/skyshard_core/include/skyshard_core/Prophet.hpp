// skyshard_core/include/skyshard_core/Prophet.hpp

#ifndef SKYSHARD_PROPHET_HPP
#define SKYSHARD_PROPHET_HPP

#include "Graph.hpp"
#include "Dijkstra.hpp"
#include <queue>
#include <numeric>

using namespace std;

namespace skyshard {

struct ScoreInfo {
    int32_t node;
    double score;
    int32_t capacity;

    bool operator<(const ScoreInfo& other) const {
        if (abs(score - other.score) > EPS) {
            return score < other.score;
        }
        return node > other.node;
    }
};

struct AssignmentPQElement {
    double neg_score;
    int32_t node;
    int32_t cap;

    bool operator>(const AssignmentPQElement& other) const {
        if (abs(neg_score - other.neg_score) > EPS) {
            return neg_score > other.neg_score;
        }
        return node > other.node;
    }
};

// Basitleştirilmiş Prophet Benzeri Skor Hesaplama
double prophet_like_score(const Graph& graph, const map<int32_t, PathInfo>& paths, int32_t src, int32_t target) {
    if (src == target) return 1.0;
    
    if (paths.find(target) == paths.end() || paths.at(target).path.empty() || paths.at(target).latency_sum == INF_D) {
        return 0.0;
    }

    const auto& path = paths.at(target).path;
    
    double prod = 1.0;
    for (int32_t pid : path) {
        prod *= graph.nodes.at(pid).reliability;
    }

    double decay_factor = 0.9;
    int32_t hops = max((int32_t)1, (int32_t)path.size() - 1);
    
    return prod * pow(decay_factor, hops - 1);
}

// Belirli bir hop limitinde ulaşılabilir düğümleri bulur
vector<int32_t> get_reachable_nodes_bfs(const Graph& graph, int32_t leader, int32_t hop_limit) {
    vector<int32_t> reachable;
    vector<int32_t> dist(graph.N + 1, -1);
    queue<int32_t> q;

    dist[leader] = 0;
    q.push(leader);
    
    while (!q.empty()) {
        int32_t u = q.front();
        q.pop();
        
        if (!graph.nodes[u].is_active) continue;

        if (u != leader && dist[u] <= hop_limit) {
            reachable.push_back(u);
        }
        
        for (const auto& e : graph.adj[u]) {
            int32_t v = e.to;
            
            if (!graph.nodes[v].is_active) continue;

            if (dist[v] == -1) {
                dist[v] = dist[u] + 1;
                if (dist[v] <= hop_limit) q.push(v);
            }
        }
    }
    return reachable;
}

// Çok kriterli (resource, network, prophet) skoru hesaplar
vector<ScoreInfo> calculate_combined_scores(
    const Graph& graph, 
    const map<int32_t, PathInfo>& paths, 
    int32_t leader, 
    const vector<int32_t>& reachable, 
    int32_t shard_size,
    const string& routing_method
) {
    int32_t max_storage = 0;
    double max_bw = 0.0;
    double max_cpu = 0.0;
    
    for (int32_t v : reachable) {
        if (!graph.nodes.at(v).is_active) continue;
        max_storage = max(max_storage, graph.nodes.at(v).storage_free_mb);
        max_bw = max(max_bw, graph.nodes.at(v).node_bw);
        max_cpu = max(max_cpu, graph.nodes.at(v).cpu_units);
    }
    if (max_storage == 0) max_storage = 1;
    if (max_bw < EPS) max_bw = 1.0;
    if (max_cpu < EPS) max_cpu = 1.0;

    map<int32_t, double> transfer_time;
    double max_tt = 0.0;
    for (int32_t v : reachable) {
        if (!graph.nodes.at(v).is_active) continue;

        const auto& p_info = paths.at(v);
        
        if (p_info.path.empty() || p_info.latency_sum == INF_D) {
            transfer_time[v] = INF_D;
        } else {
            double latency_seconds = p_info.latency_sum / 1000.0;
            double bw = p_info.bottleneck_bw;
            if (bw < EPS) bw = 0.000001; 
            
            double t = (double)shard_size / bw + latency_seconds;
            transfer_time[v] = t;
            if (t != INF_D && t > max_tt) max_tt = t;
        }
    }
    if (max_tt < EPS) max_tt = 1.0;

    map<int32_t, double> prophet_scores;
    double max_p = 0.0;

    if (routing_method == "prophet") {
        for (int32_t v : reachable) {
            if (!graph.nodes.at(v).is_active) continue;
            prophet_scores[v] = prophet_like_score(graph, paths, leader, v);
            max_p = max(max_p, prophet_scores[v]);
        }
    }
    if (max_p < EPS) max_p = 1.0; 
    
    // Ağırlıklar: rel=0.4, bw=0.25, storage=0.15, tt_penalty=0.15, cpu=0.05
    const double alpha = 0.4, beta = 0.25, gamma = 0.15, delta = 0.15, eps_w = 0.05;
    const double prophet_weight = 0.3;
    const double resource_weight = 1.0 - prophet_weight;

    vector<ScoreInfo> scores;
    for (int32_t v : reachable) {
        if (!graph.nodes.at(v).is_active) continue;

        double rel = graph.nodes.at(v).reliability;
        double bw_norm = graph.nodes.at(v).node_bw / max_bw;
        double storage_norm = (double)graph.nodes.at(v).storage_free_mb / max_storage;
        double cpu_norm = graph.nodes.at(v).cpu_units / max_cpu;
        
        double tt = transfer_time.at(v);
        double tt_norm = (tt == INF_D) ? 1.0 : tt / max_tt; 

        double resource_score = alpha * rel + beta * bw_norm + gamma * storage_norm - delta * tt_norm + eps_w * cpu_norm;
        
        double final_score = resource_score;
        if (routing_method == "prophet") {
            double prophet_norm = prophet_scores.at(v) / max_p;
            final_score = resource_score * resource_weight + prophet_weight * prophet_norm;
        }

        int32_t cap = graph.nodes.at(v).storage_free_mb / shard_size;
        
        scores.push_back({v, final_score, max(0, cap)});
    }

    return scores;
}

// Greedy Shard Ataması
vector<int32_t> greedy_shard_assignment(vector<ScoreInfo> scores, int32_t n_shards) {
    
    sort(scores.begin(), scores.end(), [](const ScoreInfo& a, const ScoreInfo& b) {
        if (abs(a.score - b.score) > EPS) {
            return a.score > b.score;
        }
        return a.node < b.node;
    });

    vector<int32_t> shard_to_node(n_shards, -1);
    int32_t assigned = 0;
    
    int32_t max_unique_assignments = min((int32_t)scores.size(), n_shards);
    
    for (int32_t i = 0; i < max_unique_assignments; ++i) {
        if (assigned >= n_shards) break;
        if (scores[i].capacity > 0) {
            shard_to_node[assigned] = scores[i].node;
            scores[i].capacity--;
            assigned++;
        }
    }
    
    priority_queue<AssignmentPQElement, vector<AssignmentPQElement>, greater<AssignmentPQElement>> pq;
    for (const auto& s : scores) {
        if (s.capacity > 0) {
            pq.push({-s.score, s.node, s.capacity}); 
        }
    }

    while (assigned < n_shards && !pq.empty()) {
        AssignmentPQElement current = pq.top();
        pq.pop();
        
        double neg_score = current.neg_score;
        int32_t node = current.node;
        int32_t cap = current.cap;
        
        shard_to_node[assigned] = node;
        assigned++;
        cap--;
        
        if (cap > 0) {
            pq.push({neg_score, node, cap});
        }
    }
    
    return shard_to_node;
}

} // namespace skyshard

#endif // SKYSHARD_PROPHET_HPP
