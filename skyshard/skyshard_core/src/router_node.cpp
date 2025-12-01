// skyshard_core/src/router_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "skyshard_msgs/msg/drone_state.hpp"
#include "skyshard_msgs/srv/update_graph.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "skyshard_core/Graph.hpp"
#include "skyshard_core/Dijkstra.hpp"
#include "skyshard_core/Prophet.hpp"

#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include <chrono>
#include <memory> // std::make_unique ve std::make_shared için eklendi
#include <set> // unique_targets için eklendi

using namespace std;
using namespace std::chrono_literals;

using DroneState = skyshard_msgs::msg::DroneState;
using UpdateGraph = skyshard_msgs::srv::UpdateGraph;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using namespace skyshard;

class RouterNode : public rclcpp::Node {
public:
    RouterNode() : Node("router_node") {
        RCLCPP_INFO(this->get_logger(), "Router Node baslatiliyor...");
        
        // Parametre Tanımları
        this->declare_parameter("drone_count", 0);
        this->declare_parameter("initial_links", vector<string>{});
        this->declare_parameter("node_resources", vector<string>{});
        this->declare_parameter("leader_id", 1);
        this->declare_parameter("data_size_mb", 100);
        this->declare_parameter("redundancy_k", 3);
        this->declare_parameter("num_shards", 10);
        this->declare_parameter("routing_method", "dijkstra");

        // Parametre Değerleri
        int drone_count = this->get_parameter("drone_count").as_int();
        leader_id_ = this->get_parameter("leader_id").as_int();
        data_size_mb_ = this->get_parameter("data_size_mb").as_int();
        redundancy_k_ = this->get_parameter("redundancy_k").as_int();
        num_shards_ = this->get_parameter("num_shards").as_int();
        routing_method_ = this->get_parameter("routing_method").as_string();
        
        if (drone_count <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Drone sayisi gecersiz.");
            return;
        }

        shard_size_ = (data_size_mb_ + redundancy_k_ - 1) / redundancy_k_;
        
        // Düzeltme 1: std::make_unique kullanıldı
        graph_ = std::make_unique<Graph>(drone_count); 
        
        load_initial_graph();
        
        // ROS2 Iletisim Kurulumu
        marker_pub_ = this->create_publisher<MarkerArray>("/skyshard/markers", 10);
        
        drone_state_sub_ = this->create_subscription<DroneState>(
            "/skyshard/drone_state", 10, 
            bind(&RouterNode::drone_state_callback, this, placeholders::_1));
            
        graph_update_service_ = this->create_service<UpdateGraph>(
            "/skyshard/update_graph", 
            bind(&RouterNode::handle_graph_update, this, placeholders::_1, placeholders::_2));
            
        // 1 saniyede bir periyodik hesaplama
        timer_ = this->create_wall_timer(
            1000ms, bind(&RouterNode::periodic_calculation, this));

        RCLCPP_INFO(this->get_logger(), "Router hazir. %d drone, Shard Boyutu: %d MB. Method: %s.", drone_count, shard_size_, routing_method_.c_str());
        
        // Ilk hesaplama
        periodic_calculation();
    }

private:
    unique_ptr<Graph> graph_;
    int32_t leader_id_;
    int32_t data_size_mb_;
    int32_t redundancy_k_;
    int32_t num_shards_;
    int32_t shard_size_;
    string routing_method_;

    rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<DroneState>::SharedPtr drone_state_sub_;
    rclcpp::Service<UpdateGraph>::SharedPtr graph_update_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    map<int32_t, PathInfo> current_paths_;
    vector<int32_t> current_shard_assignment_;


    void load_initial_graph() {
        // Node Kaynak Verileri (ID Storage BW Rel CPU)
        auto node_resources = this->get_parameter("node_resources").as_string_array();
        for (const auto& resource_str : node_resources) {
            stringstream ss(resource_str);
            int id, storage;
            double bw, rel, cpu = 1.0;
            if (ss >> id >> storage >> bw >> rel) {
                if (!(ss >> cpu)) { /* CPU optional */ }
                graph_->add_node_initial_data(id, storage, bw, rel, cpu);
            }
        }
        
        // Link Verileri (U V BW Latency)
        auto initial_links = this->get_parameter("initial_links").as_string_array();
        for (const auto& link_str : initial_links) {
            stringstream ss(link_str);
            int u, v;
            double bw, latency;
            if (ss >> u >> v >> bw >> latency) {
                graph_->add_edge(u, v, bw, latency);
            }
        }
    }

    // Hareketlilik verilerini günceller
    void drone_state_callback(const DroneState::SharedPtr msg) {
        graph_->update_node_mobility(msg->id, msg->position, msg->velocity);
    }
    
    // Senaryo güncellemelerini işler
    void handle_graph_update(
        const shared_ptr<UpdateGraph::Request> request,
        shared_ptr<UpdateGraph::Response> response) {
        
        int scenario = request->scenario_type;
        response->success = false;
        response->message = "Islem basarisiz.";

        if (scenario == 1) { // Main Drone Fail
            if (!request->failed_drone_ids.empty()) {
                int failed_id = request->failed_drone_ids.front();
                graph_->set_node_active_status(failed_id, false);
                RCLCPP_ERROR(this->get_logger(), "Drone ID %d devre disi birakildi.", failed_id);
            }
        } else if (scenario == 2) { // Link Fail
            if (request->failed_link_pair.size() == 2) {
                int u = request->failed_link_pair[0];
                int v = request->failed_link_pair[1];
                graph_->remove_edge(u, v);
                RCLCPP_ERROR(this->get_logger(), "Link %d-%d koparildi.", u, v);
            }
        }

        if (calculate_and_assign_routes()) {
            response->success = true;
            response->message = "Graph guncellendi ve rotalar yeniden hesaplandi.";
        }
    }

    // Periyodik hesaplama ve yayınlama
    void periodic_calculation() {
        if (graph_->N > 0) {
            calculate_and_assign_routes();
        }
    }

    // Ana Rota Hesaplama ve Atama
    bool calculate_and_assign_routes() {
        if (!graph_) return false;
        
        // Ulaşılabilir düğümleri bul (hop limit 5)
        vector<int32_t> reachable = get_reachable_nodes_bfs(*graph_, leader_id_, 5);
        if (reachable.empty()) {
            RCLCPP_WARN(this->get_logger(), "Ulasilabilir hedef drone bulunamadi.");
            publish_markers({}, {});
            return false;
        }
        
        // Dijkstra Hesaplaması
        map<int32_t, PathInfo> paths = dijkstra_latency_and_bottleneck(*graph_, leader_id_);
        current_paths_ = paths;
        
        // Skorlama ve Atama
        vector<ScoreInfo> scores = calculate_combined_scores(
            *graph_, paths, leader_id_, reachable, shard_size_, routing_method_);

        current_shard_assignment_ = greedy_shard_assignment(scores, num_shards_);
        
        RCLCPP_INFO(this->get_logger(), 
            "Rota Guncellendi. Hedef drone sayisi: %d", (int)reachable.size());

        publish_markers(paths, current_shard_assignment_);
        return true;
    }

    // Görselleştirme Markerlarını Yayınla
    void publish_markers(const map<int32_t, PathInfo>& paths, const vector<int32_t>& assignment) {
        MarkerArray marker_array;
        rclcpp::Clock::SharedPtr clock = this->get_clock();
        int marker_id = 0;

        // A - Tüm Aktif Bağlantıları Çiz (Mavi)
        for (int u = 1; u <= graph_->N; ++u) {
            if (!graph_->nodes[u].is_active) continue;

            for (const auto& edge : graph_->adj[u]) {
                int v = edge.to;
                if (u >= v || !graph_->nodes[v].is_active) continue;
                
                Marker link_marker;
                link_marker.header.frame_id = "world";
                link_marker.header.stamp = clock->now();
                link_marker.ns = "links";
                link_marker.id = marker_id++;
                link_marker.type = Marker::LINE_LIST;
                link_marker.action = Marker::ADD;
                link_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
                link_marker.scale.x = 0.05;
                
                link_marker.color.a = 0.5;
                link_marker.color.r = 0.0;
                link_marker.color.g = 0.0;
                link_marker.color.b = 1.0; // Mavi Link
                
                link_marker.points.push_back(graph_->nodes[u].position);
                link_marker.points.push_back(graph_->nodes[v].position);
                
                marker_array.markers.push_back(link_marker);
            } // Edge döngüsü kapandı
        } // U döngüsü kapandı

        // B - Aktif Rotayı Çiz (Yeşil)
        if (!assignment.empty()) {
            set<int32_t> unique_targets;
            for (int32_t target_id : assignment) {
                if (unique_targets.size() >= 3) break;
                unique_targets.insert(target_id);
            }

            for (int32_t target_id : unique_targets) {
                if (paths.count(target_id) && !paths.at(target_id).path.empty()) {
                    const auto& path = paths.at(target_id).path;

                    Marker route_marker;
                    route_marker.header.frame_id = "world";
                    route_marker.header.stamp = clock->now();
                    route_marker.ns = "routes";
                    route_marker.id = marker_id++;
                    route_marker.type = Marker::LINE_STRIP;
                    route_marker.action = Marker::ADD;
                    route_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
                    route_marker.scale.x = 0.15;
                    
                    route_marker.color.a = 0.8;
                    route_marker.color.r = 0.0;
                    route_marker.color.g = 1.0; // Yeşil Rota
                    route_marker.color.b = 0.0;
                    
                    for (int32_t node_id : path) {
                        // Vector için sınır kontrolü kullanıldı
                        if (node_id > 0 && node_id <= graph_->N && graph_->nodes[node_id].is_active) {
                            route_marker.points.push_back(graph_->nodes[node_id].position);
                        }
                    }
                    
                    if (route_marker.points.size() > 1) {
                         marker_array.markers.push_back(route_marker);
                    }
                }
            }
        }
        
        marker_pub_->publish(marker_array);
    } // publish_markers fonksiyonu kapandı
}; // RouterNode sınıfı kapandı

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // std::make_shared eklendi
    rclcpp::spin(std::make_shared<RouterNode>());
    rclcpp::shutdown();
    return 0;
} // main fonksiyonu kapandı
