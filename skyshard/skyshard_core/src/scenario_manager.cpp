// skyshard_core/src/scenario_manager.cpp

#include "rclcpp/rclcpp.hpp"
#include "skyshard_msgs/srv/update_graph.hpp"
#include <chrono>
#include <memory> // std::make_shared için eklendi

using namespace std;
using namespace std::chrono_literals;

using UpdateGraph = skyshard_msgs::srv::UpdateGraph;

class ScenarioManagerNode : public rclcpp::Node {
public:
    ScenarioManagerNode() : Node("scenario_manager") {
        RCLCPP_INFO(this->get_logger(), "Scenario Manager Node baslatiliyor.");

        // Parametre Tanımları
        this->declare_parameter("main_drone_id", 1);
        this->declare_parameter("link_fail_u", 2);
        this->declare_parameter("link_fail_v", 3);

        main_drone_id_ = this->get_parameter("main_drone_id").as_int();
        link_fail_u_ = this->get_parameter("link_fail_u").as_int();
        link_fail_v_ = this->get_parameter("link_fail_v").as_int();
        
        client_ = this->create_client<UpdateGraph>("/skyshard/update_graph");
        
        // 10 saniye sonra senaryolar başlasın
        timer_ = this->create_wall_timer(
            10s, bind(&ScenarioManagerNode::run_scenarios, this));
            
        current_scenario_ = 0;
    }

private:
    rclcpp::Client<UpdateGraph>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int current_scenario_;
    int main_drone_id_;
    int link_fail_u_;
    int link_fail_v_;

    bool call_update_graph_service(int scenario_type, const vector<int32_t>& failed_drone_ids = {}, const vector<int32_t>& failed_link_pair = {}) {
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "UpdateGraph servisi hazir degil.");
            return false;
        }

        // Düzeltme 3: std::make_shared kullanıldı
        auto request = std::make_shared<UpdateGraph::Request>();
        request->scenario_type = scenario_type;
        request->failed_drone_ids = failed_drone_ids;
        request->failed_link_pair = failed_link_pair;

        auto result = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Servis cagrisi basarisiz.");
            return false;
        }
        
        if (result.get()->success) {
            RCLCPP_WARN(this->get_logger(), "Senaryo Basarili: %s", result.get()->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Senaryo Basarisiz: %s", result.get()->message.c_str());
            return false;
        }
        
        // Fonksiyonun sonunda return false eklenerek derleme uyarısı/hatası önlenir.
        return false;
    }

    void run_scenarios() {
        if (current_scenario_ == 0) {
            // S1: Main Drone Fail (Ana drone düşmesi)
            RCLCPP_ERROR(this->get_logger(), "--- SENARYO 1: ANA DRONE DUSMESI (ID: %d) ---", main_drone_id_);
            call_update_graph_service(1, {main_drone_id_});
            current_scenario_ = 1;
            timer_->reset(); // 10s bekle
        } 
        else if (current_scenario_ == 1) {
            // S2: Link Fail (Belirli linkin kopması)
            RCLCPP_ERROR(this->get_logger(), "--- SENARYO 2: LINK KOPMASI (Link: %d-%d) ---", link_fail_u_, link_fail_v_);
            call_update_graph_service(2, {}, {link_fail_u_, link_fail_v_});
            current_scenario_ = 2;
            timer_->reset(); // 10s bekle
        } 
        else if (current_scenario_ == 2) {
             // S3: Route Recomputation (Mesaj aktarma senaryosu)
            RCLCPP_WARN(this->get_logger(), "--- SENARYO 3: ZORLA ROTA YENIDEN HESAPLAMA ---");
            call_update_graph_service(3);
            current_scenario_ = 3;
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "--- TUM SENARYOLAR TAMAMLANDI. ---");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioManagerNode>());
    rclcpp::shutdown();
    return 0;
}
