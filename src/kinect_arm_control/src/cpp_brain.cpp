#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm> // std::clamp için
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class CppBrainNode : public rclcpp::Node {
public:
  CppBrainNode() : Node("robot_brain_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})), 
    robot_ready_(false), is_moving_(false), last_stamp_(0) {
    
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/camera/target_coords", 10, std::bind(&CppBrainNode::topic_callback, this, _1));
    
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&CppBrainNode::joint_state_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "🧠 C++ Beyin (Geometrik IK + Limit Koruma) Baslatiliyor...");
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_stamp_ = msg->header.stamp.sec;
    if (!robot_ready_ && last_stamp_ > 0) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        init_moveit();
        robot_ready_ = true;
    }
  }

  void init_moveit() {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "my_arm");
    
    // CHOMP KULLANMAYA DEVAM EDIYORUZ
    move_group_->setPlanningPipelineId("chomp"); 
    
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setPlanningTime(1.0);
    
    // CHOMP bazen başlangıç durumunu "çarpışmada" sanabilir, bunu düzeltmek için:
    move_group_->setStartStateToCurrentState();
    
    move_group_->startStateMonitor(2.0);
    RCLCPP_INFO(this->get_logger(), "🚀 MoveIt Hazir! (CHOMP Aktif)");
  }

  // --- GELİŞMİŞ IK ÇÖZÜCÜ (LİMİT KONTROLLÜ) ---
  bool solve_ik(double x, double y, double z, std::vector<double>& angles) {
      const double L1 = 0.215;
      const double L2 = 0.230;
      const double L3 = 0.200;

      // URDF LİMİTLERİ (Radyan cinsinden)
      // Joint 1: -3.14 ile 3.14 arası
      // Joint 2: 0.0 ile 1.57 arası
      // Joint 3: 0.0 ile 2.09 arası
      const double J2_MIN = 0.0, J2_MAX = 1.57;
      const double J3_MIN = 0.0, J3_MAX = 2.09;

      try {
          // 1. Taban Açısı (Theta 1)
          double theta1 = std::atan2(y, x);

          // 2. Geometrik Hesaplamalar
          double r = std::sqrt(x*x + y*y);
          double z_offset = z - L1;
          double D = std::sqrt(r*r + z_offset*z_offset);
          double max_reach = L2 + L3;

          if (D > max_reach) {
              D = max_reach - 0.001; // Tam sınıra dayanmaması için ufak pay
          }

          // Kosinüs Teoremi (Dirsek - Joint 3)
          double cos_elbow = (L2*L2 + L3*L3 - D*D) / (2 * L2 * L3);
          cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);
          double theta3 = M_PI - std::acos(cos_elbow);

          // Omuz Açısı (Omuz - Joint 2)
          double beta = std::atan2(z_offset, r);
          double cos_alpha = (L2*L2 + D*D - L3*L3) / (2 * L2 * D);
          cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
          double theta2 = (M_PI / 2.0) - (beta + std::acos(cos_alpha));

          // --- KRİTİK NOKTA: AÇILARI URDF LİMİTLERİNE SIKIŞTIR ---
          
          // Joint 2 Limitleme
          if (theta2 > J2_MAX) theta2 = J2_MAX;
          if (theta2 < J2_MIN) theta2 = J2_MIN;

          // Joint 3 Limitleme
          if (theta3 > J3_MAX) theta3 = J3_MAX;
          if (theta3 < J3_MIN) theta3 = J3_MIN;

          // Sonuçları Kaydet
          angles.clear();
          angles.push_back(theta1);
          angles.push_back(theta2);
          angles.push_back(theta3);
          
          return true;
      } catch (...) {
          return false;
      }
  }

private:
  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!robot_ready_ || !move_group_ || is_moving_) return;
    
    // Time Sync Koruması
    if(!move_group_->getCurrentState(1.0)) return; 

    is_moving_ = true;

    std::vector<double> target_joints;
    if (solve_ik(msg->x, msg->y, msg->z, target_joints)) {
        
        RCLCPP_INFO(this->get_logger(), "📐 IK (Limitli): J1:%.2f J2:%.2f J3:%.2f", 
            target_joints[0], target_joints[1], target_joints[2]);

        move_group_->setJointValueTarget(target_joints);
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ CHOMP Onayladi! Gidiliyor...");
            move_group_->execute(my_plan);
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ CHOMP Plani Reddedildi (Limitler zorlanmis olabilir)");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ IK Hesaplama Hatasi");
    }
    
    unlock_timer_ = this->create_wall_timer(2000ms, [this]() {
        this->is_moving_ = false;
        this->unlock_timer_->cancel();
    });
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr unlock_timer_;
  bool robot_ready_, is_moving_;
  int32_t last_stamp_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppBrainNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
