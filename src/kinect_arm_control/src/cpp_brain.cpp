#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>
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
    
    RCLCPP_INFO(this->get_logger(), "🧠 C++ Beyin: Eksen, 90-Derece ve SolidWorks Kalibrasyonlu IK Baslatiliyor...");
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
    move_group_->setPlanningPipelineId("chomp"); 
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setPlanningTime(1.0);
    move_group_->setStartStateToCurrentState();
    move_group_->startStateMonitor(2.0);
    RCLCPP_INFO(this->get_logger(), "🚀 MoveIt Hazir!");
  }

  // --- EKSEN DÜZELTMELİ, 90 DERECE OFSETLİ VE KALİBRASYONLU IK ÇÖZÜCÜ ---
  bool solve_ik(double target_x, double target_y, double target_z, std::vector<double>& angles) {
      const double L1 = 0.183; 
      const double L2 = 0.220; 
      const double L3 = 0.200; 

      // Robotunun gerçek fiziksel motor limitleri (Radyan)
      // Eger ofsetten dolayi negatiflere inmesi gerekirse J2_MIN'i genisletebiliriz.
      const double J2_MIN = -0.50,  J2_MAX = 0.80; 
      const double J3_MIN = -1.0, J3_MAX = 0.10;

      try {
          // ========================================================
          // 1. EKSEN DÜZELTMESİ (SolidWorks Yön Ofseti)
          // ========================================================
          double x = target_y; 
          double y = target_x; 
          double z = target_z;

          double theta1 = std::atan2(y, x);

          // 2. Mesafeler ve Hipotenüs (D)
          double r = std::sqrt(x*x + y*y);
          double z_offset = z - L1;
          double D = std::sqrt(r*r + z_offset*z_offset);
          
          if (D > (L2 + L3)) {
              D = L2 + L3 - 0.001; 
          }

          // 3. İç Açılar (Gama: Dirsek iç açısı, Alfa: Omuz iç açısı)
          double cos_gamma = std::clamp((L2*L2 + L3*L3 - D*D) / (2 * L2 * L3), -1.0, 1.0);
          double gamma = std::acos(cos_gamma);

          double cos_alpha = std::clamp((L2*L2 + D*D - L3*L3) / (2 * L2 * D), -1.0, 1.0);
          double alpha = std::acos(cos_alpha);
          double beta = std::atan2(z_offset, r);

          // ========================================================
          // 4. EFSANEVİ EŞLEŞTİRME VE SOLIDWORKS KALİBRASYONU
          // ========================================================
          
          // Saf Matematiksel Açılar
          double raw_theta2 = beta + alpha; 
          double raw_theta3 = (M_PI / 2.0) - gamma; 

          // URDF İÇİNDEKİ GİZLİ AÇILARI (OFFSET) SİLME
          // Robotun hep havada kalmasına sebep olan ~20.7 derecelik kaymayı siliyoruz
          double J2_OFFSET = 0.362; 
          double J3_OFFSET = 0.000; 

          // Kalibre Edilmiş Nihai Açılar
          double theta2 = raw_theta2 - J2_OFFSET; 
          double theta3 = raw_theta3 - J3_OFFSET; 

          RCLCPP_INFO(this->get_logger(), "🔍 Kalibre IK -> J1: %.2f | J2: %.2f | J3: %.2f", theta1, theta2, theta3);

          // --- MOTORLARI YAKMAMAK İÇİN LİMİTLERE SIKIŞTIR ---
          if (theta2 > J2_MAX) theta2 = J2_MAX;
          if (theta2 < J2_MIN) theta2 = J2_MIN;

          if (theta3 > J3_MAX) theta3 = J3_MAX;
          if (theta3 < J3_MIN) theta3 = J3_MIN;

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
    if(!move_group_->getCurrentState(1.0)) return; 

    is_moving_ = true;

    std::vector<double> target_joints;
    if (solve_ik(msg->x, msg->y, msg->z, target_joints)) {
        
        move_group_->setJointValueTarget(target_joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ CHOMP Onayladi! Hedefe Gidiliyor...");
            move_group_->execute(my_plan);
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ CHOMP Plani Reddedildi (Limit Disi Uzanma Testi)");
        }
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
