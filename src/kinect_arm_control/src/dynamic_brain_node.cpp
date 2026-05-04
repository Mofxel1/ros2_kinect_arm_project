#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf_model/model.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class CppBrainNode : public rclcpp::Node {
public:
  // =======================================================
  // 🛠️ MANUEL KALİBRASYON VİDALARI (FINE-TUNING)
  // =======================================================
  // Kol çok aşağıda kalıyorsa OMUZ ofsetini KÜÇÜLT.
  // Kol çok geride (kısa) kalıyorsa DİRSEK açısını KÜÇÜLT.
  double OMUZ_PITCH_OFFSET = 0.345;  // Eski URDF değeri: 0.362
  double DIRSEK_SARKMA_ACISI = 1.510; // Eski varsayılan: 1.570
  // =======================================================

  CppBrainNode() : Node("dynamic_brain_node", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)})), 
    robot_ready_(false), is_moving_(false), last_stamp_(0),
    L1_(0.183), L2_(0.220), L3_(0.200) // Güvenlik varsayılanları
  {
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/camera/target_coords", 10, std::bind(&CppBrainNode::topic_callback, this, _1));
    
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&CppBrainNode::joint_state_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "🧠 C++ Beyin: YARI-DINAMIK (Manuel Kalibrasyonlu) IK Baslatiliyor...");
  }

private:
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
    
    extract_urdf_parameters();

    RCLCPP_INFO(this->get_logger(), "🚀 MoveIt ve Kalibre Edilmis Anatomi Hazir!");
  }

  void extract_urdf_parameters() {
      auto urdf_model = move_group_->getRobotModel()->getURDF();
      if (urdf_model) {
          auto j1 = urdf_model->getJoint("joint1");
          auto j2 = urdf_model->getJoint("joint2");
          auto j3 = urdf_model->getJoint("tcp_joint");

          if (j1 && j2 && j3) {
              // SADECE UZUNLUKLARI DİNAMİK OKUYORUZ (Pisagor 3D uzayda kusursuz çalışır)
              L1_ = std::abs(j1->parent_to_joint_origin_transform.position.z);
              L2_ = std::sqrt(std::pow(j2->parent_to_joint_origin_transform.position.x, 2) +
                              std::pow(j2->parent_to_joint_origin_transform.position.y, 2) +
                              std::pow(j2->parent_to_joint_origin_transform.position.z, 2));
              L3_ = std::sqrt(std::pow(j3->parent_to_joint_origin_transform.position.x, 2) +
                              std::pow(j3->parent_to_joint_origin_transform.position.y, 2) +
                              std::pow(j3->parent_to_joint_origin_transform.position.z, 2));

              RCLCPP_INFO(this->get_logger(), "📏 URDF UZUNLUKLARI OKUNDU -> L1:%.3f | L2:%.3f | L3:%.3f", L1_, L2_, L3_);
          } else {
              RCLCPP_WARN(this->get_logger(), "⚠️ URDF jointleri bulunamadi! Varsayilan degerler kullanilacak.");
          }
      } else {
          RCLCPP_WARN(this->get_logger(), "⚠️ URDF Modeli alinamadi! Varsayilan degerler kullanilacak.");
      }
  }

  bool solve_ik(double target_x, double target_y, double target_z, std::vector<double>& angles) {
      const double J2_MIN = -3.14, J2_MAX = 3.14; 
      const double J3_MIN = -3.14, J3_MAX = 3.14;

      try {
          double x = target_y; 
          double y = target_x; 
          double z = target_z;

          double theta1 = std::atan2(y, x);

          double r = std::sqrt(x*x + y*y);
          double z_offset = z - L1_; 
          double D = std::sqrt(r*r + z_offset*z_offset);
          
          if (D > (L2_ + L3_)) D = L2_ + L3_ - 0.001; 

          double cos_gamma = std::clamp((L2_*L2_ + L3_*L3_ - D*D) / (2 * L2_ * L3_), -1.0, 1.0);
          double gamma = std::acos(cos_gamma);

          double cos_alpha = std::clamp((L2_*L2_ + D*D - L3_*L3_) / (2 * L2_ * D), -1.0, 1.0);
          double alpha = std::acos(cos_alpha);
          double beta = std::atan2(z_offset, r);

          // KALİBRASYON VİDALARINI KULLANIYORUZ
          double raw_theta2 = beta + alpha; 
          double raw_theta3 = DIRSEK_SARKMA_ACISI - gamma; 

          double theta2 = raw_theta2 - OMUZ_PITCH_OFFSET; 
          double theta3 = raw_theta3; 

          RCLCPP_INFO(this->get_logger(), "🔍 Kalibre IK -> J1: %.2f | J2: %.2f | J3: %.2f", theta1, theta2, theta3);

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
            RCLCPP_WARN(this->get_logger(), "⚠️ CHOMP Plani Reddedildi");
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
  
  double L1_, L2_, L3_;
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
