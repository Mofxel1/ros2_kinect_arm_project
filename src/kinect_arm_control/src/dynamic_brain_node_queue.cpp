#include <memory>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <urdf_model/model.h>

using std::placeholders::_1;

class CppBrainNodeQueue : public rclcpp::Node {
public:
  // =======================================================
  // MANUEL KALİBRASYON AYARLARI
  // Eski dynamic_brain_node.cpp ile aynı tutuldu.
  // =======================================================
  double OMUZ_PITCH_OFFSET = 0.345;
  double DIRSEK_SARKMA_ACISI = 1.510;
  // =======================================================

  CppBrainNodeQueue()
  : Node(
      "dynamic_brain_node_queue",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
      })
    ),
    robot_ready_(false),
    is_moving_(false),
    last_stamp_(0),
    L1_(0.183),
    L2_(0.220),
    L3_(0.200)
  {
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/camera/target_coords",
      10,
      std::bind(&CppBrainNodeQueue::topic_callback, this, _1)
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&CppBrainNodeQueue::joint_state_callback, this, _1)
    );

    motion_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/arm_motion_done",
      10
    );

    RCLCPP_INFO(
      this->get_logger(),
      "🧠 Queue Brain baslatildi. /camera/target_coords dinleniyor, /arm_motion_done yayinlanacak."
    );
  }

private:
  void publish_done(bool success) {
    std_msgs::msg::Bool msg;
    msg.data = success;
    motion_done_pub_->publish(msg);
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
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      "my_arm"
    );

    move_group_->setPlanningPipelineId("chomp");
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setPlanningTime(2.0);
    move_group_->setStartStateToCurrentState();
    move_group_->startStateMonitor(2.0);

    extract_urdf_parameters();

    RCLCPP_INFO(this->get_logger(), "🚀 MoveIt hazir. Queue Brain aktif.");
  }

  void extract_urdf_parameters() {
    auto urdf_model = move_group_->getRobotModel()->getURDF();

    if (!urdf_model) {
      RCLCPP_WARN(this->get_logger(), "⚠️ URDF modeli alinamadi. Varsayilan uzunluklar kullanilacak.");
      return;
    }

    auto j1 = urdf_model->getJoint("joint1");
    auto j2 = urdf_model->getJoint("joint2");
    auto j3 = urdf_model->getJoint("tcp_joint");

    if (j1 && j2 && j3) {
      L1_ = std::abs(j1->parent_to_joint_origin_transform.position.z);

      L2_ = std::sqrt(
        std::pow(j2->parent_to_joint_origin_transform.position.x, 2) +
        std::pow(j2->parent_to_joint_origin_transform.position.y, 2) +
        std::pow(j2->parent_to_joint_origin_transform.position.z, 2)
      );

      L3_ = std::sqrt(
        std::pow(j3->parent_to_joint_origin_transform.position.x, 2) +
        std::pow(j3->parent_to_joint_origin_transform.position.y, 2) +
        std::pow(j3->parent_to_joint_origin_transform.position.z, 2)
      );

      RCLCPP_INFO(
        this->get_logger(),
        "📏 URDF uzunluklari okundu -> L1: %.3f | L2: %.3f | L3: %.3f",
        L1_,
        L2_,
        L3_
      );
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ joint1 / joint2 / tcp_joint bulunamadi. Varsayilan uzunluklar kullanilacak.");
    }
  }

  bool solve_ik(double target_x, double target_y, double target_z, std::vector<double>& angles) {
    try {
      // Eski dynamic_brain_node.cpp ile aynı eksen dönüşümü
      double x = target_y;
      double y = target_x;
      double z = target_z;

      double theta0 = std::atan2(y, x);

      double r = std::sqrt(x * x + y * y);
      double z_offset = z - L1_;
      double D = std::sqrt(r * r + z_offset * z_offset);

      double max_reach = L2_ + L3_ - 0.005;
      double min_reach = std::abs(L2_ - L3_) + 0.005;

      if (D > max_reach) {
        RCLCPP_WARN(
          this->get_logger(),
          "❌ Hedef cok uzak. D=%.3f max=%.3f | x=%.2f y=%.2f z=%.2f",
          D,
          max_reach,
          target_x,
          target_y,
          target_z
        );
        return false;
      }

      if (D < min_reach) {
        RCLCPP_WARN(
          this->get_logger(),
          "❌ Hedef cok yakin. D=%.3f min=%.3f | x=%.2f y=%.2f z=%.2f",
          D,
          min_reach,
          target_x,
          target_y,
          target_z
        );
        return false;
      }

      double cos_gamma = (L2_ * L2_ + L3_ * L3_ - D * D) / (2.0 * L2_ * L3_);
      if (cos_gamma < -1.0 || cos_gamma > 1.0) {
        RCLCPP_WARN(this->get_logger(), "❌ IK gamma cozumu yok.");
        return false;
      }

      cos_gamma = std::clamp(cos_gamma, -1.0, 1.0);
      double gamma = std::acos(cos_gamma);

      double cos_alpha = (L2_ * L2_ + D * D - L3_ * L3_) / (2.0 * L2_ * D);
      if (cos_alpha < -1.0 || cos_alpha > 1.0) {
        RCLCPP_WARN(this->get_logger(), "❌ IK alpha cozumu yok.");
        return false;
      }

      cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
      double alpha = std::acos(cos_alpha);

      double beta = std::atan2(z_offset, r);

      double raw_theta1 = beta + alpha;
      double raw_theta2 = DIRSEK_SARKMA_ACISI - gamma;

      double theta1 = raw_theta1 - OMUZ_PITCH_OFFSET;
      double theta2 = raw_theta2;

      RCLCPP_INFO(
        this->get_logger(),
        "🔍 IK -> joint0: %.2f | joint1: %.2f | joint2: %.2f",
        theta0,
        theta1,
        theta2
      );

      angles.clear();
      angles.push_back(theta0);
      angles.push_back(theta1);
      angles.push_back(theta2);

      return true;

    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "❌ IK sirasinda bilinmeyen hata.");
      return false;
    }
  }

  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (!robot_ready_ || !move_group_) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Robot henuz hazir degil. Hedef reddedildi.");
      publish_done(false);
      return;
    }

    if (is_moving_) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Robot zaten hareket ediyor. Yeni hedef reddedildi.");
      publish_done(false);
      return;
    }

    if (!move_group_->getCurrentState(1.0)) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Current state alinamadi.");
      publish_done(false);
      return;
    }

    is_moving_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "📍 Yeni hedef alindi -> x=%.3f y=%.3f z=%.3f",
      msg->x,
      msg->y,
      msg->z
    );

    std::vector<double> target_joints;

    bool ik_ok = solve_ik(msg->x, msg->y, msg->z, target_joints);

    if (!ik_ok) {
      is_moving_ = false;
      publish_done(false);
      return;
    }

    move_group_->setStartStateToCurrentState();

    bool target_ok = move_group_->setJointValueTarget(target_joints);

    if (!target_ok) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Joint hedefleri MoveIt tarafindan reddedildi.");
      is_moving_ = false;
      publish_done(false);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "⚠️ CHOMP plan uretmedi.");
      is_moving_ = false;
      publish_done(false);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "✅ Plan bulundu. Execute basliyor...");

    auto execute_result = move_group_->execute(plan);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "✅ Execute tamamlandi.");
      publish_done(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️ Execute basarisiz.");
      publish_done(false);
    }

    is_moving_ = false;
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_done_pub_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  bool robot_ready_;
  bool is_moving_;
  int32_t last_stamp_;

  double L1_;
  double L2_;
  double L3_;
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppBrainNodeQueue>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}