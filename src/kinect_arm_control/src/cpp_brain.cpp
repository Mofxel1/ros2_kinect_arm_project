#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class CppBrainNode : public rclcpp::Node
{
public:
  CppBrainNode() : Node("robot_brain_node"), robot_ready_(false)
  {
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/camera/target_coords", 10, std::bind(&CppBrainNode::topic_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&CppBrainNode::joint_state_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "⏳ Robot verileri bekleniyor (Time Sync)...");
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (robot_ready_) return;
    if (msg->header.stamp.sec > 0) {
        RCLCPP_INFO(this->get_logger(), "✅ Güncel Veri Yakalandı! Zaman: %d.%d", 
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        init_moveit();
        robot_ready_ = true;
    }
  }

  void init_moveit()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "my_arm");
    
    // CHOMP Ayarları
    move_group_->setPlanningPipelineId("chomp");
    move_group_->setPlannerId(""); 
    
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setPlanningTime(5.0);
    move_group_->setGoalPositionTolerance(0.01);     // Konum hassasiyeti (1cm)
    move_group_->setGoalOrientationTolerance(3.14);  // Oryantasyon serbest (3.14 radyan)
    
    move_group_->startStateMonitor(2.0);

    RCLCPP_INFO(this->get_logger(), "🔧 MoveIt Başlatıldı: Pipeline = CHOMP");
  }

private:
  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (!robot_ready_ || !move_group_) return;

    RCLCPP_INFO(this->get_logger(), "🎯 Hedef Alindi: X:%.2f Y:%.2f Z:%.2f", msg->x, msg->y, msg->z);

    // 1. Başlangıç durumunu güncelle
    move_group_->setStartStateToCurrentState();
    
    // 2. Hedef Pose Oluştur
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = msg->x;
    target_pose.position.y = msg->y;
    target_pose.position.z = msg->z;
    
    // --- ÇOK ÖNEMLİ: Oryantasyon Stratejisi ---
    // Robotun ŞU ANKİ oryantasyonunu hedef olarak verelim.
    // Böylece robot elini döndürmeye çalışmaz, sadece konuma gitmeye odaklanır.
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
    target_pose.orientation = current_pose.orientation; 

    // 3. Yaklaşık (Approximate) Hedef Belirle
    // Bu fonksiyon IK çözemese bile "en yakın" duruşu kabul eder.
    // RViz'deki sürükle-bırak mantığının aynısıdır.
    RCLCPP_INFO(this->get_logger(), "🧮 Yaklaşık IK Çözümü Aranıyor...");
    
    // End effector link ismini URDF'ten doğru aldığından emin ol (genelde otomatiktir)
    bool found = move_group_->setApproximateJointValueTarget(target_pose, "");

    if (found) {
        RCLCPP_INFO(this->get_logger(), "✅ Çözüm Bulundu! Planlanıyor...");
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
             RCLCPP_INFO(this->get_logger(), "📐 Plan Başarılı! Gidiliyor...");
             move_group_->execute(my_plan);
             RCLCPP_INFO(this->get_logger(), "🏁 Hareket Tamamlandı!");
        } else {
             RCLCPP_WARN(this->get_logger(), "⚠️ IK bulundu ama CHOMP yörüngeyi oluşturamadı (Çarpışma?)");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Hedef çok uzak! (Approximate IK bile bulamadı)");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool robot_ready_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppBrainNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
