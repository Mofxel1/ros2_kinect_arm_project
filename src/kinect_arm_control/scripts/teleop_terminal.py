#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import sys

class TerminalTeleop(Node):
    def __init__(self):
        super().__init__('terminal_teleop')
        # MoveIt'i devreden çıkarıp doğrudan motor sürücüsüne bağlanıyoruz
        self._action_client = ActionClient(self, FollowJointTrajectory, '/my_arm_controller/follow_joint_trajectory')

    def send_goal(self, j0, j1, j2):
        self.get_logger().info('Motor sürücüsü bekleniyor...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint0', 'joint1', 'joint2']
        
        point = JointTrajectoryPoint()
        point.positions = [j0, j1, j2]
        point.time_from_start.sec = 2  # Hareketi 2 saniyede yumuşakça tamamla
        point.time_from_start.nanosec = 0

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Gonderiliyor -> J1: {j0} | J2: {j1} | J3: {j2}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TerminalTeleop()

    print("\n" + "="*40)
    print(" 🦾 ROBOT TERMINAL KONTROLU AKTIF")
    print("="*40)
    print("Cikmak icin CTRL+C'ye basin veya 'q' yazin.\n")
    
    try:
        while rclpy.ok():
            try:
                val0 = input("Joint 0 (Sağ/Sol) açısı girin (Örn: 0.0) : ")
                if val0.lower() == 'q': break
                j0 = float(val0)

                val1 = input("Joint 1 (Omuz) açısı girin    (Örn: 0.8) : ")
                if val1.lower() == 'q': break
                j1 = float(val1)

                val2 = input("Joint 2 (Dirsek) açısı girin  (Örn: 0.1) : ")
                if val2.lower() == 'q': break
                j2 = float(val2)

                teleop_node.send_goal(j0, j1, j2)
                print("-" * 40)
                
            except ValueError:
                print("⚠️ HATA: Lütfen geçerli bir sayı girin (Nokta kullanarak, örn: 0.5)\n")
                
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
