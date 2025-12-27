#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/diff_drive_controller/cmd_vel_unstamped', 
            10
        )
        
        self.get_logger().info("Motor test node started")
        
    def send_cmd(self, linear_x, angular_z, duration):
        """指定された速度で指定時間動作"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        self.get_logger().info(f"Sending: linear_x={linear_x:.2f}, angular_z={angular_z:.2f} for {duration}s")
        
        # 指定時間送信
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)  # 10Hz
            
    def stop(self):
        """停止コマンド"""
        cmd = Twist()  # 全て0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("STOP")
        
    def run_test_sequence(self):
        """テストシーケンス実行"""
        self.get_logger().info("Starting motor test sequence...")
        
        try:
            # 前進 2秒
            self.send_cmd(0.3, 0.0, 2.0)
            self.stop()
            time.sleep(1)
            
            # 後退 2秒  
            self.send_cmd(-0.3, 0.0, 2.0)
            self.stop()
            time.sleep(1)
            
            # 左回転 2秒
            self.send_cmd(0.0, 0.5, 2.0)
            self.stop()
            time.sleep(1)
            
            # 右回転 2秒
            self.send_cmd(0.0, -0.5, 2.0)
            self.stop()
            
            self.get_logger().info("Test sequence completed!")
            
        except KeyboardInterrupt:
            self.get_logger().info("Test interrupted")
            self.stop()

def main():
    rclpy.init()
    
    motor_test = MotorTest()
    
    # 少し待機してシステムが準備されるのを待つ
    time.sleep(2)
    
    motor_test.run_test_sequence()
    
    motor_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()