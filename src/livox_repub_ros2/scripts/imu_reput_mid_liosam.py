#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # 重力加速度常数
        self.g = 10
        self.callback_count = 0  # 用于调试计数
        
        # 配置适合IMU的QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            qos_profile  # 使用配置的QoS
        )
        
        # 创建发布者
        self.publisher = self.create_publisher(
            Imu, 
            '/livox/imu/processed',
            qos_profile  # 使用相同的QoS
        )
        
        # 定期打印状态的心跳
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)
        
        self.get_logger().info('IMU处理器已启动')
        self.get_logger().info('订阅: /rslidar_imu_data, 发布: /airy/imu/processed')

    def heartbeat_callback(self):
        """定期打印状态，确认节点存活"""
        self.get_logger().info(f'IMU处理器运行正常，已处理消息: {self.callback_count}条', 
                              throttle_duration_sec=10.0)

    def imu_callback(self, msg):
        """
        IMU消息回调函数，处理加速度数据
        """
        try:
            self.callback_count += 1
            
            # 调试信息：每100条消息打印一次
            if self.callback_count % 100 == 0:
                self.get_logger().info(f'已处理 {self.callback_count} 条IMU消息')
            
            # 创建新的IMU消息对象（重要！不要重复使用）
            processed_msg = Imu()
            
            # 复制header和时间戳
            processed_msg.header = msg.header
            processed_msg.header.frame_id = "livox_frame"  # 明确设置frame_id
            
            # 复制原始数据
            processed_msg.orientation = msg.orientation
            processed_msg.orientation_covariance = msg.orientation_covariance
            processed_msg.angular_velocity = msg.angular_velocity
            processed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            
            # 处理线性加速度：乘以重力加速度g
            processed_msg.linear_acceleration.x = msg.linear_acceleration.x * self.g
            processed_msg.linear_acceleration.y = msg.linear_acceleration.y * self.g
            processed_msg.linear_acceleration.z = msg.linear_acceleration.z * self.g
            
            # 复制线性加速度的协方差矩阵
            processed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            
            # 发布处理后的消息
            self.publisher.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'IMU回调处理异常: {e}')
            # 可以选择重新初始化或采取其他恢复措施

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 创建IMU处理器节点
        imu_processor = ImuProcessor()
        
        # 运行节点
        rclpy.spin(imu_processor)
        
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        # 清理资源
        if 'imu_processor' in locals():
            imu_processor.destroy_node()
        rclpy.shutdown()
        print("IMU处理器已关闭")

if __name__ == '__main__':
    main()