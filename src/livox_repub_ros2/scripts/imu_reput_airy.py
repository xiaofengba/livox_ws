#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # 重力加速度常数
        self.g = 1.0
        self.scale = -1.0
        
        
        # 创建订阅者，订阅原始IMU数据
        self.subscription = self.create_subscription(
            Imu,
            '/rslidar_imu_data',  # 输入话题，根据实际情况调整
            self.imu_callback,
            10)
        
        # 创建发布者，发布处理后的IMU数据
        self.publisher = self.create_publisher(
            Imu, 
            '/imu/processed',  # 输出话题
            10)
        
        self.get_logger().info(f'IMU处理器已启动，重力缩放系数: {self.g}')
        self.get_logger().info('订阅: /imu/data_raw, 发布: /imu/processed')

    def imu_callback(self, msg):
        """
        IMU消息回调函数，处理加速度数据
        """
        # 创建新的IMU消息
        processed_msg = Imu()
        
        # 复制原始消息的所有内容
        processed_msg.header = msg.header
        processed_msg.header.frame_id = "rs"
        processed_msg.orientation = msg.orientation
        processed_msg.orientation_covariance = msg.orientation_covariance

        processed_msg.angular_velocity.x = msg.angular_velocity.y * self.scale 
        processed_msg.angular_velocity.y = msg.angular_velocity.x * self.scale
        processed_msg.angular_velocity.z = msg.angular_velocity.z * self.scale

        processed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # 处理线性加速度：乘以重力加速度g
        processed_msg.linear_acceleration.x = msg.linear_acceleration.y * self.g * self.scale
        processed_msg.linear_acceleration.y = msg.linear_acceleration.x * self.g * self.scale
        processed_msg.linear_acceleration.z = msg.linear_acceleration.z * self.g * self.scale
        
        # 复制线性加速度的协方差矩阵
        processed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # 发布处理后的消息
        self.publisher.publish(processed_msg)
        
        # 可选：打印调试信息（生产环境中建议注释掉）
        self.get_logger().debug(
            f'处理加速度: [{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}] -> '
            f'[{processed_msg.linear_acceleration.x:.3f}, {processed_msg.linear_acceleration.y:.3f}, {processed_msg.linear_acceleration.z:.3f}]',
            throttle_duration_sec=1.0  # 每秒最多打印一次
        )

def main(args=None):
    rclpy.init(args=args)
    
    # 创建IMU处理器节点
    imu_processor = ImuProcessor()
    
    try:
        # 运行节点
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()