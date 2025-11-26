#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct

class FastPointCloudRepublisher(Node):
    def __init__(self):
        super().__init__('fast_pointcloud_republisher')
        
        # 订阅和发布
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            PointCloud2, 
            '/rslidar_points_repub', 
            10
        )
        
        # 定义输出点云的字段
        self.output_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)
        ]
        
        # 预计算字段信息
        self.field_offsets = None
        self.field_datatypes = None
        
        self.get_logger().info('Fast PointCloud republisher node started')

    def pointcloud_callback(self, msg):
        """
        优化版本的点云转换回调函数
        """
        try:
            # 如果字段信息未初始化或发生变化，重新计算
            if (self.field_offsets is None or 
                self.field_datatypes is None or
                len(msg.fields) != len(self.field_offsets)):
                self._update_field_info(msg)
            
            # 使用NumPy进行批量转换
            output_data = self.fast_convert_pointcloud_data(msg)
            
            # 创建输出消息
            output_msg = PointCloud2()
            output_msg.header = msg.header
            output_msg.height = msg.height
            output_msg.width = msg.width
            output_msg.fields = self.output_fields
            output_msg.is_bigendian = False
            output_msg.point_step = 22  # 6个字段总大小
            output_msg.row_step = output_msg.point_step * msg.width
            output_msg.is_dense = msg.is_dense
            output_msg.data = output_data
            
            self.publisher_.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud conversion: {str(e)}')

    def _update_field_info(self, msg):
        """
        更新字段偏移和数据类型信息
        """
        self.field_offsets = {}
        self.field_datatypes = {}
        
        for field in msg.fields:
            self.field_offsets[field.name] = field.offset
            self.field_datatypes[field.name] = field.datatype

    def fast_convert_pointcloud_data(self, input_msg):
        """
        使用NumPy进行批量转换，显著提高速度
        """
        num_points = input_msg.width * input_msg.height
        
        if num_points == 0:
            return b''
        
        # 将点云数据转换为NumPy数组
        point_data = np.frombuffer(input_msg.data, dtype=np.uint8)
        
        # 重塑为点数组 (num_points, point_step)
        points_2d = point_data.reshape(num_points, input_msg.point_step)
        
        # 批量提取字段数据
        # 提取x, y, z, intensity (float32)
        x_data = points_2d[:, self.field_offsets['x']:self.field_offsets['x']+4].view(np.float32).flatten()
        y_data = points_2d[:, self.field_offsets['y']:self.field_offsets['y']+4].view(np.float32).flatten()
        z_data = points_2d[:, self.field_offsets['z']:self.field_offsets['z']+4].view(np.float32).flatten()
        intensity_data = points_2d[:, self.field_offsets['intensity']:self.field_offsets['intensity']+4].view(np.float32).flatten()
        
        # 提取ring (uint16)
        ring_data = points_2d[:, self.field_offsets['ring']:self.field_offsets['ring']+2].view(np.uint16).flatten()
        
        # 提取timestamp并根据类型处理
        timestamp_data = points_2d[:, self.field_offsets['timestamp']:self.field_offsets['timestamp']+8].view(np.float64).flatten()

        timestamp_data = (timestamp_data - timestamp_data[0])*1000

        # 转换为float32
        time_data = timestamp_data.astype(np.float32)

        # print(time_data)

        # print("====================")
        # print(time_data[0], time_data[-1],  time_data[-1]-time_data[0] )
        
        # 创建输出数据数组
        # 使用结构化数组来高效打包数据
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32), 
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)
        ])
        
        # 创建结构化数组
        structured_data = np.empty(num_points, dtype=dtype)
        structured_data['x'] = x_data
        structured_data['y'] = y_data
        structured_data['z'] = z_data
        structured_data['intensity'] = intensity_data
        structured_data['ring'] = ring_data
        structured_data['time'] = time_data



        
        # 转换为字节
        return structured_data.tobytes()

def main(args=None):
    rclpy.init(args=args)
    node = FastPointCloudRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()