#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-监听某两个坐标系之间的变换
"""

import rclpy                                              # ROS2 Python接口库
from rclpy.node import Node                               # ROS2 节点类
import tf_transformations                                 # TF坐标变换库
from tf2_ros import TransformException                    # TF左边变换的异常类
from tf2_ros.buffer import Buffer                         # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类

class TFListener(Node):

    def __init__(self, name):
        super().__init__(name)                                      # ROS2节点父类初始化

        self.declare_parameter('source_frame', 'world')             # 创建一个源坐标系名的参数
        self.source_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'source_frame').get_parameter_value().string_value

        self.declare_parameter('target_frame', 'house')             # 创建一个目标坐标系名的参数
        self.target_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()                                   # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建坐标变换的监听器

        self.timer = self.create_timer(1.0, self.on_timer)          # 创建一个固定周期的定时器，处理坐标信息

    def on_timer(self):
        try:
            now = rclpy.time.Time()                                 # 获取ROS系统的当前时间
            trans = self.tf_buffer.lookup_transform(                # 监听当前时刻源坐标系到目标坐标系的坐标变换
                self.target_frame,
                self.source_frame,
                now)
        except TransformException as ex:                            # 如果坐标变换获取失败，进入异常报告
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return
        
        pos  = trans.transform.translation                          # 获取位置信息
        quat = trans.transform.rotation                             # 获取姿态信息（四元数）
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

def main(args=None):
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = TFListener("tf_listener")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口
