#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-监听某两个坐标系之间的变换
"""

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

class TFListener(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('source_frame', 'world')
        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value

        self.declare_parameter('target_frame', 'house')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return

        euler = tf_transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.source_frame, self.target_frame, trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, euler[0], euler[1], euler[2]))

def main(args=None):
   rclpy.init(args=args)                       # ROS2 Python接口初始化
   node = TFListener("tf_listener")            # 创建ROS2节点对象并进行初始化
   rclpy.spin(node)                            # 循环等待ROS2退出
   node.destroy_node()                         # 销毁节点对象
   rclpy.shutdown()