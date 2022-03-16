#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-广播静态的坐标变换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTFBroadcaster(Node):
   def __init__(self, name):
      super().__init__(name)
      self.tf_broadcaster = StaticTransformBroadcaster(self)

      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'world'
      static_transformStamped.child_frame_id  = 'house'
      static_transformStamped.transform.translation.x = 10.0
      static_transformStamped.transform.translation.y = 5.0
      static_transformStamped.transform.translation.z = 0.0
      quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch and yaw
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]

      self.tf_broadcaster.sendTransform(static_transformStamped)

def main(args=None):
   rclpy.init(args=args)                                # ROS2 Python接口初始化
   node = StaticTFBroadcaster("static_tf_broadcaster")  # 创建ROS2节点对象并进行初始化
   rclpy.spin(node)                                     # 循环等待ROS2退出
   node.destroy_node()                                  # 销毁节点对象
   rclpy.shutdown()
