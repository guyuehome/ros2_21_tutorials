#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-广播静态的坐标变换
"""

import rclpy                                                                 # ROS2 Python接口库
from rclpy.node import Node                                                  # ROS2 节点类
from geometry_msgs.msg import TransformStamped                               # 坐标变换消息
import tf_transformations                                                    # TF坐标变换库
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # TF静态坐标系广播器类

class StaticTFBroadcaster(Node):
    def __init__(self, name):
        super().__init__(name)                                                  # ROS2节点父类初始化
        self.tf_broadcaster = StaticTransformBroadcaster(self)                  # 创建一个TF广播器对象

        static_transformStamped = TransformStamped()                            # 创建一个坐标变换的消息对象
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()  # 设置坐标变换消息的时间戳
        static_transformStamped.header.frame_id = 'world'                       # 设置一个坐标变换的源坐标系
        static_transformStamped.child_frame_id  = 'house'                       # 设置一个坐标变换的目标坐标系
        static_transformStamped.transform.translation.x = 10.0                  # 设置坐标变换中的X、Y、Z向的平移
        static_transformStamped.transform.translation.y = 5.0                    
        static_transformStamped.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)          # 将欧拉角转换为四元数（roll, pitch, yaw）
        static_transformStamped.transform.rotation.x = quat[0]                  # 设置坐标变换中的X、Y、Z向的旋转（四元数）
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transformStamped)              # 广播静态坐标变换，广播后两个坐标系的位置关系保持不变

def main(args=None):
    rclpy.init(args=args)                                # ROS2 Python接口初始化
    node = StaticTFBroadcaster("static_tf_broadcaster")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                     # 循环等待ROS2退出
    node.destroy_node()                                  # 销毁节点对象
    rclpy.shutdown()
