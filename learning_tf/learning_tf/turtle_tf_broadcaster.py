#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-广播动态的坐标变换
"""

import rclpy                                       # ROS2 Python接口库
from rclpy.node import Node                        # ROS2 节点类
from geometry_msgs.msg import TransformStamped     # 坐标变换消息
import tf_transformations                          # TF坐标变换库
from tf2_ros import TransformBroadcaster           # TF坐标变换广播器
from turtlesim.msg import Pose                     # turtlesim小海龟位置消息

class TurtleTFBroadcaster(Node):

    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化

        self.declare_parameter('turtlename', 'turtle')        # 创建一个海龟名称的参数
        self.turtlename = self.get_parameter(                 # 优先使用外部设置的参数值，否则用默认值
            'turtlename').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)      # 创建一个TF坐标变换的广播对象并初始化

        self.subscription = self.create_subscription(         # 创建一个订阅者，订阅海龟的位置消息
            Pose,
            f'/{self.turtlename}/pose',                       # 使用参数中获取到的海龟名称
            self.turtle_pose_callback, 1)

    def turtle_pose_callback(self, msg):                              # 创建一个处理海龟位置消息的回调函数，将位置消息转变成坐标变换
        transform = TransformStamped()                                # 创建一个坐标变换的消息对象

        transform.header.stamp = self.get_clock().now().to_msg()      # 设置坐标变换消息的时间戳
        transform.header.frame_id = 'world'                           # 设置一个坐标变换的源坐标系
        transform.child_frame_id = self.turtlename                    # 设置一个坐标变换的目标坐标系
        transform.transform.translation.x = msg.x                     # 设置坐标变换中的X、Y、Z向的平移
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta) # 将欧拉角转换为四元数（roll, pitch, yaw）
        transform.transform.rotation.x = q[0]                         # 设置坐标变换中的X、Y、Z向的旋转（四元数）
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(transform)     # 广播坐标变换，海龟位置变化后，将及时更新坐标变换信息

def main(args=None):
    rclpy.init(args=args)                                # ROS2 Python接口初始化
    node = TurtleTFBroadcaster("turtle_tf_broadcaster")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                     # 循环等待ROS2退出
    node.destroy_node()                                  # 销毁节点对象
    rclpy.shutdown()                                     # 关闭ROS2 Python接口
