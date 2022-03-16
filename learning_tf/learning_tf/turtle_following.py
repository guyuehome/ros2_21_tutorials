#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-通过坐标变化实现海龟跟随功能
"""

import math
import rclpy                                              # ROS2 Python接口库
from rclpy.node import Node                               # ROS2 节点类
import tf_transformations                                 # TF坐标变换库
from tf2_ros import TransformException                    # TF左边变换的异常类
from tf2_ros.buffer import Buffer                         # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类
from geometry_msgs.msg import Twist                       # ROS2 速度控制消息
from turtlesim.srv import Spawn                           # 海龟生成的服务接口
class TurtleFollowing(Node):

    def __init__(self, name):
        super().__init__(name)                                      # ROS2节点父类初始化

        self.declare_parameter('source_frame', 'turtle1')           # 创建一个源坐标系名的参数
        self.source_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'source_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()                                   # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建坐标变换的监听器

        self.spawner = self.create_client(Spawn, 'spawn')           # 创建一个请求产生海龟的客户端
        self.turtle_spawning_service_ready = False                  # 是否已经请求海龟生成服务的标志位
        self.turtle_spawned = False                                 # 海龟是否产生成功的标志位

        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1) # 创建跟随运动海龟的速度话题

        self.timer = self.create_timer(1.0, self.on_timer)         # 创建一个固定周期的定时器，控制跟随海龟的运动

    def on_timer(self):
        from_frame_rel = self.source_frame                         # 源坐标系
        to_frame_rel   = 'turtle2'                                 # 目标坐标系

        if self.turtle_spawning_service_ready:                     # 如果已经请求海龟生成服务
            if self.turtle_spawned:                                # 如果跟随海龟已经生成
                try:
                    now = rclpy.time.Time()                        # 获取ROS系统的当前时间
                    trans = self.tf_buffer.lookup_transform(       # 监听当前时刻源坐标系到目标坐标系的坐标变换
                        to_frame_rel,
                        from_frame_rel,
                        now)
                except TransformException as ex:                   # 如果坐标变换获取失败，进入异常报告
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()                                      # 创建速度控制消息
                scale_rotation_rate = 1.0                          # 根据海龟角度，计算角速度
                msg.angular.z = scale_rotation_rate * math.atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x)

                scale_forward_speed = 0.5                          # 根据海龟距离，计算线速度
                msg.linear.x = scale_forward_speed * math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2)

                self.publisher.publish(msg)                        # 发布速度指令，海龟跟随运动
            else:                                                  # 如果跟随海龟没有生成
                if self.result.done():                             # 查看海龟是否生成
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True                     
                else:                                              # 依然没有生成跟随海龟
                    self.get_logger().info('Spawn is not finished')
        else:                                                      # 如果没有请求海龟生成服务
            if self.spawner.service_is_ready():                    # 如果海龟生成服务器已经准备就绪
                request = Spawn.Request()                          # 创建一个请求的数据
                request.name = 'turtle2'                           # 设置请求数据的内容，包括海龟名、xy位置、姿态
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)

                self.result = self.spawner.call_async(request)     # 发送服务请求
                self.turtle_spawning_service_ready = True          # 设置标志位，表示已经发送请求
            else:
                self.get_logger().info('Service is not ready')     # 海龟生成服务器还没准备就绪的提示


def main(args=None):
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = TurtleFollowing("turtle_following")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口