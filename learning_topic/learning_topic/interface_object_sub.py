#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2接口示例-订阅目标位置
"""

import rclpy                                       # ROS2 Python接口库
from rclpy.node   import Node                      # ROS2 节点类
from std_msgs.msg import String                    # 字符串消息类型
from learning_interface.msg import ObjectPosition  # 自定义的目标位置消息

"""
创建一个订阅者节点
"""
class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            ObjectPosition, "/object_position", self.listener_callback, 10)       # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度

    def listener_callback(self, msg):                                             # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('Target Position: "(%d, %d)"' % (msg.x, msg.y))    # 输出日志信息，提示订阅收到的话题消息

        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("interface_position_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
