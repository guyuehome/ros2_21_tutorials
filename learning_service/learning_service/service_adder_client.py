#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2服务示例
发送两个加数，请求加法器计算
"""

import sys

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from learning_interface.srv import AddTwoInts    # 自定义的服务接口

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                                  # ROS2节点父类初始化
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = AddTwoInts.Request()
                    
    def send_request(self):
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = adderClient("service_adder_client")       # 创建ROS2节点对象并进行初始化
    node.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(node)

        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (node.request.a, node.request.b, response.sum))
            break
            
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
