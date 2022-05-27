#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-请求目标识别，等待目标位置应答
"""

import rclpy                                            # ROS2 Python接口库
from rclpy.node   import Node                           # ROS2 节点类
from learning_interface.srv import GetObjectPosition    # 自定义的服务接口

class objectClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                                  # ROS2节点父类初始化
        self.client = self.create_client(GetObjectPosition, 'get_target_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = GetObjectPosition.Request()
                    
    def send_request(self):
        self.request.get = True
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)                             # ROS2 Python接口初始化
    node = objectClient("service_object_client")       # 创建ROS2节点对象并进行初始化
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
                    'Result of object position:\n x: %d y: %d' %
                    (response.x, response.y))
            break
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
