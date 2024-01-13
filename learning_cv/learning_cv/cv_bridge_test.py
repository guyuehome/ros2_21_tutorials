#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: cvbridge使用示例
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库


"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # 创建原始图像的订阅者
        self.pub = self.create_publisher(
            Image, 'cv_bridge_image', 10)                       # 创建cv处理之后的图像发布者
        self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换


    def listener_callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # 将ROS的图像消息转化成OpenCV图像

        (rows,cols,channels) = image.shape                      # 在OpenCV的显示窗口中绘制一个圆，作为标记
        if cols > 60 and rows > 60 :
            cv2.circle(image, (60, 60), 30, (0,0,255), -1)

        cv2.imshow("Image window", image)                       # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(3)
        
        self.pub.publish(self.cv_bridge.cv2_to_imgmsg(image, "bgr8"))   # 发布cv处理之后的图像

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = ImageSubscriber("cv_bridge_test")                # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口
