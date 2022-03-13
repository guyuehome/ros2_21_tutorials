#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-提供目标识别服务
"""

import rclpy                                           # ROS2 Python接口库
from rclpy.node import Node                            # ROS2 节点类
from sensor_msgs.msg import Image                      # 图像消息类型
import numpy as np                                     # Python数值计算库
from cv_bridge import CvBridge                         # ROS与OpenCV图像转换类
import cv2                                             # Opencv图像处理库
from learning_interface.srv import GetObjectPosition   # 自定义的服务接口

lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)             # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                                     # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

        self.srv = self.create_service(GetObjectPosition,               # 创建服务器对象（接口类型、服务名、服务器回调函数）
                                       'get_target_position',
                                       self.object_position_callback)    
        self.objectX = 0
        self.objectY = 0                              

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)             # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)        # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)          # 图像中轮廓检测

        for cnt in contours:                                         # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)                     # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)       # 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                              # 将苹果的图像中心点画出来
            
            self.objectX = int(x+w/2)
            self.objectY = int(y+h/2)

        cv2.imshow("object", image)                                  # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(50)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')               # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')            # 将ROS的图像消息转化成OpenCV图像
        self.object_detect(image)                                      # 苹果检测

    def object_position_callback(self, request, response):            # 创建回调函数，执行收到请求后对数据的处理
        if request.get == True:
            response.x = self.objectX                                 # 目标物体的XY坐标
            response.y = self.objectY
            self.get_logger().info('Object position\nx: %d y: %d' %
                                   (response.x, response.y))          # 输出日志信息，提示已经反馈
        else:
            response.x = 0
            response.y = 0
            self.get_logger().info('Invalid command')                 # 输出日志信息，提示已经反馈
        return response


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImageSubscriber("service_object_server")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
