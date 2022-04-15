#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-设置目标识别的颜色阈值参数
"""

import rclpy                      # ROS2 Python接口库
from rclpy.node import Node       # ROS2 节点类
from sensor_msgs.msg import Image # 图像消息类型
from cv_bridge import CvBridge    # ROS与OpenCV图像转换类
import cv2                        # Opencv图像处理库
import numpy as np                # Python数值计算库

lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
  def __init__(self, name):
    super().__init__(name)                                  # ROS2节点父类初始化    
    self.sub = self.create_subscription(Image,              # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）     
                  'image_raw', self.listener_callback, 10) 
    self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

    self.declare_parameter('red_h_upper', 0)                # 创建一个参数，表示阈值上限
    self.declare_parameter('red_h_lower', 0)                # 创建一个参数，表示阈值下限
    
  def object_detect(self, image):
    upper_red[0] = self.get_parameter('red_h_upper').get_parameter_value().integer_value      # 读取阈值上限的参数值
    lower_red[0] = self.get_parameter('red_h_lower').get_parameter_value().integer_value      # 读取阈值下限的参数值
    self.get_logger().info('Get Red H Upper: %d, Lower: %d' % (upper_red[0], lower_red[0]))   # 通过日志打印读取到的参数值
    
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                                          # 图像从BGR颜色模型转换为HSV模型
    mask_red = cv2.inRange(hsv_img, lower_red, upper_red)                                     # 图像二值化
    contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)    # 图像中轮廓检测
    for cnt in contours:                                                                      # 去除一些轮廓面积太小的噪声
        if cnt.shape[0] < 150:
            continue
            
        (x, y, w, h) = cv2.boundingRect(cnt)                                      # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
        cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)                        # 将苹果的轮廓勾勒出来
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)           # 将苹果的图像中心点画出来
        
    cv2.imshow("object", image)                                                   # 使用OpenCV显示处理后的图像效果
    cv2.waitKey(50)
       
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')     # 输出日志信息，提示已进入回调函数
    image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")  # 将ROS的图像消息转化成OpenCV图像
    self.object_detect(image)                            # 苹果检测
  
def main(args=None):                                    # ROS2节点主入口main函数
    rclpy.init(args=args)                               # ROS2 Python接口初始化
    node = ImageSubscriber("param_object_detect")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                    # 循环等待ROS2退出
    node.destroy_node()                                 # 销毁节点对象
    rclpy.shutdown()                                    # 关闭ROS2 Python接口
