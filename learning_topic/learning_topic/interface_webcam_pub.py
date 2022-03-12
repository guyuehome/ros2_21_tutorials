#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2话题示例
按照某一频率发布图像话题

"""

import rclpy                      # ROS2 Python接口库
from rclpy.node import Node       # ROS2 节点类
from sensor_msgs.msg import Image # 图像消息类型
from cv_bridge import CvBridge    # ROS与OpenCV图像转换类
import cv2                        # Opencv图像处理库
 
"""
创建一个发布者节点
"""
class ImagePublisher(Node):
  def __init__(self):
    super().__init__(name)                                                  # ROS2节点父类初始化
    self.pub   = self.create_publisher(Image, 'usb_cam/ImageRaw', 10)       # 创建发布者对象（消息类型、话题名、队列长度）
    self.timer = self.create_timer(0.1, self.timer_callback)                # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
    self.cap = cv2.VideoCapture(0)                                          # 创建一个视频采集对象，驱动相机采集图像（相机设备号）         
    self.cv_bridge = CvBridge()                                             # 创建一个图像转换对象，用于稍后将OpenCV的图像转换成ROS的图像消息
   
  def timer_callback(self):
    ret, frame = self.cap.read()                                            # 一帧一帧读取图像
          
    if ret == True:
      self.pub.publish(self.cv_bridge.cv2_to_imgmsg(frame))          # 发布图像消息
 
    self.get_logger().info('Publishing video frame')                        # 输出日志信息，提示已经完成图像话题发布
  
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("interface_webcam_pub")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
