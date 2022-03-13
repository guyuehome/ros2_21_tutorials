#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: OpenCV物体识别测试
"""

import cv2                                                            # Opencv图像处理库
import numpy as np                                                    # Python数值计算库

lower_red = np.array([0, 90, 128])                                    # 红色范围的阈值下限
upper_red = np.array([180, 255, 255])                                 # 红色范的围阈值上限

image = cv2.imread('apple.jpg')                                       # 读取一张图片

hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                      # 图像从BGR颜色模型转换为HSV模型
mask_red = cv2.inRange(hsv_img, lower_red, upper_red)                 # 图像二值化

contours, hierarchy = cv2.findContours(\
    mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)                   # 图像中轮廓检测

for cnt in contours:                                                  # 去除一些轮廓面积太小的噪声
    if cnt.shape[0] < 150:
        continue
        
    (x, y, w, h) = cv2.boundingRect(cnt)                              # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
    cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)                # 将苹果的轮廓勾勒出来
    cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)   # 将苹果的图像中心点画出来
	
cv2.imshow("object", image)                                            # 使用OpenCV显示处理后的图像效果
cv2.waitKey(0)
cv2.destroyAllWindows()
