#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class RedDetector(Node):
    def __init__(self):
        super().__init__('red_detector')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 图像订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera_ground/image_raw',  # 根据实际话题修改
            self.image_callback,
            10)
        
        # 处理结果发布者
        self.proc_pub = self.create_publisher(Image, '/processed_image2', 10)
        self.center_pub = self.create_publisher(Point, '/red_center_offset', 10)
        
        # 定义黄色HSV阈值范围
        # self.lower_yellow = np.array([20, 100, 100])
        # self.upper_yellow = np.array([30, 255, 255])

        # 定义红色HSV阈值范围
        self.lower_red1 = np.array([0, 100, 100])    # 低区间 (H: 0-10)
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])  # 高区间 (H: 160-180)
        self.upper_red2 = np.array([180, 255, 255])

        # 绿色的HSV阈值范围
        # self.lower_green = np.array([35, 100, 100])   # H:35~85, S≥100, V≥100
        # self.upper_green = np.array([85, 255, 255])
        # 形态学操作核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

    def image_callback(self, msg):
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {str(e)}')
            return
        
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 创建绿色掩模
        # mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        # 创建红色掩模
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask_red1, mask_red2)
        # 创建黄色掩模
        # mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # 形态学处理：先腐蚀再膨胀（开运算）
        processed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        
        # 二值化
        _, binary = cv2.threshold(processed, 127, 255, cv2.THRESH_BINARY)
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # 找最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 计算轮廓矩
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                # 计算中心坐标
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # 获取图像中心
                h, w = binary.shape
                img_center = (w//2, h//2)
                
                # 计算距离
                dx = cX - img_center[0]
                dy = cY - img_center[1]
                distance = (dx**2 + dy**2)**0.5
                
                # 发布偏移量
                offset_msg = Point()
                offset_msg.x = float(dx)
                offset_msg.y = float(dy)
                offset_msg.z = float(distance)
                self.center_pub.publish(offset_msg)
                
                # 在图像上绘制
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.line(cv_image, img_center, (cX, cY), (255, 0, 0), 2)
                cv2.putText(cv_image, f"Dist: {distance:.1f}px", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                offset_msg = Point()
                offset_msg.x = 0.0
                offset_msg.y = 0.0
                offset_msg.z = 114514.0
                self.center_pub.publish(offset_msg)
        else:
            offset_msg = Point()
            offset_msg.x = 0.0
            offset_msg.y = 0.0
            offset_msg.z = 114514.0
            self.center_pub.publish(offset_msg)
        # 发布处理后的图像
        try:
            proc_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.proc_pub.publish(proc_msg)
        except Exception as e:
            self.get_logger().error(f'图像发布失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = RedDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()