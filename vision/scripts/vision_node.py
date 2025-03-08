#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node, Rate
from vision.msg import Vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv_tools # 导入工具类

class vision_pub_node(Node):
    def __init__(self):
        super().__init__('vision_pub')
        self.vision_pub = self.create_publisher(Vision, 'vision', 10)
        #self.frame_sub = self.create_subscription(Image, '/camera/ground', self.ground_callback, 1) # 实机
        self.frame_sub = self.create_subscription(Image, '/camera_ground/image_raw', self.ground_callback, 1) # 仿真
        self.bridge = CvBridge()
        self.cv_tools = cv_tools.CVTools(self)  # 创建工具类实例
        # 初始化消息
        self.msg = Vision()
        self.msg.is_line_detected = False
        self.msg.lateral_error = 0
        self.msg.angle_error = 0.0
        self.msg.is_shape_detected = False
        self.msg.center_x = 0.0
        self.msg.center_y = 0.0
        self.get_logger().info("init complete")
    
    def ground_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error convert image: {e}")
            return
        self.process(cv_image) # 处理并发布vision消息

    def process(self, frame):
        try:            
            bl_frame = self.cv_tools.backlight_compensation(frame) # 逆光补偿
            #cv2.imshow('逆光补偿效果', bl_frame)

            hl_copy = self.cv_tools.line_detect(bl_frame) # 霍夫直线
            cv2.imshow('霍夫直线效果', hl_copy)
            
            #gray_frame = cv2.cvtColor(bl_frame, cv2.COLOR_BGR2GRAY) # 灰度
            #_, thresh_frame = cv2.threshold(gray_frame, 150, 255, cv2.THRESH_BINARY) # 二值化处理
            #cv2.imshow('预处理最终效果', thresh_frame)

            #contours, _ = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # 提取轮廓
            #detect_copy = cv_tools.detect_contours(contours, frame) # 过滤轮廓，并检测
            #cv2.imshow('图形检测效果', detect_copy)

            cv2.waitKey(1)
            
            self.vision_pub.publish(self.msg) # ros发布

        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")

    def release_resources(self):
        self.get_logger().info("Resources released.")

# entry_point入口
def main():
    rclpy.init(args=None)
    node = vision_pub_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()