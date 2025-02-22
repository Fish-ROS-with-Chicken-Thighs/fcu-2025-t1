#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node, Rate
from vision.msg import Vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv_tools

class vision_pub_node(Node):
    def __init__(self):
        super().__init__('vision_pub')
        self.pub = self.create_publisher(Vision, 'vision', 10)
        #self.sub = self.create_subscription(Image, '/camera/ground', self.ground_callback, 1) # 实机
        self.sub = self.create_subscription(Image, '/camera_ground/image_raw', self.ground_callback, 1) # 仿真
        self.bridge = CvBridge()
        self.rate = self.create_rate(20)
        self.out = None
        self.get_logger().info("init complete")
    
    def ground_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error convert image: {e}")
            return
        # 处理并发布vision消息
        self.process(cv_image)

    def process(self, frame):
        if self.out is not None:
            self.out.release()

        msg = Vision()
        msg.is_shape_detected = False
        msg.center_x = 0
        msg.center_y = 0
        try:
            width, height, fps, out = cv_tools.get_video_info(frame)

            bl_frame = cv_tools.backlight_compensation(frame) # 逆光补偿
            # cv2.imshow('逆光补偿效果', bl_frame)

            hough_line_frame = cv_tools.line_detect(bl_frame) # 霍夫直线
            # cv2.imshow('霍夫直线效果', hough_line_frame)

            gray_frame = cv2.cvtColor(bl_frame, cv2.COLOR_BGR2GRAY) # 灰度
            _, thresh_frame = cv2.threshold(gray_frame, 150, 255, cv2.THRESH_BINARY) # 二值化处理
            # cv2.imshow('预处理最终效果', thresh_frame)

            contours, _ = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # 提取轮廓
            
            frame_copy = frame.copy() # 展示拷贝
            frame_copy = cv_tools.filter_contours(contours, frame, frame_copy) # 过滤轮廓，并检测

            cv_tools.imshow_and_save(frame, out) # 展示结果，写入视频

        except Exception as e:
            self.get_logger().error(f"Error occurred: {e}")
        finally:
            self.release_resources()

    def release_resources(self):
        self.out.release()
        self.get_logger().info("Resources released.")

# entry_point入口
def main():
    rclpy.init(args=None)
    node = vision_pub_node()
    try:
        while rclpy.ok():
            # 其它逻辑
            node.rate.sleep()  # 20hz
    except KeyboardInterrupt:
        node.get_logger().info("shutting down.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()