#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BoolSubscriber(Node):
    def __init__(self):
        super().__init__('servo_control')
        self.subscription = self.create_subscription(
            Bool,
            'bool_flag',  # 订阅的话题名称
            self.listener_callback,
            10  # QoS队列深度
        )
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('收到')
            # 如果需要打印时间戳可以改为：
            # self.get_logger().info(f'收到 [{self.get_clock().now().to_msg().sec}]')

def main(args=None):
    rclpy.init(args=args)
    node = BoolSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()