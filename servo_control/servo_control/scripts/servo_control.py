#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time
import threading

class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control')
        
        # 硬件参数
        self.servo_pin = 33
        self.current_angle = 0
        self.keep_active = True
        
        # GPIO初始化（严格遵循原始代码）
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT, initial=GPIO.HIGH)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        
        # 信号维持线程
        self.signal_thread = threading.Thread(target=self.maintain_signal)
        self.signal_thread.start()
        
        # ROS订阅
        self.subscription = self.create_subscription(
            Bool,
            'servo_flag',
            self.callback,
            10
        )

    def maintain_signal(self):
        """持续维持PWM信号"""
        self.pwm.start(0)
        try:
            while self.keep_active:
                duty = 8.5 if self.current_angle == 90 else 3.5
                self.pwm.ChangeDutyCycle(duty)
                time.sleep(0.1)  # 10Hz刷新频率
        except:
            self.pwm.stop()

    def callback(self, msg):
        """消息回调"""
        new_angle = 90 if msg.data else 0
        if new_angle != self.current_angle:
            self.current_angle = new_angle
            self.get_logger().info(f"角度更新: {self.current_angle}度")
            time.sleep(0.5)  # 确保舵机完成转动

    def __del__(self):
        """资源清理"""
        self.keep_active = False
        self.signal_thread.join()
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("硬件资源已释放")

def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("安全终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()