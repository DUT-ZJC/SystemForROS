import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap, QImage
import numpy as np

class Image_Display(QThread):
    image_received = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.node = None
    
    def run(self):
        rclpy.init()  # 初始化 ROS2
        self.node = Image_get(self.image_received)  # 创建自定义节点
        try:
            rclpy.spin(self.node)  # 启动 ROS2 主循环
        except Exception as e:
            print(f"Exception in ROS2Thread: {e}")
        finally:
            self.node.destroy_node()
            rclpy.try_shutdown()


class Image_get(Node):
    """自定义 ROS2 节点"""
    def __init__(self, signal):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,  # 消息类型
            'camera/image',  # 订阅的话题名称
            self.listener_callback,  # 回调函数
            10)  # QoS
        self.signal = signal  # 用于将图像传递到主线程的信号

    def listener_callback(self, msg):
        """回调函数，运行在子线程中"""
        self.get_logger().info('Received image')
        # 将 ROS2 图像消息转换为 NumPy 数组
        image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # 将 NumPy 数组转换为 QImage
        q_image = QImage(image_data.data, msg.width, msg.height, msg.width * 3, QImage.Format_RGB888)
        # 发射信号，将图像传递到主线程
        self.signal.emit(q_image)