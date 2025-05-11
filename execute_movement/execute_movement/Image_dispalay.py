import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PyQt5.QtCore import QThread, pyqtSignal,Qt
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap, QImage
import numpy as np

class Image_Display(QThread):
    def __init__(self,ImageLabel):
        super().__init__()
        self.node = None
        self.imagelab = ImageLabel
    
    def run(self):
        self.node = Image_get(self.imagelab)  # 创建自定义节点
        try:
            rclpy.spin(self.node)  # 启动 ROS2 主循环
        except Exception as e:
            print(f"Exception in ROS2Thread: {e}")
        finally:
            self.node.destroy_node()
            rclpy.try_shutdown()


class Image_get(Node):
    """自定义 ROS2 节点"""
    def __init__(self, ImageLabel):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,  # 消息类型
            'rgb/image_raw',  # 订阅的话题名称
            self.listener_callback,  # 回调函数
            10)  # QoS
        self.ImageLabel =  ImageLabel # 用于将图像传递到主线程的信号
        self.q_image = None
        self.pixmap = None

    def listener_callback(self, msg):
       
        # 只处理关键信息，减少日志量
        if msg.encoding != 'bgra8':
            return
            
        # 直接转换为QImage（跳过numpy转换）
        self.q_image = QImage(
        msg.data, 
        msg.width, 
        msg.height, 
        msg.step,
        QImage.Format_RGBA8888  # 使用ARGB格式
    ).rgbSwapped()  # 关键操作：交换R和B通道

        self.pixmap = QPixmap.fromImage(self.q_image)
        scaled_pixmap = self.pixmap.scaled(800, 600, Qt.KeepAspectRatio)  # 调整大小并保持宽高比

        self.ImageLabel.setPixmap(scaled_pixmap)
        self.ImageLabel.setScaledContents(False)  # 不需要自动缩放 QLabel 的大小
        
        