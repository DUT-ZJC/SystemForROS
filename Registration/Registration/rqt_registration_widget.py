import math
import pathlib
import numpy as np
import transforms3d as tfs
from std_msgs.msg import String
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer

from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from rqt_gui.main import Main
from datetime import datetime
from collections import deque
from std_msgs.msg import String
from Registration.point_cloud_get import PointCloudViewerNode
import threading
from . import pca_icp

class RqtRegistrationWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtRegistrationWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context
        self._node = context.node
        self.registrateFlag = False
        self.points_node = PointCloudViewerNode()
        self.re = pca_icp.PCA_ICP()
        self.re.finished.connect(self.re_finished)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.points_node)

        # 启动事件循环线程
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.daemon = True  # 设置为守护线程
        self.executor_thread.start()
        self.msg = String()
        self._node.publish_= self._node.create_publisher(String, 'SystemLog', 10)  # prevent unused variable warning'''

        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()

        self.vtkWidget = QVTKRenderWindowInteractor(self)
        self.vtkWidget.resize(600, 600)
        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'Registration')
        ui_dir = pathlib.Path(package_path) / 'share' / 'Registration' / 'resource'
        ui_file = ui_dir / 'Registration.ui'
        # Extend the widget with all attributes and children from UI file
        try:
            loadUi(str(ui_file.resolve()), self._widget)
            self._node.get_logger().info(f"UI file loaded successfully: {ui_file}")  # 输出加载成功的消息
        except Exception as e:
            self._node.get_logger().info(f"Failed to load UI file: {ui_file}. Error: {e}")

        if self._widget is None:
            self._node.get_logger().info('Node initialized')
        
        # Give QObjects reasonable names
        self._widget.setObjectName('RegistrationUI')
        self._widget.verticalLayout.insertWidget(0, self.vtkWidget)
        context.add_widget(self._widget)
	self.PrintLog('注册窗口已打开')
        self._widget.progressBar.setMinimum(0)  # 设置最小值
        self._widget.progressBar.setMaximum(100)  # 设置最大值
        self._widget.progressBar.setValue(0)  # 初始进度为 0
        self._widget.progressBar_2.setMinimum(0)  # 设置最小值
        self._widget.progressBar_2.setMaximum(100)  # 设置最大值
        self._widget.progressBar_2.setValue(0)  # 初始进度为 0

        self._widget.getpoints.clicked[bool].connect(self.getpoints)
        self._widget.registrate.clicked[bool].connect(self.registrate)
        self._widget.registrate.setEnabled(False)

        self._update_ui_timer = QTimer(self)
        self._update_ui_timer.timeout.connect(self._updateUI)
        self._update_ui_timer.start(100)
        
        
    def _updateUI(self):
        if self.points_node.point_cloud_received == True:
            self.points_node.point_cloud_received = False
            min_distance =  self.points_node.min_ditance
            self.node_thread1 = threading.Thread(target=self.points_node.model_show(self))
            self.node_thread1.daemon = True
            self.node_thread1.start() 
            self.PrintLog("已获取点云")
            self.PrintLog("相机位置：{:.3f}m".format(min_distance))
            if min_distance >= 0.1 and min_distance <= 0.50:

                self.re.get_RGBpoints_and_model(self.points_node.processed_pcd,self.points_node.VtkModel)
                self.PrintLog("相机位置处于正常位置")
                self._widget.registrate.setEnabled(True)
            else:
                self.PrintLog("相机不在正确位置")
            
        self._widget.progressBar.setValue(int(self.points_node.progress*100))
        self._widget.progressBar_2.setValue(int(self.re.progress*100))
        
    def shutdown(self):
        self.PrintLog('注册完成')
        self._update_ui_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def getpoints(self):
        self.points_node.receiveEable = 1
        self.PrintLog('正在获取点云')

    def registrate(self):
        self.re.start()
        self.registrateFlag = True
        self.PrintLog('正在配准')
        

    def AllButton(self,a):
        self._widget.setEnabled(a)

    def re_finished(self):
        self.registrateFlag = False
        self.PrintLog('配准完成')
        self.node_thread2 = threading.Thread(target=self.re.show_two_model(self))
        self.node_thread2.daemon = True
        self.node_thread2.start()
        self.re.Publish_tf()
        self.PrintLog('变换已发布')

    def PrintLog(self,Log):
        self.msg.data = Log
        self._node.publish_.publish(self.msg)
