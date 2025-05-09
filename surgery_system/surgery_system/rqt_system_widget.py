import math
import pathlib
import numpy as np
import transforms3d as tfs
from std_msgs.msg import String
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QStandardItemModel, QStandardItem
import copy
import sys
from rqt_gui.main import Main
import subprocess  # 导入 subprocess 模块
from datetime import datetime
from collections import deque

class RqtSystemWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtSystemWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context
        self.Log = deque(maxlen=10) #msglist
        self._node = context.node
        subprocess.Popen(['ros2', 'launch', 'easy_handeye2','publish.launch.py'])
        
        self._node.subscription = self._node.create_subscription(
            String,
            'SystemLog',
            self.PrintLog,
            10)
        self._node.subscription  # prevent unused variable warning'''

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
        self._widget2 = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'camera')
        ui_dir = pathlib.Path(package_path) / 'share' / 'camera' / 'resource'
        ui_file = ui_dir / 'system.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(str(ui_file.resolve()), self._widget2)
        try:
            loadUi(str(ui_file.resolve()), self._widget)
            self._node.get_logger().info(f"UI file loaded successfully: {ui_file}")  # 输出加载成功的消息
        except Exception as e:
            self._node.get_logger().info(f"Failed to load UI file: {ui_file}. Error: {e}")

        if self._widget is None:
            self._node.get_logger().info('Node initialized')
        
        # Give QObjects reasonable names
        self._widget.setObjectName('SystemUI')

        context.add_widget(self._widget)

        self._widget.calibrate.clicked[bool].connect(self.calibrate)
        self._widget.Registration.clicked[bool].connect(self.Registration)
        self._widget.planning.clicked[bool].connect(self.planning)
        self._widget.act.clicked[bool].connect(self.act)

        self._update_ui_timer = QTimer(self)
        self._update_ui_timer.timeout.connect(self._updateUI)
        self._update_ui_timer.start(100)

        self.model = QStandardItemModel(0,2)
        self.model.setHorizontalHeaderLabels(['----Log Message----', '--------Time-------'])
        self._widget.statetxt.setModel(self.model)
        self._widget.statetxt.resizeColumnsToContents()  # 根据内容自动调整列宽

        
    def _updateUI(self):
        if self.Log:
            current_time = datetime.now().strftime("%H:%M:%S")
            # 创建行数据
            row = [QStandardItem(self.Log.popleft()), QStandardItem(current_time)]
            # 将行数据添加到模型
            self.model.appendRow(row)
            

    def shutdown(self):
        self._update_ui_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def calibrate(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        row = [QStandardItem("正在打开标定窗口"), QStandardItem(current_time)]
        self.model.appendRow(row)
        # 使用 subprocess 启动另一个插件
        subprocess.Popen(['ros2', 'launch', 'easy_handeye2', 'calibrate.launch.py'])

    def Registration(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        row = [QStandardItem("正在打开注册窗口"), QStandardItem(current_time)]
        self.model.appendRow(row)
        subprocess.Popen(['rqt', '--standalone', 'registration.rqt_registration.RqtRegistration'])
    
    def planning(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        row = [QStandardItem("正在打开规划窗口"), QStandardItem(current_time)]
        self.model.appendRow(row)
        subprocess.Popen(['rqt', '--standalone', 'path_plan.rqt_path_plan.RqtPath_plan'])

    def act(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        row = [QStandardItem("正在打开执行窗口"), QStandardItem(current_time)]
        self.model.appendRow(row)
        subprocess.Popen(['rqt', '--standalone', 'camera.rqt_systemui.RqtSystem'])

    def PrintLog(self, msg):
        self.Log.append(msg.data)
    
    def AllButton(self,a):
        self._widget.setEnabled(a)
        self._widget.setEnabled(a)
        self._widget.setEnabled(a)
        self._widget.setEnabled(a)
