import math
import pathlib
import numpy as np
import transforms3d as tfs
from std_msgs.msg import String
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer

from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from rclpy.node import Node

from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis

from PyQt5.QtWidgets import QWidget
from std_msgs.msg import String
from path_plan.Path_calculate import Path_generation
import threading
from . import Path_calculate

class RqtPath_planWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtPath_planWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context
        self._node = context.node
        self.path_cal = Path_calculate.Path_generation()
        self.path_cal.finished.connect(self.cal_finished)

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
        self.vtkWidget.resize(500, 600)

        self.chartView = QChartView()
        self.chartView.resize(700, 600)

        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'path_plan')
        ui_dir = pathlib.Path(package_path) / 'share' / 'path_plan' / 'resource'
        ui_file = ui_dir / 'Path_Plan.ui'
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
        self._widget.infoWidget.setObjectName('RegistrationUI')
        self._widget.verticalLayout_6.insertWidget(0, self.vtkWidget)
        self._widget.verticalLayout_6.insertWidget(1, self.chartView)
        context.add_widget(self._widget)
        self.PrintLog('规划窗口已打开')

        self._widget.progressBar.setMinimum(0)  # 设置最小值
        self._widget.progressBar.setMaximum(100)  # 设置最大值
        self._widget.progressBar.setValue(0)  # 初始进度为 0
    

        self._widget.plan_start.clicked[bool].connect(self.path_calculate)
        self._widget.publish_path.clicked[bool].connect(self.path_publish)
        self._widget.Look.clicked[bool].connect(self.Look)

        self._widget.publish_path.setEnabled(False)
        self._widget.plan_start.setEnabled(False)
        self._widget.Look.setEnabled(False)

        self._widget.X_position.valueChanged.connect(self.XValueChanged)
        self._widget.Y_position.valueChanged.connect(self.YValueChanged)
        self._widget.Z_position.valueChanged.connect(self.ZValueChanged)
        self._widget.radius.valueChanged.connect(self.RValueChanged)
        self._widget.min_cutting_depth.valueChanged.connect(self.DValueChanged)

        self._update_ui_timer = QTimer(self)
        self._update_ui_timer.timeout.connect(self._updateUI)
        self._update_ui_timer.start(100)
        
        
    def _updateUI(self):

        if (    self._widget.X_position.value() is not 0
            and self._widget.Y_position.value() is not 0
            and self._widget.Z_position.value() is not 0
            and self._widget.radius.value() is not 0
            and self._widget.min_cutting_depth.value() is not 0):

            self._widget.plan_start.setEnabled(True)
            self._widget.Look.setEnabled(True)

        self._widget.progressBar.setValue(int(self.path_cal.progress))
        
    def shutdown(self):
        self.PrintLog('规划完成')
        self._update_ui_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def path_calculate(self):
        self.path_cal.start()
        self.PrintLog('正在获取路径')

    def path_publish(self):
        pass
        
    def cal_finished(self):
        self.node_thread1 = threading.Thread(target=self.path_cal.show_modle())
        self.node_thread1.daemon = True
        self.node_thread2 = threading.Thread(target=self.path_cal.show_chat(self.chartView))
        self.node_thread2.daemon = True
	self.PrintLog('路径已生成')
        self.node_thread1.start()
        self.node_thread2.start()

    def Look(self):
        self.node_thread = threading.Thread(target=self.path_cal.view(self.vtkWidget))
        self.node_thread.daemon = True

    def PrintLog(self,Log):
        self.msg.data = Log
        self._node.publish_.publish(self.msg)

    def XValueChanged(self,value):
        self.path_cal.SetX(value)
    def YValueChanged(self,value):
        self.path_cal.SetY(value)
    def ZValueChanged(self,value):
        self.path_cal.SetZ(value)
    def RValueChanged(self,value):
        self.path_cal.SetR(value)
    def DValueChanged(self,value):
        self.path_cal.SetD(value)
