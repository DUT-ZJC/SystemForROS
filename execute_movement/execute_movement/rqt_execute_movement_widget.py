import yaml
import pathlib
import numpy as np
import transforms3d as tfs
from std_msgs.msg import String
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from PyQt5 import uic
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt

from rclpy.node import Node
from PyQt5.QtWidgets import QWidget
from std_msgs.msg import String,Int64
import threading
from pathlib import Path
from execute_movement.Robot_move_client import RobotClient
from execute_movement.Image_dispalay import Image_Display
from execute_movement.Path_display import Path_Display
import subprocess  # 导入 subprocess 模块

save_path = Path.home() / '.ros2'/'surgery'/'path.yaml'

class RqtExecute_MovementWidget(QWidget):
    def __init__(self, parent, context):
        super(RqtExecute_MovementWidget, self).__init__()
        self._parent = parent
        self._plugin_context = context
        self._node = context.node
        self._node.target_index = 0
        self._node.current_index = 0
        
        self.msg = String()
        self._node.publish_= self._node.create_publisher(String, 'SystemLog', 10)  # prevent unused variable warning'''
        self._node.index_listener = self._node.create_subscription(Int64,"point_index",self.listener_callback,10)
        
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

        self.path_load()

        self._widget = QWidget()
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        self.vtkWidget.resize(500, 600)
        

        # Get path to UI file which should be in the "resource" folder of this package
        _, package_path = get_resource('packages', 'execute_movement')
        ui_dir = pathlib.Path(package_path) / 'share' / 'execute_movement' / 'resource'
        ui_file = ui_dir / 'act.ui'
        # Extend the widget with all attributes and children from UI file
        try:
            loadUi(str(ui_file.resolve()), self._widget)
            self._node.get_logger().info(f"UI file loaded successfully: {ui_file}")  # 输出加载成功的消息
        except Exception as e:
            self._node.get_logger().info(f"Failed to load UI file: {ui_file}. Error: {e}")

        if self._widget is None:
            self._node.get_logger().info('Node initialized')
        
        # Give QObjects reasonable names
        self._widget.setObjectName('actUI')
        self._widget.verticalLayout.insertWidget(0, self.vtkWidget)
        context.add_widget(self._widget)
        self.PrintLog('规划窗口已打开')

        self._widget.progressBar.setMinimum(0)  # 设置最小值
        self._widget.progressBar.setMaximum(100)  # 设置最大值
        self._widget.progressBar.setValue(0)  # 初始进度为 0

        self._widget.Progress_Set.setMinimum(0)  # 设置最小值
        self._widget.Progress_Set.setMaximum(100)  # 设置最大值
        self._widget.Progress_Set.setValue(0)

        self._widget.Launch_Emu.clicked[bool].connect(self.Emu_launch)
        self._widget.Launch_Rob.clicked[bool].connect(self.Rob_Launch_Rob)
        self._widget.move_to_start.clicked[bool].connect(self.move_to_start)
        self._widget.press_to_move.clicked[bool].connect(self.onButtonPressed)
        self._widget.press_to_move.released.connect(self.onButtonReleased)
        self._widget.move_begin.clicked[bool].connect(self.move_begin)
        self._widget.move_stop.clicked[bool].connect(self.move_stop)
        self._widget.Stop_and_Back.clicked[bool].connect(self.Stop_and_Back)
        self._widget.Rob_Enable.stateChanged.connect(self.RobStateChanged)
        self._widget.Progress_Set.valueChanged.connect(self.Set_Progress)

        self._widget.ImageLabel.setText("等待图像加载...")  # 设置初始文本
        self._widget.ImageLabel.setScaledContents(True)  # 确保图像自动缩放
        self._widget.ImageLabel.resize(500, 600)  # 设置初始大小
        
        self.Image_thread = Image_Display(self._widget.ImageLabel)
        self.Image_thread.start()


        self.Path_thread = Path_Display(self)
        self.Path_thread.Date_Init()
        self.Path_thread.start()

        self._widget.move_to_start.setEnabled(False)
        self._widget.press_to_move.setEnabled(False)
        self._widget.move_begin.setEnabled(False)


        self._update_ui_timer = QTimer(self)
        self._update_ui_timer.timeout.connect(self._updateUI)
        self._update_ui_timer.start(100)
        
        
    def _updateUI(self):

        self._widget.progressBar.setValue(int(self._node.current_index/len(self.actpoint)*100))
        
    def shutdown(self):
        self.PrintLog('规划完成')
        self._update_ui_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
    
    

    def path_load(self):

            # 设置默认加载路径
            load_path = Path.home() / ".ros2" / "surgery" / "path.yaml"
            
            load_path = Path(load_path)
            
            # 检查文件是否存在
        

            # 加载 YAML 文件
            with open(load_path, 'r') as f:
                path_data = yaml.safe_load(f)

            # 转换数据格式
            self.actpoint = path_data['actpoint']
            self.rotation = [np.array(r) for r in path_data['rotation']]
            self.center_point = np.array(path_data['center_point'][0])
            self.deep_position = path_data['deep_position']
            self.thickness = path_data['thickness']
            if 'radius' in path_data:
                self.radius = path_data['radius']
            else:
                self.radius = 5.0  # 默认值

            self.PrintLog(f" 路径加载完成 ")
            return True
            
    


    def Emu_launch(self):
        subprocess.Popen(["ros2", "launch", "lbr_bringup", "mock.launch.py",
    "moveit:=true", "model:=iiwa14"])
        subprocess.Popen(["ros2", "launch", "lbr_bringup", "rviz.launch.py"])
        subprocess.Popen(["ros2", "launch", "lbr_bringup", "move_group.launch.py",
    "mode:=mock", "model:=iiwa14"])
        
        subprocess.Popen(["ros2", "launch", "robot_act_move", "robot_server.launch.py",
    "mode:=mock", "model:=iiwa14"])
        self.client = RobotClient(self._node)
        self.PrintLog(f" 仿真已启动 ")

    def Rob_Launch_Rob(self):
        pass
    def move_to_start(self):
        self.client.MoveToStart()

    def onButtonPressed(self):
        self.client.KeepMove()

    def onButtonReleased(self):
        self.client.StopMove()

    def move_begin(self):
        self.client.AutoMoveStart()

    def move_stop(self):
        self.client.StopMove()

    def Stop_and_Back(self):
        self.client.StopAndBack()

    def RobStateChanged(self,state):
        if state == Qt.Checked:
            print("Checkbox is checked")
        elif state == Qt.Unchecked:
            print("Checkbox is unchecked")

    def Set_Progress(self,value):
        self._node.target_index = int(value/100*len(self.actpoint))





    def listener_callback(self,msg):
        self._node.current_index = msg.data

    def PrintLog(self,Log):
        self.msg.data = Log
        self._node.publish_.publish(self.msg)


