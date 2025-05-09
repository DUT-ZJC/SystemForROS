from PyQt5.QtCore import QThread, QMutex
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from pathlib import Path
import vtk
import numpy as np
import sympy as sp
from scipy.interpolate import splrep, splev

model_path = Path.home() / 'easy_handeye2' / 'install' / 'path_plan' / 'share' / 'path_plan' / 'model' / 'vitual_model.stl'

class Path_Display(QThread):
    def __init__(self,Qwidget):
        super().__init__()
        self.VTKWindow = Qwidget.vtkWidget
        self.Qwidget = Qwidget
        self.index = 0  # 初始索引
        self.points_vtk = vtk.vtkPoints()
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("Colors")
        self.polydata_points = vtk.vtkPolyData()
        self.mapper = vtk.vtkPolyDataMapper()
        self.actor = vtk.vtkActor()
        self.index1 = 0
        self.update_needed = False
        self.mutex = QMutex()

    def set_index(self, index):
        self.mutex.lock()  # 获取锁
        self.index1 = index
        self.update_needed = True  # 设置更新标志
        self.mutex.unlock()  # 释放锁

    def run(self):
        while self.running:
            self.mutex.lock() 
            if self.update_needed:
                self.mutex.lock()   # 获取锁
                self.index = self.index1
                self.update_needed = False  # 重置更新标志
                self.mutex.unlock()  # 释放锁
                self.Update_Path_point()
            else:
                self.msleep(10)  # 如果不需要更新，线程休眠100毫秒


    def stop(self):
        """停止线程"""
        self.running = False  # 设置标志变量为False，退出循环
        self.quit()  # 退出线程
        self.wait()  # 等待线程结束

     ###############以下为可视化部分#############
    def Date_Init(self):
        # 创建一个STL读取器
        reader = vtk.vtkSTLReader()
        reader.SetFileName(model_path)
        reader.Update()
        self.polydata = reader.GetOutput()
        self.closest_point = self.Qwidget.center_point
        self.actpoint = self.Qwidget.actpoint
        self.rotation = self.Qwidget.rotation
        self.normal  = self.rotation[:,2]
        self.view()

    def display_normal(self):
        # 创建 vtkLineSource 对象
        line_source = vtk.vtkLineSource()
        line_source.SetPoint1(self.closest_point[1])
        line_source.SetPoint2([self.closest_point[1][i] + self.normal[i] * 10 for i in range(3)])  # 法线方向，长度为10

        # 创建 mapper 和 actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(line_source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.0, 0.0, 1.0)  # 设置法线的颜色为蓝色
        actor.GetProperty().SetLineWidth(3)  # 设置法线的宽度

        return actor

    def create_cylinder(self):
        """
        创建一个中空圆柱
        """
        # 创建圆柱源
        cylinder_source = vtk.vtkCylinderSource()
        cylinder_source.SetRadius(self.radius)
        cylinder_source.SetHeight(10.0)
        cylinder_source.SetResolution(50)  # 设置圆柱的分辨率
        cylinder_source.CappingOff()  # 创建中空圆柱

        # 创建变换对象
        transform = vtk.vtkTransform()
        cylinder_cener =  (self.closest_point[1][0]-4*self.normal[0],self.closest_point[1][1]-4*self.normal[1],self.closest_point[1][2]-4*self.normal[2])
        transform.Translate(cylinder_cener)  # 将圆柱移动到中心点

        # 计算旋转矩阵
        y_axis = np.array([0, 1, 0])
        direction = np.array(self.normal) / np.linalg.norm(self.normal)  # 归一化方向向量
        axis = np.cross(y_axis, direction)
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 0 else y_axis
        angle = np.arccos(np.dot(y_axis, direction)) * 180 / np.pi

        # 应用旋转变换
        transform.RotateWXYZ(angle, *axis)

        # 应用变换
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetInputConnection(cylinder_source.GetOutputPort())
        transform_filter.SetTransform(transform)
        transform_filter.Update()

        # 创建 mapper 和 actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.0, 1.0, 0.0)  # 设置圆柱的颜色为绿色
        actor.GetProperty().SetOpacity(0.5)  # 设置圆柱的透明度为半透明

        return actor

    def display_model_and_curve(self):
        highlight_points_actor = self.highlight_points_with_color(self.actpoint, self.outer_pointlist)
        self.renderer.AddActor(highlight_points_actor)

    def display_cylinder(self,normal_actor,cylinder_actor,renderWindow):

        modelMapper = vtk.vtkPolyDataMapper()
        modelMapper.SetInputData(self.polydata)

        modelActor = vtk.vtkActor()
        modelActor.SetMapper(modelMapper)
        modelActor.GetProperty().SetColor(0.7, 0.7, 0.7)  # 设置模型颜色
        modelActor.GetProperty().SetOpacity(0.7)

        # 创建渲染窗口和交互器
        self.renderer = vtk.vtkRenderer()
        renderWindow.GetRenderWindow().AddRenderer(self.renderer)
        renderWindowInteractor = renderWindow.GetRenderWindow().GetInteractor()

        # 添加模型、高亮点、法线和中空圆柱到 renderer
        self.renderer.AddActor(modelActor)
        self.renderer.AddActor(normal_actor)
        self.renderer.AddActor(cylinder_actor)

        # 添加自定义坐标系

        self.renderer.SetBackground(1, 1, 1)  # 设置背景颜色为白色
        self.renderer.ResetCamera()

        # 渲染和显示
        renderWindowInteractor.Initialize()
    

    def view(self):
        normal_actor = self.display_normal()
        cylinder_actor = self.create_cylinder()
        self.display_cylinder(normal_actor,cylinder_actor,self.VTKWindow)
    
    def Update_Path_point(self):
        self.points_vtk.Reset()
        self.colors.Reset()

        for i in range(self.index + 1):
            point = self.actpoint[i]
            self.points_vtk.InsertNextPoint(point)
            self.colors.InsertNextTuple3(255, 255, 0)  # 黄色

        self.polydata_points.SetPoints(self.points_vtk)
        self.polydata_points.GetPointData().SetScalars(self.colors)

        # 创建 vtkPolyVertex 对象
        vertices = vtk.vtkPolyVertex()
        vertices.GetPointIds().SetNumberOfIds(self.points_vtk.GetNumberOfPoints())
        for i in range(self.points_vtk.GetNumberOfPoints()):
            vertices.GetPointIds().SetId(i, i)

        # 创建 vtkCellArray 对象
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(vertices)

        self.polydata_points.SetVerts(cells)

        # 更新 mapper 和 actor
        self.mapper.SetInputData(self.polydata_points)
        self.actor.SetMapper(self.mapper)
        self.actor.GetProperty().SetPointSize(5)  # 设置高亮点的大小

        self.renderer.AddActor(self.actor)
        # 重新渲染
        self.VTKWindow.GetRenderWindow().Render()