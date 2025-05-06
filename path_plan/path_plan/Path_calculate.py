import vtk
import numpy as np
import sympy as sp
from scipy.interpolate import splrep, splev
from PyQt5.QtCore import QThread, Qt
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from PyQt5.QtGui import QPen, QColor
from pathlib import Path
import math

model_path = Path.home() / 'easy_handeye2' / 'install' / 'path_plan' / 'share' / 'path_plan' / 'model' / 'vitual_model.stl'

class Path_generation(QThread):
    def __init__(self):
        super().__init__()
        self.stl_file = model_path
        self.target_point = [0,0,0]  #加工的中心点
        self.radius = 0              #加工半径
        self.deviation = 0.7        #搜索偏差
        self.feed_in_raund = 0     #最小切深度mm
        self.point_num_oneraund = 100    

        self.progress = 0.0
        self.Interval = 0 #加工总角度
        self.point_num_total = 0                   #总点数
        self.actpoint = []                         #实际加工点
        self.actpointdict = {}                     #实际加工点字典，key为点坐标，value为法向量高度                     #一圈的进给量
        self.depth_in_firstraund = 0           #第一圈最大切深度，给一个初值
        self.depth_in_lastraund = 0                #最后一圈最大切深
        self.depthlist = list()                     #切深列表
        self.deep_positionlist = list()             
        self.rotation = None
        self.Length = 0

        self.polydata = None
        self.closest_point = list()                #加工点在模型中的实际点
        self.normal = None                         #加工点的法线
        self.inner_points = list()                 #搜索范围内的内圈点
        self.outer_points = list()
                         #搜索范围内的外圈点
        self.inner_depthlist = list()              #内圈点的切深度列表

        self.inner_pointlist = None                #插值得到的内圈点列表
        self.outer_pointlist = None                #插值得到的外圈点列表                     
        
        self.inner_heighdicts = {}                  #内圈点高度字典
        self.outer_heighdicts = {}                  #外圈点高度字典
        self.inner_spline = None                    #内圈点高度插值函数    
        self.outer_spline = None                    #外圈点高度插值函数


    def SetX(self,X):
        self.target_point[0] = X
    def SetY(self,Y):
        self.target_point[1] = Y
    def SetZ(self,Z):
        self.target_point[2] = Z
    def SetR(self,R):
        self.radius = R
    def SetD(self,D):
        self.feed_in_raund = D

    ###############以下为路径####################3
    def load_stl(self):
        # 创建一个STL读取器
        reader = vtk.vtkSTLReader()
        reader.SetFileName(self.stl_file)
        reader.Update()
        self.polydata = reader.GetOutput()


    def find_closest_point(self):
        # 查找最接近目标点的点
        picker = vtk.vtkPointPicker()
        picker.SetTolerance(1e-6)
        closest_point_id = -1
        min_distance = float('inf')
        for i in range(self.polydata.GetNumberOfPoints()):
            point = [0, 0, 0]
            self.polydata.GetPoint(i, point)
            distance = vtk.vtkMath.Distance2BetweenPoints(point, self.target_point)
            if distance < min_distance:
                min_distance = distance
                closest_point_id = i
                closest_point = point
        self.closest_point.append(closest_point_id)
        self.closest_point.append(closest_point)

    def compute_normals(self):
        # 计算法线
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(self.polydata)
        normals.ComputePointNormalsOn()
        normals.Update()
        normal_polydata = normals.GetOutput()
        self.normal = [0, 0, 0]
        normal_polydata.GetPointData().GetNormals().GetTuple(self.closest_point[0], self.normal)

    def calculate_point_distances(self):
        point_distance_dict = {}
        normal_vector = vtk.vtkVector3d(*self.normal)
        for i in range(self.polydata.GetNumberOfPoints()):
            point = [0, 0, 0]
            self.polydata.GetPoint(i, point)
            point_vector = vtk.vtkVector3d(point[0] - self.closest_point[1][0], point[1] - self.closest_point[1][1], point[2] - self.closest_point[1][2])
            cross_product = vtk.vtkVector3d.Cross(normal_vector, point_vector)
            distance = cross_product.Norm() / normal_vector.Norm()
            if self.radius-self.deviation <= distance <= self.deviation+self.radius:
                distance_difference = distance - self.radius  
                point_distance_dict[tuple(point)] = distance_difference
        return point_distance_dict
    
    def point_sorted_by_distance(self):
        points_within_range = self.calculate_point_distances()


        for point, distance in points_within_range.items():
            point_id = self.polydata.FindPoint(point)
            if 0 <= point_id <= 112718:
                self.inner_points.append((point, distance))
            elif 112718 < point_id <= 220206:  
                self.outer_points.append((point, distance))

        # 按偏差从小到大排序
        self.inner_points = sorted(self.inner_points, key=lambda item: item[1])
        self.outer_points = sorted(self.outer_points, key=lambda item: item[1])

    def match_negative_to_positive(self, points):
            """
            匹配负偏差点到最近的正偏差点，如果距离大于0.8mm，则舍弃该点对
            :param points: 点列表，每个元素为 (point, distance)
            :return: 匹配字典，键为负偏差点，值为最近的正偏差点列表（可能为空）
            """
            negative_points = [point for point, distance in points if distance < 0]
            positive_points = [point for point, distance in points if distance >= 0]

            pairs = {}
            for negative in negative_points:
                # 计算负偏差点到所有正偏差点的距离
                distances = [np.linalg.norm(np.array(negative) - np.array(pos)) for pos in positive_points]
                # 找到距离小于等于0.8mm的正偏差点
                valid_indices = [i for i, dist in enumerate(distances) if dist <= 0.75]
                if valid_indices:
                    # 如果有符合条件的点，按距离从小到大排序并取前两个
                    closest_indices = sorted(valid_indices, key=lambda i: distances[i])[:2]
                    closest_points = [positive_points[i] for i in closest_indices]
                    pairs[negative] = closest_points
                else:
                    # 如果没有符合条件的点，匹配结果为空列表
                    pairs[negative] = []

            return pairs
    

    def find_intersections(self, point_pairs):
        total_points = sum(len(points) for points in point_pairs.values())  # 计算总点数
        intersections = []
        
        for i, (point1, points) in enumerate(point_pairs.items(), start=1):  # 使用enumerate获取索引
            if len(points) == 2:
                line_eq = self.create_line_equation(point1, points[0])
                intersection_point = self.cylinder_intersection(line_eq, self.closest_point[1], self.normal)
                if intersection_point:
                    intersections.append(intersection_point)
                
                line_eq = self.create_line_equation(point1, points[1])
                intersection_point = self.cylinder_intersection(line_eq, self.closest_point[1], self.normal)
                if intersection_point:
                    intersections.append(intersection_point)
            
            if len(points) == 1:
                line_eq = self.create_line_equation(point1, points[0])
                intersection_point = self.cylinder_intersection(line_eq, self.closest_point[1], self.normal)
                if intersection_point:
                    intersections.append(intersection_point)
            
            # 更新进度
            self.progress += (1 / total_points) * 45.0  # 计算进度百分比

        return intersections

    def create_line_equation(self, point1, point2):
        # 创建直线方程
        x1, y1, z1 = point1
        x2, y2, z2 = point2
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        return (dx, dy, dz, x1, y1, z1)

    def cylinder_intersection(self, line_eq, center, normal):
        # 解析求解直线与圆柱的交点
        dx, dy, dz, x0, y0, z0 = line_eq
        r = self.radius
        x, y, z = sp.symbols('x y z')
        a = normal[2]*(y-center[1])-normal[1]*(z-center[2])
        b = normal[0]*(z-center[2])-normal[2]*(x-center[0])
        c = normal[1]*(x-center[0])-normal[0]*(y-center[1])

        eq1 = a**2 + b**2 + c**2 - r**2
        #0=dx(y-y0)-dy(x-x0)
        #0=dx(z-z0)-dz(x-x0)
        #0=dy(z-z0)-dz(y-y0)
        if dy == 0 and dz == 0:
            eq2 = dx*(y-y0)-dy*(x-x0)
            eq3 = dx*(z-z0)-dz*(x-x0)
        else :
            eq2 = dx*(z-z0)-dz*(x-x0)
            eq3 = dy*(z-z0)-dz*(y-y0)

        solutions = sp.solve((eq1, eq2, eq3), (x, y, z))

        numerical_solution1 = [float(sol.evalf()) for sol in solutions[0]]
        numerical_solution2 = [float(sol.evalf()) for sol in solutions[1]]

        distances1 = [np.linalg.norm(np.array(numerical_solution1) - np.array(line_eq[3:6]))]
        distances2 = [np.linalg.norm(np.array(numerical_solution2) - np.array(line_eq[3:6]))]
        
        if distances1[0] < distances2[0]:
            return tuple(numerical_solution1)
        else:
            return tuple(numerical_solution2)


    def find_closest_point_to_line(self, points, point1, point2):
        # 计算两个交点到直线两端点的距离，并选择较近的交点
        distances = [np.linalg.norm(np.array(p) - np.array(point1)) for p in points]
        closest_index = np.argmin(distances)
        return points[closest_index]

    
    def Curve_fitting(self,dicts):
        x_values = []
        y_values = []
        for value in dicts.values():
            x_values.append(value[1])
            y_values.append(value[0])
        x_values = np.array(x_values)
        y_values = np.array(y_values)
        #coefficients = np.polyfit(x_values, y_values, 50)
        tck = splrep(x_values, y_values, s=0)
        return tck

    
    def waypoint_generation(self,tck1,tck2,Bench_point):

        x_fit = np.linspace(0, self.Interval, self.point_num_total)

        angle = 0
        num = 0
        while angle*self.feed_in_raund/(np.pi*2) < splev(angle%(2*np.pi),tck2):
            point = np.array([self.radius*np.cos(angle),self.radius*np.sin(angle),0])
            point = np.dot(point,Bench_point[3])+Bench_point[4]
            depth = angle * self.feed_in_raund/(np.pi*2)
            point = point - depth*np.array(self.normal)
            self.actpoint.append(tuple(point))
            angle += 2*np.pi/self.point_num_oneraund
            num += 1
            if num < self.point_num_oneraund:
                if depth < splev(angle%(2*np.pi),tck2):
                    self.depthlist.append(0)
                else:
                    self.depthlist.append(depth - splev(angle%(2*np.pi),tck1))
            elif num - self.point_num_oneraund > 0:
                if self.depthlist[num - self.point_num_oneraund] ==0:
                    self.depthlist.append(depth - splev(angle%(2*np.pi),tck1))
                else:
                    self.depthlist.append(self.feed_in_raund)

            self.deep_positionlist.append(depth)

        for i in range(self.point_num_oneraund):
            point = np.array([self.radius*np.cos(angle),self.radius*np.sin(angle),0])
            point = np.dot(point,Bench_point[3])+Bench_point[4]
            depth = angle * self.feed_in_raund/(np.pi*2)
            if depth > splev(angle%(2*np.pi),tck2):
                point = point -splev(angle%(2*np.pi),tck2)*np.array(self.normal)
                self.inner_depthlist.append(0)
                self.depthlist.append(splev(angle%(2*np.pi),tck2)-self.depthlist[num-self.point_num_oneraund])
                self.deep_positionlist.append(splev(angle%(2*np.pi),tck2))
            else:
                point = point - depth*np.array(self.normal)
                self.inner_depthlist.append(splev(angle%(2*np.pi),tck2) - depth)
                self.depthlist.append(self.feed_in_raund)
                self.deep_positionlist.append(depth)

            self.actpoint.append(tuple(point))
            angle += 2*np.pi/self.point_num_oneraund
            num += 1

        i = 0
        while max(self.inner_depthlist)> 0:
            point = np.array([self.radius*np.cos(angle),self.radius*np.sin(angle),0])
            point = np.dot(point,Bench_point[3])+Bench_point[4]
            depth = angle * self.feed_in_raund/(np.pi*2)
            if depth > splev(angle%(2*np.pi),tck2):
                point = point -splev(angle%(2*np.pi),tck2)*np.array(self.normal)
                self.inner_depthlist[i] = 0
                self.depthlist.append(splev(angle%(2*np.pi),tck2)-self.depthlist[num-self.point_num_oneraund])
                self.deep_positionlist.append(splev(angle%(2*np.pi),tck2))
            else:
                point = point - depth*np.array(self.normal)
                self.inner_depthlist[i] = (splev(angle%(2*np.pi),tck2) - depth)
                self.depthlist.append(self.feed_in_raund)
                self.deep_positionlist.append(depth)
            self.actpoint.append(tuple(point))
            angle += 2*np.pi/self.point_num_oneraund
            i += 1
            i = i % self.point_num_oneraund
            num += 1
        
        self.point_num_total = len(self.actpoint)
        self.Interval = angle - 2*np.pi/self.point_num_oneraund

            


    def path_generation(self):
    
        self.point_sorted_by_distance()
      
        inner_pairs = self.match_negative_to_positive(self.inner_points)
        outer_pairs = self.match_negative_to_positive(self.outer_points)
        self.inner_pointlist = self.find_intersections(inner_pairs)
        self.outer_pointlist = self.find_intersections(outer_pairs)

        #计算高度
        for point in self.inner_pointlist:
            point_vector = np.array((point[0] - self.closest_point[1][0]-10*self.normal[0], point[1] - self.closest_point[1][1]-10*self.normal[1], point[2] - self.closest_point[1][2]-10*self.normal[2]))
            distance = np.dot(point_vector, self.normal)
            result = np.array(tuple(x - distance*y for x, y in zip(point_vector, self.normal)))
            self.inner_heighdicts[point] = [abs(distance), result]
        self.inner_heighdicts = dict(sorted(self.inner_heighdicts.items(), key=lambda item: item[1][0]))

        Bench_point = [0,100.0,0,0,0,0,1000.0]
        #Bench_point[0]：外壳最高点坐标
        #Bench_point[1]：外壳最高点在圆柱轴线的投影
        #Bench_point[2]：基准点半径向量
        #Bench_point[3]：以基准点方向为x方向，圆心为原点的坐标系的旋转矩阵
        #Bench_point[4]：上边坐标系的平移向量
        #Bench_point[5]：内壳最高点角度坐标
        #Bench_point[6]：内壳最高点高度
        for point in self.outer_pointlist:  
            point_vector = np.array((point[0] - self.closest_point[1][0]-10*self.normal[0], point[1] - self.closest_point[1][1]-10*self.normal[1], point[2] - self.closest_point[1][2]-10*self.normal[2]))
            distance = np.dot(point_vector, self.normal)
            result = np.array(tuple(x - distance*y for x, y in zip(point_vector, self.normal)))
            self.outer_heighdicts[point] = [abs(distance), result]
            if abs(distance) < Bench_point[1]:
                Bench_point[1] = abs(distance)
                Bench_point[0] = point
                Bench_point[2] = result    
        self.outer_heighdicts = dict(sorted(self.outer_heighdicts.items(), key=lambda item: item[1][0]))

        Bench_point[3] = np.array([Bench_point[2],np.cross(self.normal,Bench_point[2]),np.array(self.normal)*5])
        Bench_point[4] = np.array(Bench_point[0])-np.array(Bench_point[2])
        Bench_point[3] = Bench_point[3]/5

        self.rotation = Bench_point[3].T

        for key, value in self.inner_heighdicts.items():
            Bench_vector = Bench_point[2]
            cos = np.dot(Bench_vector,value[1])/25
            vector = np.cross(Bench_vector,value[1])
            if np.dot(vector,self.normal) > 0:
                sin = np.linalg.norm(vector)/25
            else:
                sin = -np.linalg.norm(vector)/25
            angle = np.arctan2(sin, cos)
            if angle < 0:
                angle += 2*np.pi
            if Bench_point[6] > value[0]:
                Bench_point[6] = value[0]
                Bench_point[5] = key
            if self.depth_in_lastraund < value[0]:
                self.depth_in_lastraund = value[0] 

            self.inner_heighdicts[key] = [value[0] - Bench_point[1], angle]

        Bench_point[6] = Bench_point[6]-Bench_point[1]
        self.depth_in_lastraund = self.depth_in_lastraund - Bench_point[1]

        Bench_point[5] = self.inner_heighdicts[Bench_point[5]][1]
        self.progress = 95.0
        #计算角度
        for key, value in self.outer_heighdicts.items():
            if key == Bench_point[0]:
                self.outer_heighdicts[key] = [0, 0]
                continue
            point = Bench_point[2]
            cos = np.dot(point,value[1])/25
            vector = np.cross(point,value[1])
            if np.dot(vector,self.normal) > 0:
                sin = np.linalg.norm(vector)/25
            else:
                sin = -np.linalg.norm(vector)/25
            angle = np.arctan2(sin, cos)
            if angle < 0:
                angle += 2*np.pi
            if self.depth_in_firstraund < value[0]:
                self.depth_in_firstraund = value[0]
            self.outer_heighdicts[key] = [value[0] - Bench_point[1], angle]

        self.inner_heighdicts = dict(sorted(self.inner_heighdicts.items(), key=lambda item: item[1][1]))
        
        self.outer_heighdicts = dict(sorted(self.outer_heighdicts.items(), key=lambda item: item[1][1]))

        print(f"depth_in_firstraund:{self.depth_in_firstraund}")
        self.depth_in_firstraund = self.depth_in_firstraund - Bench_point[1]
        self.depth_in_lastraund  = self.depth_in_lastraund - Bench_point[6]



        self.inner_spline = self.Curve_fitting(self.inner_heighdicts)
        self.outer_spline = self.Curve_fitting(self.outer_heighdicts)
        print(f"feed_in_raund:{self.feed_in_raund}")
        print(f"depth_in_firstraund:{self.depth_in_firstraund}")
        print(f"depth_in_lastraund:{self.depth_in_lastraund}")

        self.waypoint_generation(self.outer_spline,self.inner_spline,Bench_point)
        print(f"Number of points in all raund:{len(self.actpoint)}")

        self.progress = 100.0
    

    def curve_length(self,points):
        points = np.array(points)
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        length = np.sum(distances)
        return length

    def run(self):
        self.path_generation()
        self.Length = self.curve_length(self.actpoint)

    def show_chat(self,chartView):
        self.chart = QChart()
        self.chart.setTitle("路径深度图")
        chartView.setChart(self.chart)

        x_fit = np.linspace(0, np.pi*2, self.point_num_oneraund)
        y1_fit = splev(x_fit,self.inner_spline)
        y2_fit = splev(x_fit,self.outer_spline)

        # 创建X轴和Y轴
        self.axisX = QValueAxis()
        self.axisX.setRange(0, np.pi*2)
        self.axisX.setLabelFormat("%i")
        self.axisX.setTitleText("角度")
        self.chart.addAxis(self.axisX, Qt.AlignBottom)

        self.axisY = QValueAxis()
        self.axisY.setRange(0, math.ceil(max(y1_fit)))
        self.axisY.setLabelFormat("%i")
        self.axisY.setTitleText("深度(mm)")
        self.chart.addAxis(self.axisY, Qt.AlignLeft)

        # 创建折线系列并添加数据点

        self.print_line(x_fit,y2_fit,label="外曲面深度", color="blue")
        raund = len(self.depthlist)//self.point_num_oneraund
        for i in range(raund):
            y_fit = self.deep_positionlist[i*self.point_num_oneraund:(i+1)*self.point_num_oneraund]
            self.print_line(x_fit,y_fit)
        a = self.point_num_total%self.point_num_oneraund
        self.print_line(x_fit[0:a],self.deep_positionlist[raund*self.point_num_oneraund:])
        self.print_line(x_fit,y1_fit,label="内曲面深度", color="red")

    def print_line(self,x, y,label = "high",color = 'black'):
        # 将NumPy数组中的数据添加到QLineSeries中
        series = QLineSeries()
        if label is not "high" :
            series.setName(label)
        for i in range(len(self.deep_positionlist)):
            series.append(x[i], y[i])
        pen = QPen(QColor(color))  # 创建一个画笔
        series.setPen(pen)
        self.chart.addSeries(series)
        series.attachAxis(self.axisX)
        series.attachAxis(self.axisY)


    ###############以下为可视化部分#############
    def highlight_points_with_color(self, sorted_inner_points, sorted_outer_points):
        # 创建 vtkPoints 对象
        points_vtk = vtk.vtkPoints()
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")

        # 根据分类设置颜色
        for point in sorted_inner_points:
            points_vtk.InsertNextPoint(point)
            colors.InsertNextTuple3(255, 255, 0)  # 黄色

        for point in sorted_outer_points:
            points_vtk.InsertNextPoint(point)
            colors.InsertNextTuple3(255, 0, 0)  # 红色

        # 创建 vtkPolyVertex 对象
        vertices = vtk.vtkPolyVertex()
        vertices.GetPointIds().SetNumberOfIds(points_vtk.GetNumberOfPoints())
        for i in range(points_vtk.GetNumberOfPoints()):
            vertices.GetPointIds().SetId(i, i)

        # 创建 vtkCellArray 对象
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(vertices)

        # 创建 vtkPolyData 对象
        polydata_points = vtk.vtkPolyData()
        polydata_points.SetPoints(points_vtk)
        polydata_points.SetVerts(cells)
        polydata_points.GetPointData().SetScalars(colors)

        # 创建 mapper 和 actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata_points)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(5)  # 设置高亮点的大小

        return actor 

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

    def display_model_and_curve(self, highlight_points_actor):
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
    
    """"
    def add_custom_axes(self, renderer):
        # 定义坐标系的原点
        origin = self.closest_point[1]

        # 定义坐标系的三个向量
        axes = np.array([
            [0.18151281, -0.97768725, -0.10573912],
            [-0.84129337, -0.09870866, -0.53149042],
            [0.50919402, 0.18542999, -0.84043866]
        ])

        # 创建三条线表示 x, y, z 轴
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # 红色、绿色、蓝色
        for i in range(3):
            line_source = vtk.vtkLineSource()
            line_source.SetPoint1(origin)
            line_source.SetPoint2(origin + axes[i] * 10)  # 将向量放大 10 倍以便更清晰显示

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(line_source.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(colors[i])
            actor.GetProperty().SetLineWidth(2)  # 设置线宽

            renderer.AddActor(actor)
    """
    

    def view(self,VTKWidget):
        self.load_stl()
        self.find_closest_point()
        self.compute_normals()
        normal_actor = self.display_normal()
        cylinder_actor = self.create_cylinder()
        self.display_cylinder(normal_actor,cylinder_actor,VTKWidget)

    def show_modle(self):
        highlight_points_actor = self.highlight_points_with_color(self.actpoint, self.outer_pointlist)
        self.display_model_and_curve(highlight_points_actor)



def main():
    stl_file = "合并后的STL.stl"
    target_point = [208.84, 67.35, 11.66]
    radius = 5.0
    deviation = 0.7

    # 创建Path_generation类的实例
    path_gen = Path_generation(stl_file, target_point, radius, deviation,1.0)

    # 加载STL文件
    path_gen.path_generation()
    path_gen.show_modle()

    # 查找最接近目标点的点
   

if __name__ == "__main__":
    main()
