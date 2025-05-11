import numpy as np
from sklearn.decomposition import PCA
from stl import mesh
import pickle
from sklearn.neighbors import KDTree
from . import vtk_show
import vtk
import open3d as o3d
from pathlib import Path
from PyQt5.QtCore import QThread, pyqtSignal
import subprocess  # 导入 subprocess 模块
from scipy.spatial.transform import Rotation as R

model_path = Path.home() / 'lbr-stack' / 'install' / 'registration' / 'share' / 'registration' / 'model' / 'outer_model.stl'

class PCA_ICP(QThread):
    def __init__(self):
        super().__init__()
        mesh_virtual = mesh.Mesh.from_file(model_path)
        self.points_virtual = np.vstack((mesh_virtual.v0, mesh_virtual.v1, mesh_virtual.v2))
        self.points_virtual = np.unique(self.points_virtual, axis=0)/1000.0
        self.progress = 0.0
        self.transform_virtual_to_real = np.eye(4)
        finished = pyqtSignal()

    def get_RGBpoints_and_model(self,points_real,real_vtk_model):
        self.real_model = real_vtk_model
        self.RGBpoints_real = points_real
        self.points_real = np.asarray(self.RGBpoints_real.points)

    def PCA(self):
        # 重置进度
        center_virtual, principal_axes_virtual = self.compute_center_and_pca(self.points_virtual, 0)
        center_real, principal_axes_real = self.compute_center_and_pca(self.points_real, 1)

        transform_virtual = self.create_transform_matrix(center_virtual, principal_axes_virtual)

        transform_real = self.create_transform_matrix(center_real, principal_axes_real)

        transform_world_to_real = transform_real
        transform_virtual_to_world = np.linalg.inv(transform_virtual)

        self.Init_transform_virtual_to_real = transform_world_to_real @ transform_virtual_to_world
        self.progress = 0.05  # 更新进度

    def compute_center_and_pca(self,points,a):
        # 计算重心
        center = np.mean(points, axis=0)
        # 中心化点云
        points_centered = points - center
        # 进行PCA分析
        pca = PCA(n_components=3)
        pca.fit(points_centered)
        # 提取特征向量
        principal_axes = pca.components_

        det = np.linalg.det(principal_axes)

        # 如果行列式为 -1，调整某一行的符号
        if det < 0:
            principal_axes[2, :] *= -1  # 取反第三行
            det = np.linalg.det(principal_axes)
            print(f"调整后的 PCA 特征向量矩阵的行列式为: {det}")
            
        if a == 1:
            principal_axes[0] = -principal_axes[0]
            principal_axes[1] = principal_axes[1]
            principal_axes[2] = -principal_axes[2]

        print("主成分：\n", principal_axes)
        principal_axes = principal_axes.T
        print("重心：\n", center)
        return center, principal_axes

    def create_transform_matrix(self,center, axes):
        # 旋转矩阵
        rotation_matrix = axes
        # 平移矩阵
        translation_matrix = center

        # 坐标系变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation_matrix
        print("坐标系变换矩阵：\n", transform_matrix)
        return transform_matrix

    def ICP(self, max_iterations=500, tolerance=1e-10):
        # 重置进度
        self.progress = 0.10  
        transformation = self.Init_transform_virtual_to_real

        for iteration in range(max_iterations):
            # 应用当前变换矩阵到源点云
            transformed_source_points = np.dot(self.points_virtual, transformation[:3, :3].T) + transformation[:3, 3]
            
            # 构建目标点云的 KD-tree
            transformed_source_points_tree = KDTree(transformed_source_points)
            
            # 查询最近邻点
            distances, indices = transformed_source_points_tree.query(self.points_real, k=1)
            
            # 计算刚体变换
            source_center = np.mean(transformed_source_points[indices.flatten()], axis=0)
            target_center = np.mean(self.points_real, axis=0)

            source_points_centered = transformed_source_points[indices.flatten()] - source_center
            target_points_centered = self.points_real - target_center
            
            H = np.dot(source_points_centered.T, target_points_centered)

            U, S, Vt = np.linalg.svd(H)
            R = np.dot(Vt.T, U.T)
            
            if np.linalg.det(R) < 0:
                Vt[2, :] *= -1
                R = np.dot(Vt.T, U.T)
            
            t = np.mean(self.points_real - np.dot(transformed_source_points[indices.flatten()], R.T), axis=0)
            
            new_transformation = np.eye(4)
            new_transformation[:3, :3] = R
            new_transformation[:3, 3] = t
            new_transformation = np.dot(new_transformation, transformation)
            
            if np.linalg.norm(new_transformation - transformation) < tolerance:
                print("收敛了！")
                break
            
            transformation = new_transformation

            # 更新进度
            self.progress = 0.1 + (iteration / max_iterations) * 0.9

        self.transform_virtual_to_real = transformation
        self.progress = 1.0  # 完成进度

    def translate(self):
        mesh = o3d.io.read_triangle_mesh(model_path)  # 替换为你的 STL 文件路径

        # 获取点云数据
        points = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        # 将点的坐标单位从毫米转换为米
        points /= 1000.0

        # 应用变换矩阵
        transformed_points = np.dot(points, self.transform_virtual_to_real[:3, :3].T) + self.transform_virtual_to_real[:3, 3]

        # 创建 vtkPolyData 对象
        vtk_poly_data = vtk.vtkPolyData()

        # 创建 vtkPoints 对象
        vtk_points = vtk.vtkPoints()
        for point in transformed_points:
            vtk_points.InsertNextPoint(point)

        # 创建 vtkCellArray 对象
        vtk_cells = vtk.vtkCellArray()
        for triangle in triangles:
            vtk_triangle = vtk.vtkTriangle()
            vtk_triangle.GetPointIds().SetId(0, triangle[0])
            vtk_triangle.GetPointIds().SetId(1, triangle[1])
            vtk_triangle.GetPointIds().SetId(2, triangle[2])
            vtk_cells.InsertNextCell(vtk_triangle)

        # 设置点和单元
        vtk_poly_data.SetPoints(vtk_points)
        vtk_poly_data.SetPolys(vtk_cells)
        return vtk_poly_data
    
    def run(self):
        self.PCA()
        self.ICP()

    def show_two_model(self,QWidget):
        renderer = QWidget.vtkWidget.GetRenderWindow().GetRenderers().GetFirstRenderer()
        QWidget.vtkWidget.GetRenderWindow().RemoveRenderer(renderer)
        self.ren = vtk_show.create_renderer()
        QWidget.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = QWidget.vtkWidget.GetRenderWindow().GetInteractor()
        vtk_show.add_actor(self.ren,self.real_model)
        vtk_show.add_actor(self.ren,self.translate(),1)
        self.ren.ResetCamera()
        self.iren.Initialize()
    
    def Publish_tf(self,camera_to_base):
        
        translation = np.dot(camera_to_base,self.transform_virtual_to_real)

        # 提取旋转矩阵
        rotation_matrix = self.transform_virtual_to_real[:3, :3]

        # 将旋转矩阵转换为四元数
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        # 构造 static_transform_publisher 的命令
        frame_id = "lbr_link_0"  # 父坐标系
        child_frame_id = "virtual_frame"  # 子坐标系
        command = [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", "{:.6f}".format(translation[0]),
            "--y", "{:.6f}".format(translation[1]),
            "--z", "{:.6f}".format(translation[2]),
            "--qx", "{:.6f}".format(quaternion[0]),
            "--qy", "{:.6f}".format(quaternion[1]),
            "--qz", "{:.6f}".format(quaternion[2]),
            "--qw", "{:.6f}".format(quaternion[3]),
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id
        ]
        # 使用 subprocess.Popen 启动 static_transform_publisher
        subprocess.Popen(command)



