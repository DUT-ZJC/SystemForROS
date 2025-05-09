import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import struct
import sys
from open3d.visualization import gui
import vtk
from PyQt5.QtGui import QWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt

from . import vtk_show

class PointCloudViewerNode(Node):
    def __init__(self):
        super().__init__('point_cloud_viewer_node')
        self.point_cloud_received = False
        self.points = None
        self.colors = None
        self.processed_pcd = None

        self.progress = 0.0  # 初始化进度为 0.0

        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            'points2',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.receiveEable = 0

    def listener_callback(self, msg):
        if self.receiveEable == 1:
            self.receiveEable = 0
            points = []
            colors = []
            # 使用sensor_msgs_py读取点云数据，指定字段名称
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
                x = point[0]
                y = point[1]
                z = point[2]
                rgb = point[3]

                # 将rgb浮点数转换为整数
                rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]

                # 从rgb整数中提取红、绿、蓝通道值
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF

                points.append([x, y, z])
                colors.append([r / 255.0, g / 255.0, b / 255.0])  # 归一化到[0, 1]

            self.points = np.array(points)
            self.colors = np.array(colors)

            # 更新进度为 0.1（10%）
            self.progress = 0.1

            # 处理点云
            self.process_point_cloud()
            self.point_cloud_received = True

    def process_point_cloud(self):
        # 将点云数据转换为Open3D的PointCloud对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)

        # 更新进度为 0.2（20%）
        self.progress = 0.3

        # 使用RANSAC进行平面分割
        distance_threshold = 0.010  # 距离阈值
        ransac_n = 3  # RANSAC每次迭代中用于拟合模型的点数
        num_iterations = 1000000  # RANSAC迭代次数

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )

        # 更新进度为 0.4（40%）
        self.progress = 0.5

        # 提取平面外点
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        # 获取平面方程的参数
        a, b, c, d = plane_model

        # 计算原点代入平面方程的符号
        origin_side = np.sign(a * 0 + b * 0 + c * 0 + d)

        # 筛选与原点同一侧的点
        same_side_indices = []
        inliers_set = set(inliers)
        for i, point in enumerate(self.points):
            if i not in inliers_set:
                x, y, z = point
                point_side = np.sign(a * x + b * y + c * z + d)
                if point_side == origin_side:
                    same_side_indices.append(i)

        # 创建与原点同一侧的点云
        same_side_cloud = pcd.select_by_index(same_side_indices)

        # 更新进度为 0.6（60%）
        self.progress = 0.7

        # 对same_side_cloud进行聚类
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
            labels = np.array(same_side_cloud.cluster_dbscan(eps=0.05, min_points=50, print_progress=True))

        # 找到最大的簇
        max_label = labels.max()
        largest_cluster_idx = 0
        largest_cluster_size = 0
        for i in range(max_label + 1):
            cluster_size = np.sum(labels == i)
            if cluster_size > largest_cluster_size:
                largest_cluster_size = cluster_size
                largest_cluster_idx = i

        # 提取最大的簇
        largest_cluster_indices = np.where(labels == largest_cluster_idx)[0].tolist()

        self.progress = 0.9
        # 创建最大的簇点云
        largest_cluster_cloud = same_side_cloud.select_by_index(largest_cluster_indices)

        # 设置最终处理后的点云为最大的簇
        self.processed_pcd = largest_cluster_cloud

        self.processed_mesh = self.point_cloud_to_mesh(self.processed_pcd)

        points_array = np.asarray(self.processed_pcd.points)
        min_z_value = np.min(points_array[:, 2])
        self.min_ditance = min_z_value

        # 更新进度为 1.0（100%）
        self.progress = 1.0

    def getModelFromO3d(self, model):
        colors = np.asarray(model.vertex_colors)
        # 获取顶点和三角形数据
        vertices = np.asarray(model.vertices)
        triangles = np.asarray(model.triangles)
        # 创建 vtkPoints 存储顶点数据
        points = vtk.vtkPoints()
        for vertex in vertices:
            points.InsertNextPoint(vertex)

        # 创建 vtkCellArray 存储三角形数据
        cells = vtk.vtkCellArray()
        for triangle in triangles:
            cell = vtk.vtkTriangle()
            cell.GetPointIds().SetId(0, triangle[0])
            cell.GetPointIds().SetId(1, triangle[1])
            cell.GetPointIds().SetId(2, triangle[2])
            cells.InsertNextCell(cell)

        # 创建 vtkUnsignedCharArray 存储颜色数据
        color_array = vtk.vtkUnsignedCharArray()
        color_array.SetNumberOfComponents(3)
        color_array.SetName("Colors")
        color_array.SetNumberOfTuples(len(colors))
        for i, color in enumerate(colors):
            color_array.SetTuple3(i, int(255 * color[0]), int(255 * color[1]), int(255 * color[2]))

        # 创建 vtkPolyData 存储几何数据
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(points)
        poly_data.SetPolys(cells)
        poly_data.GetPointData().SetScalars(color_array)

        return poly_data
    
    def model_show(self,QWidget):
        self.ren = vtk_show.create_renderer()
        QWidget.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = QWidget.vtkWidget.GetRenderWindow().GetInteractor()
        self.VtkModel = self.getModelFromO3d(self.processed_mesh)
        vtk_show.add_actor(self.ren,self.VtkModel)
        self.ren.ResetCamera()
        self.iren.Initialize()
    
    
    def point_cloud_to_mesh(self, pcd, alpha=0.025):
        """
        将点云转换为三角网格
        """
    # 使用 alpha_shape 算法
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
        return mesh

def main(args=None):
    rclpy.init(args=args)
    point_cloud_viewer = PointCloudViewerNode()
    point_cloud_viewer.run()
    point_cloud_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


