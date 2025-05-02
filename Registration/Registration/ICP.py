import numpy as np
from stl import mesh
import pickle
from sklearn.neighbors import KDTree

# 加载 STL 文件并提取点云
def load_stl_point_cloud(file_path):
    mesh_data = mesh.Mesh.from_file(file_path)
    points = np.vstack((mesh_data.v0, mesh_data.v1, mesh_data.v2))
    return np.unique(points, axis=0)

# ICP 点云配准算法
def icp(source_points, target_points, initial_transformation, max_iterations=100, tolerance=1e-6):
    # 初始化变换矩阵
    transformation = initial_transformation
    
    for iteration in range(max_iterations):
        # 应用当前变换矩阵到源点云
        transformed_source_points = np.dot(source_points, transformation[:3, :3].T) + transformation[:3, 3]
        
        # 构建目标点云的 KD-tree
        transformed_source_points_tree = KDTree(transformed_source_points)
        
        # 查询最近邻点
        distances, indices = transformed_source_points_tree.query(target_points, k=1)
        
        # 计算刚体变换
        # 使用配对点计算新的变换矩阵
        # 这里使用了 Kabsch 算法来计算最优旋转和平移
        source_center = np.mean(transformed_source_points[indices.flatten()], axis=0)
        target_center = np.mean(target_points, axis=0)

        source_points_centered = transformed_source_points[indices.flatten()] - source_center
        target_points_centered = target_points - target_center
        
        print(iteration) 
        #print(source_points_centered)
        #print(target_points_centered)
        H = np.dot(source_points_centered.T, target_points_centered)

        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # 确保旋转矩阵是正确的
        if np.linalg.det(R) < 0:
            print("det(R) < 0, reflection detected!, correcting for it...")
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        t = np.mean(target_points - np.dot(transformed_source_points[indices.flatten()], R.T), axis=0)
        # 更新变换矩阵
        new_transformation = np.eye(4)
        new_transformation[:3, :3] = R
        new_transformation[:3, 3] = t
        new_transformation = np.dot(new_transformation, transformation)
        # 检查收敛条件
        print(np.linalg.norm(new_transformation - transformation))
        if np.linalg.norm(new_transformation - transformation) < tolerance:
            print("收敛了！")
            break
        
        transformation = new_transformation
    
    return transformation

# 加载点云
source_points = load_stl_point_cloud('外侧.stl')  # 外侧点云作为源点云
target_points = load_stl_point_cloud('内侧部分.stl')  # 内侧点云作为目标点云

# 加载初始变换矩阵
with open('transform_outside_to_inside.pkl', 'rb') as f:
    initial_transformation = pickle.load(f)

# 设置迭代参数
max_iterations = 1000  # 最大迭代次数
tolerance = 1e-10  # 收敛阈值

# 执行 ICP 算法
final_transformation = icp(source_points, target_points, initial_transformation, max_iterations, tolerance)

# 保存最终变换矩阵
with open('final_transformation.pkl', 'wb') as f:
    pickle.dump(final_transformation, f)

print("迭代后的外侧到内侧的变换矩阵已保存到 final_transformation.pkl")