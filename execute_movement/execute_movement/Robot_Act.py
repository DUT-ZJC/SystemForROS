from __future__ import print_function
from __future__ import division
from transforms3d.quaternions import qmult
from transforms3d.euler import euler2quat
from geometry_msgs.msg import Quaternion
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import threading
import rclpy
import numpy as np
from pathlib import Path
import math
from tf2_geometry_msgs import from_matrix
from std_msgs.msg import Int64
save_path = Path.home() / ''

class RobotAct:
    def __init__(self, node: rclpy.node.Node,  max_velocity_scaling: float,
                 max_acceleration_scaling: float, angle_delta: float, translation_delta: float):
        self.node = node
        self.publish_ = self.node.create_publisher(Int64,"point_index",10)
        self.msg = Int64()

        self.path_load(save_path)
        self.Total_Point_Num = len(self.actpoint)
        self.Start_point = tuple((np.array(self.actpoint[0]) + 10*self.normal))
        self.Stop_Flag = True
        self.Target_index = 0
        self.Current_index = 0

        self.mgc = MoveGroupCommander()
        self.mgc.set_planner_id("RRTConnectkConfigDefault")  # TODO: this is only needed for the UR5
        self.mgc.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.mgc.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.joint_limits = [math.radians(90)] * 5 + [math.radians(180)] + [math.radians(350)]  # TODO: make param
        self.mgc.set_pose_reference_frame("my_frame")
        self.fallback_joint_limits = [math.radians(90)] * 4 + [math.radians(90)] + [math.radians(180)] + [
            math.radians(350)]
        if len(self.mgc.get_active_joints()) == 6:
            self.fallback_joint_limits = self.fallback_joint_limits[1:]
        self.angle_delta = angle_delta
        self.translation_delta = translation_delta

        self.lock = threading.Lock()
    def set_and_moveto_starting_position(self):
        # sets the starting position to the current robot cartesian EE position and checks movement in all directions
        # TODO: make repeatable
        #  - save the home position and each target position as joint position
        #  - plan to each target position and back, save plan if not crazy
        #  - return true if can plan to each target position without going crazy
        # Convert self.Start_point to a Pose message
        start_pose = Pose()
        start_pose.position.x = self.Start_point[0]
        start_pose.position.y = self.Start_point[1]
        start_pose.position.z = self.Start_point[2]
        start_pose.orientation.x = self.quaternion.x
        start_pose.orientation.y = self.quaternion.y
        start_pose.orientation.z = self.quaternion.z
        start_pose.orientation.w = self.quaternion.w
        # Assuming the orientation is the same as the current pose
        self.fixed_orientation = start_pose.orientation

        # Set the target pose
        self.mgc.set_pose_target(start_pose)

        # Plan and execute the movement
        plan = self.mgc.plan()
        if plan.joint_trajectory.points:
            self.mgc.execute(plan, wait=True)
            return True
        else:
            return False
    

    def keep_moving(self):
        with self.lock:
            self.Stop_Flag = False

        while(self.Current_index < (self.Total_Point_Num-1)):
            with self.lock:
                if self.Stop_Flag:
                    return False
            if self.Current_index == 0:
                target_pose = Pose()
                target_point = self.actpoint[0]
                for i in range(100):
                    target_pose.position.x = target_point[0] -i*self.normal[0]/10
                    target_pose.position.y = target_point[1] -i*self.normal[1]/10
                    target_pose.position.z = target_point[2] -i*self.normal[2]/10
                    # Assuming the orientation is the same as the current pose
                    target_pose.orientation = self.fixed_orientation
                    self.mgc.set_pose_target(target_pose)
                    plan = self.mgc.plan()
                    if plan.joint_trajectory.points:
                        self.mgc.execute(plan, wait=True)
                self.Current_index = 1
                self.Publish_Index()
            else:
                target_point = self.actpoint[self.Current_index+1]
                self.Move_to_TargetPoint(target_point)
                self.Publish_Index()
        return True
    
    def Stop(self):
        with self.lock:
            self.Stop_Flag = True
    
    def Automove(self,Target_Index):
        with self.lock:
            self.Stop_Flag = False
        self.Target_index = Target_Index
        while(self.Current_index < self.Target_index):
            with self.lock:
                if self.Stop_Flag:
                    return False
            if self.Current_index == 0:
                target_pose = Pose()
                target_point = self.actpoint[0]
                for i in range(100):
                    target_pose.position.x = target_point[0] -i*self.normal[0]/10
                    target_pose.position.y = target_point[1] -i*self.normal[1]/10
                    target_pose.position.z = target_point[2] -i*self.normal[2]/10
                    # Assuming the orientation is the same as the current pose
                    target_pose.orientation = self.fixed_orientation
                    self.mgc.set_pose_target(target_pose)
                    plan = self.mgc.plan()
                    if plan.joint_trajectory.points:
                        self.mgc.execute(plan, wait=True)
                self.Current_index = 1
                self.Publish_Index()
            else:
                target_point = self.actpoint[self.Current_index+1]
                self.Move_to_TargetPoint(target_point)
                self.Publish_Index()
        return True
    

    def Stop_and_Back(self):
        with self.lock:
            self.Stop_Flag = True

        self.current_point = self.actpoint[self.Current_index]
        self.curren_heigh = self.deep_position[self.Current_index]
        for i in range(100):
            target_pose = Pose()
            target_pose.position.x = self.current_point[0]  +i*self.normal[0]*(self.curren_heigh+10)
            target_pose.position.y = self.current_point[1]  +i*self.normal[0]*(self.curren_heigh+10)
            target_pose.position.z = self.current_point[2]  +i*self.normal[0]*(self.curren_heigh+10)
            # Assuming the orientation is the same as the current pose
            target_pose.orientation = self.fixed_orientation
            self.mgc.set_pose_target(target_pose)
            plan = self.mgc.plan()
            if plan.joint_trajectory.points:
                self.mgc.execute(plan, wait=True)
        return True



    def Move_to_TargetPoint(self,target_point):
        target_pose = Pose()
        target_pose.position.x = target_point[0]
        target_pose.position.y = target_point[1]
        target_pose.position.z = target_point[2]
        # Assuming the orientation is the same as the current pose
        target_pose.orientation = self.fixed_orientation
        self.mgc.set_pose_target(target_pose)
        plan = self.mgc.plan()
        if plan.joint_trajectory.points:
            self.mgc.execute(plan, wait=True)
            self.Current_index +=1

    def Publish_Index(self):
        self.msg.data = self.Current_index
        self.publish_.publish(self.msg)

    def path_load(self, load_path):
            """
            从指定路径加载路径信息
            path_data = {
                'center_point': center_point,
                'actpoint': actpoint_array,
                'rotation': self.path_cal.rotation,
                'deep_position': deep_position,
                'thickness': thickness
            }
            :param load_path: 加载路径的文件名（带.npy后缀）
            """

            # 加载 .npy 文件
            path_data = np.load(load_path, allow_pickle=True).item()
            
            # 提取 actpoint 和 rotation 数据
            self.actpoint = path_data['actpoint'].tolist()  # 转换回列表
            self.rotation = path_data['rotation']
            self.center_point = path_data['center_point'].tolist()
            self.deep_position = path_data['deep_position'].tolist()
            self.thickness = path_data['thickness'].tolist()
            self.normal  = self.rotation[:,2].T
            self.quaternion = from_matrix(self.rotation)
