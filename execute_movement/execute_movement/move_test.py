import rclpy
from rclpy.node import Node
from moveit.planning_interface import MoveGroupInterface
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory

class MoveToPose(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")
        self.get_logger().info("MoveGroupInterface initialized")

        # 获取机器人名称参数
        self.declare_parameter("robot_name", "lbr")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        # 初始化 MoveGroupInterface
        self.move_group = MoveGroupInterface(self, "arm", robot_name)

        # 设置规划参考框架
        self.move_group.set_pose_reference_frame("lbr_link_0")

        # 设置末端执行器链接
        self.move_group.set_end_effector_link("lbr_link_ee")

    def move_to_pose(self, target_pose):
        # 设置目标姿态
        self.move_group.set_pose_target(target_pose)

        # 规划并执行
        plan = self.move_group.plan()
        if plan:
            self.get_logger().info("Planning succeeded")
            self.move_group.execute(plan)
            return True
        else:
            self.get_logger().error("Planning failed")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPose()

    # 定义目标姿态
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    # 移动到目标姿态
    if node.move_to_pose(target_pose):
        node.get_logger().info("Successfully moved to target pose")
    else:
        node.get_logger().error("Failed to move to target pose")

    rclpy.shutdown()

if __name__ == "__main__":
    main()