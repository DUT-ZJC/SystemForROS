#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class MoveToPose
{
public:
    MoveToPose(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        node_->declare_parameter("robot_name", "lbr");
        auto robot_name = node_->get_parameter("robot_name").as_string();
        // 初始化 MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

        // 设置规划参考框架
        move_group_->setPoseReferenceFrame("lbr_link_0");

        // 设置末端执行器链接
        move_group_->setEndEffectorLink("lbr_link_ee");

        // 输出当前状态
        RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface initialized");
    }

    bool moveToPose(const geometry_msgs::msg::Pose& target_pose)
    {
        // 设置目标姿态
        move_group_->setPoseTarget(target_pose);

        // 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(node_->get_logger(), "Planning succeeded");
            move_group_->execute(plan);
            return true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
            return false;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_to_pose_node");

    // 创建 MoveToPose 对象
    MoveToPose move_to_pose(node);

    // 定义目标姿态
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.5;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    // 移动到目标姿态
    if (move_to_pose.moveToPose(target_pose))
    {
        RCLCPP_INFO(node->get_logger(), "Successfully moved to target pose");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to target pose");
    }

    rclcpp::shutdown();
    return 0;
}