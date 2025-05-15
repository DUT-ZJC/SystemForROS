#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Configure node
    auto node_ptr = rclcpp::Node::make_shared("hello_moveit");
    node_ptr->declare_parameter("robot_name", "lbr");
    auto robot_name = node_ptr->get_parameter("robot_name").as_string();

    // Create MoveGroupInterface (lives inside robot_name namespace)
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
        node_ptr, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description", robot_name));

    // Set the reference frame
    move_group_interface.setPoseReferenceFrame("world");

    // Set position and orientation tolerances
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.001);

    // Set maximum velocity and acceleration scaling factors
    move_group_interface.setMaxAccelerationScalingFactor(0.5);
    move_group_interface.setMaxVelocityScalingFactor(0.5);

    // Move to Home position
    RCLCPP_INFO(node_ptr->get_logger(), "Moving to pose: Zero");
    move_group_interface.setNamedTarget("zero");
    move_group_interface.move();

    // Set the initial target pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.6;
    target_pose.pose.position.z = 1.4;

    // Set the orientation to rotate 90 degrees around the X axis
    double angle = M_PI / 2.0;  // 90 degrees in radians
    target_pose.pose.orientation.w = cos(angle / 2.0);
    target_pose.pose.orientation.x = sin(angle / 2.0);
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;

    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(target_pose.pose);
    move_group_interface.move();

    // Define waypoints for spiral movement in x-y plane with z-axis descent
    std::vector<geometry_msgs::msg::Pose> waypoints;
    double centerA = target_pose.pose.position.x;
    double centerB = target_pose.pose.position.y;
    double radius = 0.1;
    double z_step = -0.01;  // Step size for z-axis descent
    double z_initial = target_pose.pose.position.z;

    for (double th = 0; th <= (3.141526 * 4); th += 0.015) {
        target_pose.pose.position.x = centerA + radius * cos(th);
        target_pose.pose.position.y = centerB + radius * sin(th);
        target_pose.pose.position.z = z_initial + z_step * th;
        waypoints.push_back(target_pose.pose);
    }

    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(node_ptr->get_logger(), "Cartesian path %.2f%% achieved", fraction * 100.0);

    // Execute the trajectory if the path was computed successfully
    if (fraction == 1.0) {
        move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(node_ptr->get_logger(), "Failed to compute Cartesian path.");
    }

    // Move back to Home position
    RCLCPP_INFO(node_ptr->get_logger(), "Moving to pose: Zero");
    move_group_interface.setNamedTarget("zero");
    move_group_interface.move();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}