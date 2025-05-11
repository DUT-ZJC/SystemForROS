#ifndef MY_ROBOT_PACKAGE_ROBOTACT_HPP
#define MY_ROBOT_PACKAGE_ROBOTACT_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int64.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <mutex>
#include <iostream>
#include <filesystem>
#include <cstdlib> 

namespace execute_movement
{
    class RobotAct
    {
    public:
        RobotAct();
        
        void ActInit(rclcpp::Node::SharedPtr node);

        bool set_and_move_to_starting_position();
        bool keep_moving();
        bool stop();
        bool auto_move(int target_index);
        bool stop_and_back();

        void move_to_target_point(const std::vector<double>& target_point);
        void publish_index();
        void path_load(const std::string& load_path);

        geometry_msgs::msg::Quaternion quaternion_to_pose_orientation(const std::vector<double>& quaternion);

    private:
        std::filesystem::path save_path_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::Node::SharedPtr node_;
        std::vector<std::vector<double>> center_point_;
        std::vector<std::vector<double>> actpoint_;
        std::vector<std::vector<double>> rotation_;
        std::vector<double> deep_position_;
        std::vector<double> thickness_;
        std::vector<double> normal_;
        std::vector<double> quaternion_;
        geometry_msgs::msg::Quaternion fixed_orientation_;
        size_t total_point_num_;
        std::vector<double> start_point_;
        size_t current_index_;
        size_t target_index_; // 添加此成员变量
        bool stop_flag_;
        std::mutex mutex_;
    };
}

#endif // MY_ROBOT_PACKAGE_ROBOTACT_HPP