#ifndef MY_ROBOT_PACKAGE_ROBOTACT_HPP
#define MY_ROBOT_PACKAGE_ROBOTACT_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <mutex>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <nanoflann.hpp>
#include <cmath>
#include <limits>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
namespace execute_movement
{
    struct PointCloud {
        std::vector<std::vector<double>> pts;
        inline size_t kdtree_get_point_count() const { return pts.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    class RobotAct
    {
    public:
        std::atomic<bool> running;
        RobotAct();
        void ActInit(rclcpp::Node::SharedPtr node);
        rclcpp::CallbackGroup::SharedPtr get_timer_group() const { return timer_group_; }

        rclcpp::CallbackGroup::SharedPtr timer_group_;
        bool set_and_move_to_starting_position();
        bool back_to_beginning_position();
        bool stop();
        bool auto_move(int target_index);
        bool stop_and_back();
        void move_to_target_point(int now_index,int target_index);
        void publish_current_pose();
        int find_nearest_point(const std::vector<double>& query_point);
        void publish_index(size_t current_index_);
        void path_load(const std::string& load_path);
        geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(const std::vector<std::vector<double>>& rotation_matrix);

    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        moveit_msgs::msg::RobotTrajectory trajectory;
        trajectory_processing::TimeOptimalTrajectoryGeneration toptg;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        std::shared_ptr<robot_trajectory::RobotTrajectory> rt_;

        geometry_msgs::msg::TransformStamped tf_w_v;
        const double velocity_ = 0.005;
        const double acceleration_ = 0.05;
        std::filesystem::path save_path_;
        std::vector<double> target_point;
        geometry_msgs::msg::PoseStamped begin_pose;
        geometry_msgs::msg::PoseStamped now_pose;
        geometry_msgs::msg::PoseStamped now_virtual;
        moveit::core::RobotStatePtr now_state;
        void set_speed_and_move(double velocity = 0.005,double acceleration = 0.1);
        std::vector<geometry_msgs::msg::Pose> waypoints;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr Index_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr LogPublish;
        rclcpp::Node::SharedPtr node_;
        std::vector<std::vector<double>> center_point_, actpoint_, rotation_;
        std::vector<double> deep_position_, thickness_, normal_, quaternion_, start_point_;
        geometry_msgs::msg::Quaternion fixed_orientation_;
        size_t total_point_num_, current_index_, target_index_,last_index_,last_act_index_;
        bool stop_flag_, reset_flag_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::mutex mutex_;
        PointCloud point_cloud_;
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3>;
        std::unique_ptr<KDTree> kdtree_;
    };
}

#endif // MY_ROBOT_PACKAGE_ROBOTACT_HPP