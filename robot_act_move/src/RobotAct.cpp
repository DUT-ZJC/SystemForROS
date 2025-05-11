#include "RobotAct.hpp"
namespace execute_movement
{
    RobotAct::RobotAct()
        : current_index_(0), stop_flag_(false), target_index_(0)
    {   
        const char* home = std::getenv("HOME");
        // 拼接路径
        save_path_ = std::filesystem::path(home) / ".ros2" / "surgery" / "path.yaml";


        // Load path data
        path_load(save_path_);

    }
    
    void RobotAct::ActInit(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        auto robot_name = node_->get_parameter("robot_name").as_string();
        // Initialize MoveGroupCommander
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPoseReferenceFrame("lbr_link_0");

        // 设置末端执行器链接
        move_group_->setEndEffectorLink("lbr_link_ee");
        publisher_ = node_->create_publisher<std_msgs::msg::Int64>("point_index", 10);
    }
    bool RobotAct::set_and_move_to_starting_position()
    {
        // Convert Start_point to Pose message
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = start_point_[0];
        start_pose.position.y = start_point_[1];
        start_pose.position.z = start_point_[2];
        start_pose.orientation.x = quaternion_[0];
        start_pose.orientation.y = quaternion_[1];
        start_pose.orientation.z = quaternion_[2];
        start_pose.orientation.w = quaternion_[3];

        // Set the target pose
        move_group_->setPoseTarget(start_pose);

        // Plan and execute the movement
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 使用新的 MoveItErrorCode
        if (success)
        {
            move_group_->execute(plan);
        }
        return success;
    }

    bool RobotAct::keep_moving()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = false;

        while (current_index_ < total_point_num_ - 1)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_flag_)
            {
                return false;
            }

            if (current_index_ == 0)
            {
                geometry_msgs::msg::Pose target_pose;
                auto target_point = actpoint_[0];
                for (int i = 0; i < 100; ++i)
                {
                    target_pose.position.x = target_point[0] - i * normal_[0] / 10;
                    target_pose.position.y = target_point[1] - i * normal_[1] / 10;
                    target_pose.position.z = target_point[2] - i * normal_[2] / 10;
                    target_pose.orientation = fixed_orientation_;
                    move_group_->setPoseTarget(target_pose);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) // 使用新的 MoveItErrorCode
                    {
                        move_group_->execute(plan);
                    }
                }
                current_index_ = 1;
                publish_index();
            }
            else
            {
                auto target_point = actpoint_[current_index_ + 1];
                move_to_target_point(target_point);
                publish_index();
            }
        }
        return true;
    }

    bool RobotAct::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = true;
        return true;
    }

    bool RobotAct::auto_move(int target_index)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = false;
        target_index_ = target_index;

        while (current_index_ < target_index_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_flag_)
            {
                return false;
            }

            if (current_index_ == 0)
            {
                geometry_msgs::msg::Pose target_pose;
                auto target_point = actpoint_[0];
                for (int i = 0; i < 100; ++i)
                {
                    target_pose.position.x = target_point[0] - i * normal_[0] / 10;
                    target_pose.position.y = target_point[1] - i * normal_[1] / 10;
                    target_pose.position.z = target_point[2] - i * normal_[2] / 10;
                    target_pose.orientation = fixed_orientation_;
                    move_group_->setPoseTarget(target_pose);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) // 使用新的 MoveItErrorCode
                    {
                        move_group_->execute(plan);
                    }
                }
                current_index_ = 1;
                publish_index();
            }
            else
            {
                auto target_point = actpoint_[current_index_ + 1];
                move_to_target_point(target_point);
                publish_index();
            }
        }
        return true;
    }

    bool RobotAct::stop_and_back()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = true;

        auto current_point = actpoint_[current_index_];
        auto current_height = deep_position_[current_index_];
        for (int i = 0; i < 100; ++i)
        {
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = current_point[0] + i * normal_[0] * (current_height + 10);
            target_pose.position.y = current_point[1] + i * normal_[1] * (current_height + 10);
            target_pose.position.z = current_point[2] + i * normal_[2] * (current_height + 10);
            target_pose.orientation = fixed_orientation_;
            move_group_->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) // 使用新的 MoveItErrorCode
            {
                move_group_->execute(plan);
            }
        }
        return true;
    }

    void RobotAct::move_to_target_point(const std::vector<double>& target_point)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = target_point[0];
        target_pose.position.y = target_point[1];
        target_pose.position.z = target_point[2];
        target_pose.orientation = fixed_orientation_;
        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) // 使用新的 MoveItErrorCode
        {
            move_group_->execute(plan);
            current_index_++;
        }
    }

    void RobotAct::publish_index()
    {
        std_msgs::msg::Int64 msg;
        msg.data = current_index_;
        publisher_->publish(msg);
    }

    void RobotAct::path_load(const std::string& load_path)
    {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(load_path);

        // Load data from YAML file
        center_point_.clear();
        actpoint_.clear();
        rotation_.clear();
        deep_position_.clear();
        thickness_.clear();

        for (const auto& node : config["center_point"]) {
            center_point_.push_back(node.as<std::vector<double>>());
        }
        for (const auto& node : config["actpoint"]) {
            actpoint_.push_back(node.as<std::vector<double>>());
        }
        for (const auto& node : config["rotation"]) {
            rotation_.push_back(node.as<std::vector<double>>());
        }
        deep_position_ = config["deep_position"].as<std::vector<double>>();
        thickness_ = config["thickness"].as<std::vector<double>>();

        // Calculate normal and quaternion
        normal_ = rotation_[0];
        quaternion_ = rotation_[0];
        fixed_orientation_ = quaternion_to_pose_orientation(quaternion_);

        // Calculate total point number
        total_point_num_ = actpoint_.size();

        // Calculate start point
        start_point_ = {actpoint_[0][0] + 10 * normal_[0], actpoint_[0][1] + 10 * normal_[1], actpoint_[0][2] + 10 * normal_[2]};
    }

    geometry_msgs::msg::Quaternion RobotAct::quaternion_to_pose_orientation(const std::vector<double>& quaternion)
    {
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = quaternion[0];
        orientation.y = quaternion[1];
        orientation.z = quaternion[2];
        orientation.w = quaternion[3];
        return orientation;
    }
}