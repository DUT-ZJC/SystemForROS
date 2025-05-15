#include "RobotAct.hpp"
namespace execute_movement
{
    RobotAct::RobotAct()
        : current_index_(0), stop_flag_(true), target_index_(0),reset_flag_(false),last_index_(9999999),running(true),last_act_index_(0)
    {   

        point_cloud_.pts = {}; // 初始化为空点云
        kdtree_ = std::make_unique<KDTree>(
            3, 
            point_cloud_, 
            nanoflann::KDTreeSingleIndexAdaptorParams(10)
        );
        const char* home = std::getenv("HOME");
        // 拼接路径
        save_path_ = std::filesystem::path(home) / ".ros2" / "surgery" / "path.yaml";
        // Load path data
        path_load(save_path_);
    }
    
    void RobotAct::ActInit(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        auto robot_name = node->get_parameter("robot_name").as_string();
        
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, moveit::planning_interface::MoveGroupInterface::Options("arm", "robot_description",
                                                                        robot_name));

        //joint_state_broadcaster
    // 2. 通过参数服务器设置关节话题（推荐方式）
        // Initialize MoveGroupCommander
    
        move_group_->setPoseReferenceFrame("virtual_frame");
        // 设置末端执行器链接
        move_group_->setEndEffectorLink("lbr_link_ee");
        move_group_->setGoalPositionTolerance(0.001);
        move_group_->setGoalOrientationTolerance(0.001);
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.1);  // 速度缩放因子，0.1 表示最大速度为默认最大速度的 10%
        move_group_->startStateMonitor(10);
        LogPublish = node_->create_publisher<std_msgs::msg::String>("SystemLog", 10);
        Index_publisher_ = node_->create_publisher<std_msgs::msg::Int64>("point_index", 10);
        pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

        timer_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        begin_pose = move_group_->getCurrentPose();
        rclcpp::Rate rate(10); // 设置循环频率为 10 Hz
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::Duration(1), node_);
        tf2_ros::TransformListener tf_listener(*tf_buffer);
        rt_ = std::make_shared<robot_trajectory::RobotTrajectory>(move_group_->getRobotModel(), move_group_->getName());


        tf_w_v = tf_buffer->lookupTransform(
            "virtual_frame", "world",
            tf2::TimePointZero,
            tf2::Duration(std::chrono::seconds(1)));

        //timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&RobotAct::publish_current_pose, this),timer_group_);
        while (running)
        {
            now_state = move_group_->getCurrentState();
            now_pose = move_group_->getCurrentPose();
            tf2::doTransform(now_pose, now_virtual, tf_w_v);
            if (now_pose.pose.position.x!=0&&now_pose.pose.position.y!=0&&now_pose.pose.position.z!=0&&stop_flag_ == false)
            {

                current_index_ = find_nearest_point({now_virtual.pose.position.x*1000, now_virtual.pose.position.y*1000, now_virtual.pose.position.z*1000});
                if (current_index_!=target_index_&&current_index_!=last_index_)
                {
                   pose_publisher_->publish(now_virtual);
                   publish_index(current_index_); 
                   last_index_ = current_index_;
                }
                RCLCPP_INFO(node_->get_logger(), "Current Index: (%ld)",current_index_);


            }
            rate.sleep();
        }
        
    }

    bool RobotAct::set_and_move_to_starting_position()
    {   
        move_group_->setStartStateToCurrentState();
        // Convert Start_point to Pose messag
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = start_point_[0]/1000;
        start_pose.position.y = start_point_[1]/1000;
        start_pose.position.z = start_point_[2]/1000;
        start_pose.orientation = fixed_orientation_;
        waypoints.push_back(start_pose);
        set_speed_and_move(0.1,0.1);
        return true;
    }

    bool RobotAct::back_to_beginning_position()
    {   
        geometry_msgs::msg::PoseStamped begin_pose_virtual;
        tf2::doTransform(begin_pose, begin_pose_virtual, tf_w_v);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position = begin_pose_virtual.pose.position;;
        target_pose.orientation = begin_pose_virtual.pose.orientation;
        waypoints.push_back(target_pose);
        set_speed_and_move(0.1,0.1);
        return true;
    }

    bool RobotAct::stop()
    {   
        
        move_group_-> stop();
        stop_flag_ = true;
        last_act_index_ = current_index_;
        return true;
    }

    bool RobotAct::auto_move(int target_index)
    {
        if (current_index_ == 0)
        {
            target_index_ = target_index;
            geometry_msgs::msg::Pose target_pose;
            for (int i = 1; i < 11; ++i)
            {
                target_pose.position.x = (start_point_[0] - i * normal_[0] )/1000;
                target_pose.position.y = (start_point_[1] - i * normal_[1] )/1000;
                target_pose.position.z = (start_point_[2] - i * normal_[2] )/1000;
                target_pose.orientation = fixed_orientation_;
                waypoints.push_back(target_pose);
            }
            move_to_target_point(0,target_index_);
            return true;
        }
        else
        {
            if (reset_flag_ == true)
            {
                reset_flag_ = false;

                std::vector<double> last_point_ = actpoint_[last_act_index_];
                for (int i = 10000; i > 0; --i)
                {
                    geometry_msgs::msg::Pose target_pose;
                    target_pose.position.x = (last_point_[0] + i*normal_[0] * 100/10000)/1000;
                    target_pose.position.y = (last_point_[1] + i*normal_[1] * 100/10000)/1000;
                    target_pose.position.z = (last_point_[2] + i*normal_[2] * 100/10000)/1000;
                    target_pose.orientation = fixed_orientation_;
                    waypoints.push_back(target_pose);
                }
                move_to_target_point(last_act_index_,target_index);
                return true;

            }else{
                move_to_target_point(last_act_index_,target_index);
                return true;
            }
     
        }
    }

    bool RobotAct::stop_and_back()
    {
        stop();
        reset_flag_ = true;
        for (int i = 0; i < 10000; ++i)
        {
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = (now_virtual.pose.position.x + i*normal_[0] * 100/1000/10000);
            target_pose.position.y = (now_virtual.pose.position.y + i*normal_[1] * 100/1000/10000);
            target_pose.position.z = (now_virtual.pose.position.z + i*normal_[2] * 100/1000/10000);
            target_pose.orientation = fixed_orientation_;
            waypoints.push_back(target_pose);
        }
        set_speed_and_move();
        return true;
    }

    void RobotAct::move_to_target_point(int now_index,int target_index)
    {   
        stop_flag_ = false;
        if (now_index<target_index)
        {
            for (int index =now_index+1; index < target_index+1;index++)
            {   
                target_point = actpoint_[index];
                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = target_point[0]/1000;
                target_pose.position.y = target_point[1]/1000;
                target_pose.position.z = target_point[2]/1000;
                target_pose.orientation = fixed_orientation_;
                waypoints.push_back(target_pose);  // 目标位姿
            }
            set_speed_and_move();
        }
        else{
            for (int index =now_index; index > target_index;index--)
            {
                target_point = actpoint_[index];
                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = target_point[0]/1000;
                target_pose.position.y = target_point[1]/1000;
                target_pose.position.z = target_point[2]/1000;
                target_pose.orientation = fixed_orientation_;
                waypoints.push_back(target_pose);  // 目标位姿
            }
            set_speed_and_move();
        }
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
        normal_ = {rotation_[0][2], rotation_[1][2], rotation_[2][2]};
        fixed_orientation_ = rotationMatrixToQuaternion(rotation_);

        // Calculate total point number
        total_point_num_ = actpoint_.size();

        // 更新点云数据
        point_cloud_.pts = actpoint_;
        
        // 重建KDTree索引
        kdtree_->buildIndex(); // 注意：不需要参数！


        // Calculate start point
        start_point_ = {actpoint_[0][0] + 10 * normal_[0], actpoint_[0][1] + 10 * normal_[1], actpoint_[0][2] + 10 * normal_[2]};
    }

    geometry_msgs::msg::Quaternion RobotAct::rotationMatrixToQuaternion(const std::vector<std::vector<double>>& rotation_matrix)
    {
        // 将旋转矩阵转换为tf2::Matrix3x3
        tf2::Matrix3x3 mat(-rotation_matrix[0][0], -rotation_matrix[0][1], rotation_matrix[0][2],
                        -rotation_matrix[1][0], -rotation_matrix[1][1], rotation_matrix[1][2],
                        -rotation_matrix[2][0], -rotation_matrix[2][1], rotation_matrix[2][2]);

        // 从tf2::Matrix3x3获取四元数
        tf2::Quaternion quat;
        mat.getRotation(quat);

        // 将tf2::Quaternion转换为geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = quat.x();
        quaternion.y = quat.y();
        quaternion.z = quat.z();
        quaternion.w = quat.w();

        return quaternion;
    }

    int RobotAct::find_nearest_point(const std::vector<double>& query_point) {  // 参数名必须与声明一致

        size_t ret_index = 0;
        double out_dist_sqr = 0.0;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        
        const double* query_ptr = query_point.data();  // 明确获取指针
        kdtree_->findNeighbors(
            resultSet,
            query_ptr,  // 使用明确的指针变量
            nanoflann::SearchParams(10)
        );
        
        return static_cast<int>(ret_index);
    }

    /*void RobotAct::publish_current_pose()
    {
            geometry_msgs::msg::PoseStamped current_pose = now_pose;
            
            if (current_pose.pose.position.x!=0&&current_pose.pose.position.y!=0&&current_pose.pose.position.z!=0&&stop_flag_ == false)
            {
                int64_t current_index_ = find_nearest_point({current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z});
                if (current_index_!=target_index_)
                {
                   pose_publisher_->publish(current_pose);
                   publish_index(current_index_); 
                }
            } 
        
                

            // 日志输出
            RCLCPP_INFO(node_->get_logger(), "Current Pose: (%f, %f, %f)", 
                        current_pose.pose.position.x, 
                        current_pose.pose.position.y, 
                        current_pose.pose.position.z);
       
    }*/

    void RobotAct::publish_index( size_t current_index_)
    {
        std_msgs::msg::Int64 msg;
        msg.data = (int64_t)current_index_;
        Index_publisher_->publish(msg);
    }

    void RobotAct::set_speed_and_move(double velocity ,double acceleration )
    {
        double fraction = move_group_->computeCartesianPath(waypoints, 0.001,0.0, trajectory);

        rt_->setRobotTrajectoryMsg(*now_state, trajectory);
        toptg.computeTimeStamps(*rt_, velocity, acceleration);
        rt_->getRobotTrajectoryMsg(trajectory);
	    plan.trajectory_ = trajectory;
        //adjustTrajectorySpeed(waypoints, trajectory, 5.0);
                        
        if (fraction >= 0.99) // 使用新的 MoveItErrorCode
        {   
            // 异步执行计划
            move_group_->asyncExecute(plan);
        }
        waypoints.clear();
        
    }
    


}