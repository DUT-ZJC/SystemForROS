#include <rclcpp/rclcpp.hpp>
#include <execute_movement_msgs/srv/move_to_strat.hpp>
#include <execute_movement_msgs/srv/keep_move.hpp>
#include <execute_movement_msgs/srv/stop_move.hpp>
#include <execute_movement_msgs/srv/auto_move_start.hpp>
#include <execute_movement_msgs/srv/stop_and_back.hpp>
#include "RobotAct.hpp"
#include <cmath>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class RobotServer : public rclcpp::Node
{
public:

    rclcpp::CallbackGroup::SharedPtr default_group_;
    rclcpp::CallbackGroup::SharedPtr get_local_mover_timer_group() const {return local_mover_->get_timer_group();}

    std::shared_ptr<execute_movement::RobotAct> local_mover_;
    explicit RobotServer(const rclcpp::NodeOptions & options)
    : Node("Surgery_Robot_Server", options)
    {
        
        rclcpp::on_shutdown([this]() {this->set_running(false);});
        default_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // Setup services
        move_to_start_service_ = this->create_service<execute_movement_msgs::srv::MoveToStrat>(
            "move_to_start", std::bind(&RobotServer::move_to_start, this, std::placeholders::_1, std::placeholders::_2));
        keep_move_service_ = this->create_service<execute_movement_msgs::srv::KeepMove>(
            "keep_move", std::bind(&RobotServer::keep_move, this, std::placeholders::_1, std::placeholders::_2));
        stop_move_service_ = this->create_service<execute_movement_msgs::srv::StopMove>(
            "stop_move", std::bind(&RobotServer::stop_move, this, std::placeholders::_1, std::placeholders::_2));
        auto_move_start_service_ = this->create_service<execute_movement_msgs::srv::AutoMoveStart>(
            "auto_move", std::bind(&RobotServer::auto_move_start, this, std::placeholders::_1, std::placeholders::_2));
        stop_and_back_service_ = this->create_service<execute_movement_msgs::srv::StopAndBack>(
            "stop_and_back", std::bind(&RobotServer::stop_and_back, this, std::placeholders::_1, std::placeholders::_2));
            
    }
    
    void InitServer()
    {
        local_mover_ = std::make_shared<execute_movement::RobotAct>();

        local_mover_->ActInit(this->shared_from_this());
    }
    void set_running(bool value) {
        local_mover_->running = value;
    }


private:

    rclcpp::Service<execute_movement_msgs::srv::MoveToStrat>::SharedPtr move_to_start_service_;
    rclcpp::Service<execute_movement_msgs::srv::KeepMove>::SharedPtr keep_move_service_;
    rclcpp::Service<execute_movement_msgs::srv::StopMove>::SharedPtr stop_move_service_;
    rclcpp::Service<execute_movement_msgs::srv::AutoMoveStart>::SharedPtr auto_move_start_service_;
    rclcpp::Service<execute_movement_msgs::srv::StopAndBack>::SharedPtr stop_and_back_service_;

    void move_to_start(
        const std::shared_ptr<execute_movement_msgs::srv::MoveToStrat::Request> request,
        std::shared_ptr<execute_movement_msgs::srv::MoveToStrat::Response> response)
    {
        bool state = local_mover_->set_and_move_to_starting_position();
        response->success = state;
    }

    void keep_move(
        const std::shared_ptr<execute_movement_msgs::srv::KeepMove::Request> request,
        std::shared_ptr<execute_movement_msgs::srv::KeepMove::Response> response)
    {
        bool state = local_mover_->back_to_beginning_position();
        response->success = state;
    }

    void stop_move(
        const std::shared_ptr<execute_movement_msgs::srv::StopMove::Request> request,
        std::shared_ptr<execute_movement_msgs::srv::StopMove::Response> response)
    {
        bool state = local_mover_->stop();
        response->success = state;
    }

    void auto_move_start(
        const std::shared_ptr<execute_movement_msgs::srv::AutoMoveStart::Request> request,
        std::shared_ptr<execute_movement_msgs::srv::AutoMoveStart::Response> response)
    {
        bool state = local_mover_->auto_move(request->target_pose_index);
        response->success = state;
    }

    void stop_and_back(
        const std::shared_ptr<execute_movement_msgs::srv::StopAndBack::Request> request,
        std::shared_ptr<execute_movement_msgs::srv::StopAndBack::Response> response)
    {
        bool state = local_mover_->stop_and_back();
        response->success = state;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_opts;
    node_opts.arguments({
        "--ros-args",
        "-r", "joint_states:=/lbr/joint_states"
    });

    // 2) 初始化 ROS 2

    // 3) 用带 options 的构造函数创建节点
    auto node = std::make_shared<RobotServer>(node_opts);
    node->declare_parameter("robot_name", "lbr");

    // 使用单线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });
    spin_thread.detach();
    node->InitServer();



    // 清理
    rclcpp::shutdown();
    return 0;

}