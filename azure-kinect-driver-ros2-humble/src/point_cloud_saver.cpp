#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <sstream>

class PointCloudSaver : public rclcpp::Node
{
public:
    PointCloudSaver() : Node("point_cloud_saver"), frame_count_(0), max_frames_(20)
    {
        // 创建订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points2", 10, std::bind(&PointCloudSaver::point_cloud_callback, this, std::placeholders::_1));
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (frame_count_ >= max_frames_)
        {
            RCLCPP_INFO(this->get_logger(), "Reached maximum frame count. Stopping subscription.");
            return; // 达到最大帧数后，停止订阅
        }

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        try
        {
            // 转换 ROS 消息为 PCL 点云
            pcl::fromROSMsg(*msg, cloud);

            std::ostringstream ss;
            ss << "point_cloud_" << frame_count_ << ".pcd"; // 生成文件名
            pcl::io::savePCDFile(ss.str(), cloud); // 保存 PCD 文件

            RCLCPP_INFO(this->get_logger(), "Saved point cloud frame %d to %s", frame_count_, ss.str().c_str());
            frame_count_++;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to process point cloud: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int frame_count_;
    const int max_frames_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
