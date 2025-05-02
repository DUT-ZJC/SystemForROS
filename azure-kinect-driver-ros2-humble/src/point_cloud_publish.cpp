#include <iostream>
#include <memory>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // 包含迭代器头文件

class PointCloudPublisherNode : public rclcpp::Node
{
public:
    PointCloudPublisherNode(const std::string& file_path)
        : Node("point_cloud_publisher_node")
    {
        // 创建publisher，发布PointCloud2消息到"points2"话题
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);

        // 读取PCD文件
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", file_path.c_str());
            return;
        }

        // 创建一个新的PointCloud2消息
        point_cloud_.data.clear();
        point_cloud_.height = cloud->height;
        point_cloud_.width = cloud->width;
        point_cloud_.is_dense = cloud->is_dense;

        // 使用PointCloud2Modifier设置字段
        sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        point_cloud_.header.frame_id = "map"; // 设置frame_id，根据需要修改
        point_cloud_.header.stamp = this->now();

        // 计算每个字段的步长和偏移量
        uint32_t offset = 0;
        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_, "b");

        // 遍历PCL点云并填充ROS2点云消息
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            *iter_x = cloud->points[i].x;
            *iter_y = cloud->points[i].y;
            *iter_z = cloud->points[i].z;
            *iter_r = cloud->points[i].r;
            *iter_g = cloud->points[i].g;
            *iter_b = cloud->points[i].b;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }

        // 定义一个定时器，定期发布点云
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PointCloudPublisherNode::publishPointCloud, this));
    }

private:
    void publishPointCloud()
    {
        if (publisher_->get_subscription_count() > 0)
        {
            point_cloud_.header.stamp = this->now();
            publisher_->publish(point_cloud_);
            RCLCPP_INFO(this->get_logger(), "Published point cloud to 'points2'");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    sensor_msgs::msg::PointCloud2 point_cloud_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("point_cloud_publisher_node"), "No PCD file path provided.");
        return 1;
    }

    std::string file_path = argv[1];
    auto node = std::make_shared<PointCloudPublisherNode>(file_path);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}