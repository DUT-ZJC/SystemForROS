#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/io/pcd_io.h>
#include <cmath> // for std::isfinite

using namespace std::placeholders;

class EdgeDetectionNode : public rclcpp::Node
{
public:
    EdgeDetectionNode() : Node("edge_detection_node")
    {
        // 创建点云订阅者
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points2", 1, std::bind(&EdgeDetectionNode::point_cloud_callback, this, _1));

        // 打印节点启动信息
        RCLCPP_INFO(this->get_logger(), "Edge Detection Node started.");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 将ROS消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // 打印点云信息
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->points.size());

        // 过滤掉无效的点（nan或inf）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& point : cloud->points)
        {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            {
                valid_cloud->points.push_back(point);
            }
        }
        valid_cloud->width = valid_cloud->points.size();
        valid_cloud->height = 1;
        valid_cloud->is_dense = false;

        // 边缘检测处理
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        perform_edge_detection(valid_cloud, edge_cloud);

        // 打印边缘点云信息
        RCLCPP_INFO(this->get_logger(), "Edge cloud size: %zu", edge_cloud->points.size());

        // 保存分割后的彩色点云为PCD文件
        save_edge_cloud(edge_cloud);
    }

    void perform_edge_detection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
    {
        // 使用RegionGrowingRGB算法进行边缘检测
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud(input_cloud);
        reg.setDistanceThreshold(0.02); // 替代 setSearchRadius
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // 设置平滑阈值
        reg.setPointColorThreshold(5.0); // 设置颜色阈值

        std::vector<pcl::PointIndices> indices;
        reg.extract(indices);

        // 提取边缘点
        for (const auto& region : indices)
        {
            for (const auto& idx : region.indices)
            {
                output_cloud->points.push_back((*input_cloud)[idx]);
            }
        }
        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = false;

        // 打印边缘点云信息
        RCLCPP_INFO(this->get_logger(), "Extracted edge cloud with %zu points", output_cloud->points.size());
    }

    void save_edge_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_cloud)
    {
        std::string filename = "edge_cloud.pcd";
        if (pcl::io::savePCDFile(filename, *edge_cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't write file %s \n", filename.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Saved %s \n", filename.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdgeDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}