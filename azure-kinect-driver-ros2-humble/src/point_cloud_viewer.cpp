#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

class PointCloudViewer : public rclcpp::Node
{
public:
    PointCloudViewer() : Node("point_cloud_viewer")
    {
        // 初始化PCL可视化器
        viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);
        viewer_->initCameraParameters();
    }

    void loadAndShowPointCloud(const std::string& file_path)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 读取点云文件
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", file_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %s with %zu points", file_path.c_str(), cloud->points.size());

        // 显示点云
        viewer_->removeAllPointClouds();
        viewer_->addPointCloud<pcl::PointXYZRGB>(cloud, "sample_cloud");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_cloud");
        viewer_->spin();
    }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PointCloudViewer>();

    // 指定点云文件路径，例如第10帧
    std::string file_path = "point_cloud_0.pcd"; // 注意：文件名从0开始计数

    node->loadAndShowPointCloud(file_path);

    rclcpp::shutdown();
    return 0;
}