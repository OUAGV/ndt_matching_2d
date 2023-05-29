#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
class PointCloudToPCD : public rclcpp::Node
{
public:
    PointCloudToPCD() : Node("pointcloud_to_pcd")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "accumulated_cloud",
            10,
            std::bind(&PointCloudToPCD::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::io::savePCDFileASCII("/home/tatsuki/colcon_ws/src/ndt_matching_2d/pcd/output.pcd", cloud);
        RCLCPP_INFO(this->get_logger(), "Saved PCD file.");
        }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToPCD>());
    rclcpp::shutdown();
    return 0;
}
