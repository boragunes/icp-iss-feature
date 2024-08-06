#ifndef PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_
#define PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <deque>

class Pcl_Example : public rclcpp::Node
{
  public:
    explicit Pcl_Example(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~Pcl_Example() {};

  protected:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr iss_features_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odometry_publisher_;

    std::string param_topic_pointcloud_in;
    std::string param_topic_pointcloud_out;
    std::string param_topic_iss_features;
    std::string param_topic_odometry;
    std::deque<geometry_msgs::msg::PoseStamped> path_history_;

    // Add this line to declare previous_cloud_ as a member variable
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> previous_cloud_;
};

#endif //PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_
