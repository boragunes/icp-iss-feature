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

#include <tf2_ros/transform_broadcaster.h>  // Include tf2_ros header
#include <geometry_msgs/msg/transform_stamped.hpp>  // Include TransformStamped header
#include <tf2_eigen/tf2_eigen.hpp> 

class Pcl_Example : public rclcpp::Node
{
  public:
    explicit Pcl_Example(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~Pcl_Example() {};

  protected:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void transform_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
     rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr iss_features_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr previous_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odometry_publisher_;

    std::string param_topic_pointcloud_in;
    std::string param_topic_pointcloud_out;
    std::string param_topic_iss_features;
    std::string param_topic_odometry;
    std::string param_topic_aligned_cloud;
    std::string param_topic_matlab_transform; 
    std::deque<geometry_msgs::msg::PoseStamped> path_history_;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_history_;

    // Add this line to declare previous_cloud_ as a member variable
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> previous_cloud_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cumulative_cloud_;
    geometry_msgs::msg::TransformStamped latest_transform_;
    Eigen::Matrix4f previous_transfrom_=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f odom_to_map_transform_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f matlab_transform_ = Eigen::Matrix4f::Identity();  // Add this line

    tf2_ros::TransformBroadcaster tf_broadcaster_;  // Add this line
};

#endif //PCL_EXAMPLE__PCL_EXAMPLE_NODE_HPP_
