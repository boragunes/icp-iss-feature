// Copyright (c) 2022 Jonas Mahler

// This file is part of pcl_example.

// pcl_example is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with Foobar. If not, see <https://www.gnu.org/licenses/>. 

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "pcl_example/pcl_example_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>  // Include ICP header

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>



#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav_msgs/msg/path.hpp"
#include <deque>

using std::placeholders::_1;

  

Pcl_Example::Pcl_Example(const rclcpp::NodeOptions &options)
: Node("pcl_example", options),
  tf_broadcaster_(this)  // Initialize tf_broadcaster_ with the Node instance
{
  
  declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_iss_features", "bf_lidar/iss_features");
  declare_parameter<std::string>("topic_odometry", "bf_lidar/odometry");
  declare_parameter<std::string>("topic_aligned_cloud", "bf_lidar/aligned_cloud");
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_iss_features = get_parameter("topic_iss_features").as_string();
  param_topic_odometry = get_parameter("topic_odometry").as_string();
  param_topic_aligned_cloud = get_parameter("topic_aligned_cloud").as_string();
  
  aligned_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_aligned_cloud, 100);
  iss_features_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_iss_features,100 );
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Path>(param_topic_odometry, 100);

  previous_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("bf_lidar/previous_cloud", 100);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("bf_lidar/filtered_cloud", 100);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in, 100, std::bind(&Pcl_Example::topic_callback, this, _1));
  previous_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::deque<geometry_msgs::msg::PoseStamped> path_history_;

}

void Pcl_Example::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_xyz);

    // Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_xyz);
    voxel_grid.setLeafSize(0.3f, 0.3f, 0.3f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid.filter(*cloud_filtered);
    RCLCPP_INFO(this->get_logger(), "point: %ld size", cloud_filtered->points.size());

    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "base_link2";
    filtered_cloud_msg.header.stamp = msg->header.stamp;
    filtered_cloud_publisher_->publish(filtered_cloud_msg);


    if (!previous_cloud_->empty())
    {

        sensor_msgs::msg::PointCloud2 previous_cloud_msg;
        pcl::toROSMsg(*previous_cloud_, previous_cloud_msg);
        previous_cloud_msg.header.frame_id = "base_link2";
        previous_cloud_msg.header.stamp = msg->header.stamp;
        previous_cloud_publisher_->publish(previous_cloud_msg);


        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_filtered);
        icp.setInputTarget(previous_cloud_);
        icp.setMaxCorrespondenceDistance(2.0);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-12);

        pcl::PointCloud<pcl::PointXYZ> final_cloud;

        icp.align(final_cloud);
        RCLCPP_INFO(this->get_logger(), "ICP Converged: %s", icp.hasConverged() ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Fitness Score: %f", icp.getFitnessScore());

        if (icp.hasConverged())
        {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            previous_transfrom_ = transformation * previous_transfrom_;

            // Publish the transformed point cloud
            sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
            pcl::toROSMsg(final_cloud, aligned_cloud_msg);
            aligned_cloud_msg.header.frame_id = "base_link2";
            aligned_cloud_msg.header.stamp = msg->header.stamp;
            aligned_cloud_publisher_->publish(aligned_cloud_msg);

            // Publish odometry path
            geometry_msgs::msg::PoseStamped odom_msg;
            odom_msg.header.frame_id = "map";
            odom_msg.header.stamp = msg->header.stamp;

            Eigen::Quaternionf q(Eigen::Matrix3f(previous_transfrom_.block<3,3>(0,0)));
            odom_msg.pose.position.x = previous_transfrom_(0, 3);
            odom_msg.pose.position.y = previous_transfrom_(1, 3);
            odom_msg.pose.position.z = previous_transfrom_(2, 3);
            odom_msg.pose.orientation.x = q.x();
            odom_msg.pose.orientation.y = q.y();
            odom_msg.pose.orientation.z = q.z();
            odom_msg.pose.orientation.w = q.w();

            path_history_.push_back(odom_msg);

            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = msg->header.stamp;
            path_msg.poses.insert(path_msg.poses.end(), path_history_.begin(), path_history_.end());
            odometry_publisher_->publish(path_msg);

            // Publish the odom_to_map transform
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = msg->header.stamp;
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "base_link2";
            transformStamped.transform.translation.x = previous_transfrom_(0, 3);
            transformStamped.transform.translation.y = previous_transfrom_(1, 3);
            transformStamped.transform.translation.z = previous_transfrom_(2, 3);
            Eigen::Quaternionf q_odom_to_map(Eigen::Matrix3f(previous_transfrom_.block<3,3>(0,0)));
            transformStamped.transform.rotation.x = q_odom_to_map.x();
            transformStamped.transform.rotation.y = q_odom_to_map.y();
            transformStamped.transform.rotation.z = q_odom_to_map.z();
            transformStamped.transform.rotation.w = q_odom_to_map.w();

            tf_broadcaster_.sendTransform(transformStamped);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
    }

    // Update previous cloud
    *previous_cloud_ = *cloud_filtered;

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "Conversion time: %ld microseconds", duration);
}

