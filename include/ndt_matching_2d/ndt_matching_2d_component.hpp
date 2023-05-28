// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "ndt_matching_2d/visibility_control.h"

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <memory> // shared_ptr in pub_
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/utils.h>
namespace ndt_matching_2d
{
  class NdtMatching2dComponent : public rclcpp::Node
  {
  public:
    NDT_MATCHING_2D_PUBLIC
    explicit NdtMatching2dComponent(const rclcpp::NodeOptions &options);
    void initialCloudRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setCurrentPointCloudFromScan(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishCurrentRelativePose(const std::string frame_id, const std::string child_frame_id,
                                    const geometry_msgs::msg::PoseStamped pose);
    void updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, rclcpp::Time stamp);
    void publishAccumulatedCloud();
    void downsamplePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
    void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
    bool containsNaN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  private:
    //  std::shared_ptr<rclcpp::Publisher<perception_msgs::msg::Tracking2D>> pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_relative_pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_cloud_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    bool initial_pose_set;
    std::string reference_frame_id;
    std::string base_frame_id;
    geometry_msgs::msg::PoseWithCovariance initial_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    geometry_msgs::msg::PoseStamped current_relative_pose_;

    // parameters
    // 収束判定の閾値
    double trans_epsilon;
    // ニュートン法のステップサイズ
    double stepsize;
    // ボクセルサイズ[m]
    double resolution;
    // 最大繰り返し回数
    int max_iterations;
    // 蓄積点群用のダウンサンプリングのボクセルサイズ[m]
    double leafsize_source;
    // ターゲット点群用のダウンサンプリングのボクセルサイズ[m]
    double leafsize_target;
    // この値[m]以上の距離の点は除去する
    double pc_range;
  };
} // namespace ndt_matching_2d
