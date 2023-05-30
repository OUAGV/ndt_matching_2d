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
#include <pclomp/ndt_omp.h>
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
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ndt_matching_2d
{
  class NdtLocalization2dComponent : public rclcpp::Node
  {
  public:
    NDT_MATCHING_2D_PUBLIC
    explicit NdtLocalization2dComponent(const rclcpp::NodeOptions &options);
    void setCurrentPointCloudFromScan(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishCurrentRelativePose(const std::string frame_id, const std::string child_frame_id,
                                    const geometry_msgs::msg::PoseStamped pose);
    void updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, rclcpp::Time stamp);
    void downsamplePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
    void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
    bool containsNaN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void poseCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pointcloud_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_relative_pose_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    rclcpp::SubscriptionOptions scan_options;
    rclcpp::SubscriptionOptions pose_options;

    bool initial_pose_set;
    bool is_accumulated_cloud_initialized;
    std::string reference_frame_id;
    std::string base_frame_id;
    std::string odom_frame_id;
    geometry_msgs::msg::PoseWithCovariance initial_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    geometry_msgs::msg::PoseStamped current_relative_pose_;
    geometry_msgs::msg::Twist current_relative_twist_;
    std::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt_;
    std::mutex mtx_;
    // parameters
    // 収束判定の閾値
    double trans_epsilon;
    // ニュートン法のステップサイズ
    double stepsize;
    // ボクセルサイズ[m]
    double resolution;
    // 最大繰り返し回数
    int max_iterations;
    // 蓄積点群用のダウンサンプリングのボクセルサイズ[m] 値が大きいとたくさん点が消える
    double leafsize_source;
    // ターゲット点群用のダウンサンプリングのボクセルサイズ[m] 値が大きいとたくさん点が消える
    double leafsize_target;
    // この値[m]以上の距離の点は除去する
    double pc_range;
    // ダウンサンプリングを行った後の点群の量の下限値
    int downsampling_point_bottom_num;
    // ヨー角速度の絶対値が閾値[rad]を超えるときはNDTを行わない
    // めっちゃ早く回ると0.4くらい、中くらいの速度だと0.2くらい角速度が出る
    double yaw_rate_threshold;
    // ndtの並列実行数
    int omp_num_thread_;
  };
} // namespace ndt_matching_2d
