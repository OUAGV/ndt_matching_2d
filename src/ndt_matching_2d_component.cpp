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

// Headers in this package
#include "ndt_matching_2d/ndt_matching_2d_component.hpp"
// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace ndt_matching_2d
{
  NdtMatching2dComponent::NdtMatching2dComponent(const rclcpp::NodeOptions &options)
      : Node("ndt_matching_2d_node", options),
        initial_pose_set(false),
        is_accumulated_cloud_initialized(false),
        accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        current_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  {
    declare_parameter("reference_frame_id", "map");
    get_parameter("reference_frame_id", reference_frame_id);
    declare_parameter("base_frame_id", "base_link");
    get_parameter("base_frame_id", base_frame_id);
    declare_parameter("transform_epsilon", 1.0e-6);
    get_parameter("transform_epsilon", trans_epsilon);
    declare_parameter("step_size", 0.1);
    get_parameter("step_size", stepsize);
    declare_parameter("resolution", 0.1);
    get_parameter("resolution", resolution);
    declare_parameter("max_iterations", 20);
    get_parameter("max_iterations", max_iterations);
    declare_parameter("leafsize_source", 0.03);
    get_parameter("leafsize_source", leafsize_source);
    declare_parameter("leafsize_target", 0.045);
    get_parameter("leafsize_target", leafsize_target);
    declare_parameter("pc_range", 15.0);
    get_parameter("pc_range", pc_range);
    declare_parameter("downsampling_point_bottom_num", 300);
    get_parameter("downsampling_point_bottom_num", downsampling_point_bottom_num);
    declare_parameter("yaw_rate_threshold", 0.23);
    get_parameter("yaw_rate_threshold", yaw_rate_threshold);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    current_relative_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
        "current_pose", 10);
    accumulated_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("accumulated_cloud", 10);

    // subscriptionOptionsを別々に設定すると並列にcallback出来る
    scan_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pose_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&NdtMatching2dComponent::timerCallback, this),
        timer_group);

    // subscriptionOptionsを別々にすると精度がかなり悪くなるため、同じcallback_groupにする
    // こうするとcallbackが同時に呼ばれることがなく、mutexを使わなくても良くなり、なんかうまいこといく
    pose_sub = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&NdtMatching2dComponent::poseCallback, this, std::placeholders::_1), scan_options);
    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&NdtMatching2dComponent::scanCallback, this, std::placeholders::_1), scan_options);
  }

  /**
   * @brief 50msごとにmap->base_linkのtfと/current_poseをpublishする
   *
   */
  void NdtMatching2dComponent::timerCallback()
  {
    std::lock_guard<std::mutex> lock(mtx_);
  }

  /**
   * @brief odomをsubscribeして、初期座標として登録する
   *
   * @param msg
   */
  void NdtMatching2dComponent::poseCallback(nav_msgs::msg::Odometry::SharedPtr msg)
  {

    // 初期位置が設定されていない場合は初期位置を設定
    if (!initial_pose_set)
    {
      initial_pose = msg->pose;
      RCLCPP_INFO(
          get_logger(), "initial_pose set x : %lf y : %lf", initial_pose.pose.position.x,
          initial_pose.pose.position.y);
    }
    else
    {
      std::lock_guard<std::mutex> lock(mtx_);
      // 初期位置が既に設定されているときは、現在位置を更新
      current_relative_pose_.pose.position.x = msg->pose.pose.position.x;
      current_relative_pose_.pose.position.y = msg->pose.pose.position.y;
      current_relative_pose_.pose.position.z = msg->pose.pose.position.z;
      current_relative_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
      current_relative_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
      current_relative_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
      current_relative_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
      current_relative_twist_.angular.z = msg->twist.twist.angular.z;
    }
    initial_pose_set = true;
  }

  /**
   * @brief scanをsubscribeして、NDTを行う
   *
   * @param msg
   */
  void NdtMatching2dComponent::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // RCLCPP_INFO(get_logger(), "scan received");
    if (!initial_pose_set)
    {
      RCLCPP_INFO(get_logger(), "initial_pose is not set");
      return;
    }
    if (accumulated_cloud->points.size() == 0)
    {
      // if (is_accumulated_cloud_initialized)
      if (false)
      {
        RCLCPP_WARN(get_logger(), "accumulated_cloud is empty");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "initializing accumulated cloud");
        setCurrentPointCloudFromScan(msg);
        // 初期cloudを蓄積cloudとして登録
        initialCloudRegistration(current_cloud);
      }
      return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (std::abs(current_relative_twist_.angular.z) >= yaw_rate_threshold)
    {
      RCLCPP_WARN(get_logger(), "yaw_rate is too large");
      return;
    }
    setCurrentPointCloudFromScan(msg);
    std::vector<int> nan_index;
    // NaNを消去しないとNDTがうまくいかないらしい
    pcl::removeNaNFromPointCloud(*current_cloud, *current_cloud, nan_index);

    updateRelativePose(current_cloud, msg->header.stamp);
    publishCurrentRelativePose(
        reference_frame_id, base_frame_id, current_relative_pose_);
    current_relative_pose_pub->publish(current_relative_pose_);
    publishAccumulatedCloud();
  }

  /**
   * @brief 初期位置を設定する
   *
   * @param cloud
   */
  void NdtMatching2dComponent::initialCloudRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    Eigen::Vector3f offset(initial_pose.pose.position.x, initial_pose.pose.position.y, 0.0);
    Eigen::Quaternionf q(
        initial_pose.pose.orientation.w, initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y, initial_pose.pose.orientation.z);
    accumulated_cloud->header.frame_id = reference_frame_id;
    pcl::transformPointCloud(*cloud, *accumulated_cloud, offset, q);
    std::vector<int> nan_index;
    // NaNを消去しないとNDTがうまくいかないらしい
    pcl::removeNaNFromPointCloud(*accumulated_cloud, *accumulated_cloud, nan_index);
    is_accumulated_cloud_initialized = true;
  }

  /**
   * @brief scanをPointCloudに変換する
   *
   * @param msg
   */
  void NdtMatching2dComponent::setCurrentPointCloudFromScan(
      sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    current_cloud->points.clear();
    current_cloud->header.frame_id = base_frame_id;
    current_cloud->height = 1;
    current_cloud->width = msg->ranges.size();
    current_cloud->points.resize(current_cloud->height * current_cloud->width);
    for (int i = 0; i < msg->ranges.size(); i++)
    {
      pcl::PointXYZ point;
      point.x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
      point.y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
      point.z = 0.0;
      current_cloud->points.push_back(point);
    }
  }

  /**
   * @brief 現在の相対位置をpublishする
   *
   * @param frame_id
   * @param child_frame_id
   * @param pose
   */
  void NdtMatching2dComponent::publishCurrentRelativePose(const std::string frame_id, const std::string child_frame_id,
                                                          const geometry_msgs::msg::PoseStamped pose)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.header.stamp = pose.header.stamp;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform.translation.x = pose.pose.position.x;
    transform_stamped.transform.translation.y = pose.pose.position.y;
    transform_stamped.transform.translation.z = pose.pose.position.z;
    transform_stamped.transform.rotation.w = pose.pose.orientation.w;
    transform_stamped.transform.rotation.x = pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = pose.pose.orientation.z;
    // RCLCPP_INFO(get_logger(), "frame_id : %s child_frame_id : %s", frame_id.c_str(), child_frame_id.c_str());
    broadcaster_->sendTransform(transform_stamped);
  }

  void NdtMatching2dComponent::publishAccumulatedCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*accumulated_cloud, cloud_msg);
    cloud_msg.header.frame_id = reference_frame_id;
    accumulated_cloud_pub->publish(cloud_msg);
  }

  /**
   * @brief 現在の相対位置を更新する
   *
   * @param input_cloud
   * @param stamp
   */
  void NdtMatching2dComponent::updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, rclcpp::Time stamp)
  {

    /*initialize*/
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    /*filtering*/
    std::vector<double> range_local{
        -pc_range,
        pc_range,
        -pc_range,
        pc_range};
    std::vector<double> range_global{
        current_relative_pose_.pose.position.x - pc_range,
        current_relative_pose_.pose.position.x + pc_range,
        current_relative_pose_.pose.position.y - pc_range,
        current_relative_pose_.pose.position.y + pc_range};
    PassThroughFilter(current_cloud, current_cloud, range_global);
    PassThroughFilter(accumulated_cloud, accumulated_cloud, range_local);
    downsamplePoints(current_cloud, leafsize_source);
    /*drop out*/
    if (current_cloud->points.empty() || accumulated_cloud->points.empty())
    {
      RCLCPP_WARN(get_logger(), "cloud is empty");
      return;
    }

    // パラメータのセット
    ndt.setTransformationEpsilon(trans_epsilon);
    ndt.setStepSize(stepsize);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iterations);
    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(accumulated_cloud);

    // 初期推定値の設定
    geometry_msgs::msg::Transform transform;
    transform.translation.x = current_relative_pose_.pose.position.x;
    transform.translation.y = current_relative_pose_.pose.position.y;
    transform.translation.z = current_relative_pose_.pose.position.z;
    transform.rotation = current_relative_pose_.pose.orientation;
    Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast<float>();
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // 蓄積点群の方が点群数が少ないとエラーが出るので処理
    if (current_cloud->points.size() > accumulated_cloud->points.size())
    {
      RCLCPP_WARN(get_logger(), "current_cloud->points.size() : %d > accumulated_cloud->points.size() : %d",
                  current_cloud->points.size(), accumulated_cloud->points.size());
      pcl::transformPointCloud(*current_cloud, *current_cloud, mat);
      *accumulated_cloud += *current_cloud;
      return;
    }

    if (containsNaN(current_cloud) || containsNaN(accumulated_cloud))
    {
      RCLCPP_WARN(get_logger(), "cloud contains NaN");
      return;
    };

    // alignの実行
    std::cout << "aligning ..." << std::endl;
    // align = 移動とスコア計算のイテレーション
    // 第1引数は推定後のPointCloudであり,これを蓄積していく。
    // 第2引数は初期推定値
    ndt.align(*output_cloud, mat);
    std::cout << "DONE" << std::endl;

    /*drop out*/
    if (!ndt.hasConverged())
    {
      RCLCPP_WARN(get_logger(), "ndt has not converged");
      return;
    }

    /*print*/
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() << std::endl
              << " score: " << ndt.getFitnessScore() << std::endl;
    std::cout << "ndt.getFinalTransformation()" << std::endl
              << ndt.getFinalTransformation() << std::endl;
    std::cout << "init_guess" << std::endl
              << mat << std::endl;
    std::cout << "ndt.getFinalNumIteration() = " << ndt.getFinalNumIteration() << std::endl;

    // 結果の取得と計算
    Eigen::Matrix4f final_transform = ndt.getFinalTransformation();
    tf2::Matrix3x3 rotation_mat;
    rotation_mat.setValue(
        static_cast<double>(final_transform(0, 0)), static_cast<double>(final_transform(0, 1)),
        static_cast<double>(final_transform(0, 2)), static_cast<double>(final_transform(1, 0)),
        static_cast<double>(final_transform(1, 1)), static_cast<double>(final_transform(1, 2)),
        static_cast<double>(final_transform(2, 0)), static_cast<double>(final_transform(2, 1)),
        static_cast<double>(final_transform(2, 2)));
    tf2::Quaternion quat;
    rotation_mat.getRotation(quat);
    current_relative_pose_.header.stamp = stamp;
    current_relative_pose_.header.frame_id = reference_frame_id;
    current_relative_pose_.pose.position.x = static_cast<double>(final_transform(0, 3));
    current_relative_pose_.pose.position.y = static_cast<double>(final_transform(1, 3));
    current_relative_pose_.pose.position.z = static_cast<double>(final_transform(2, 3));
    current_relative_pose_.pose.orientation.x = quat.x();
    current_relative_pose_.pose.orientation.y = quat.y();
    current_relative_pose_.pose.orientation.z = quat.z();
    current_relative_pose_.pose.orientation.w = quat.w();

    RCLCPP_INFO(
        get_logger(), "current_relative_pose x : %lf y : %lf yaw : %lf", current_relative_pose_.pose.position.x,
        current_relative_pose_.pose.position.y, tf2::getYaw(quat));

    /*register*/
    *accumulated_cloud += *output_cloud;
    downsamplePoints(accumulated_cloud, leafsize_target);
  }

  void NdtMatching2dComponent::downsamplePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize)
  {
    const int before_size = pc->points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pc);
    vg.setLeafSize(static_cast<float>(leafsize), static_cast<float>(leafsize), static_cast<float>(leafsize));
    vg.filter(*tmp);
    // 点を減らしすぎるとalignでセグフォになるため、最低点数を設定
    if (tmp->points.size() < downsampling_point_bottom_num)
    {
      RCLCPP_WARN(get_logger(), "downsampled cloud size is too small");
      return;
    }
    *pc = *tmp;
    RCLCPP_INFO(get_logger(), "downsampled cloud size before : %d after : %d", before_size, pc->points.size());
  }
  void NdtMatching2dComponent::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out,
                                                 std::vector<double> range)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(range[0], range[1]);
    pass.filter(*pc_out);
    pass.setInputCloud(pc_out);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(range[2], range[3]);
    pass.filter(*pc_out);
  }

  bool NdtMatching2dComponent::containsNaN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    for (const auto &point : cloud->points)
    {
      if (!pcl::isFinite(point))
      {
        return true;
      }
    }
    return false;
  }

} // namespace ndt_matching_2d
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_matching_2d::NdtMatching2dComponent)
