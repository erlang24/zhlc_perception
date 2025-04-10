// Copyright 2021 TIER IV, Inc.
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

#include "lidar_centerpoint/node.hpp"

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/preprocess/pointcloud_densification.hpp>
#include <lidar_centerpoint/ros_utils.hpp>
#include <lidar_centerpoint/utils.hpp>
#include <pcl_ros/transforms.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options), tf_buffer_(this->get_clock())
{
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold", 0.35));                     //得分阈值
  const float circle_nms_dist_threshold =
    static_cast<float>(this->declare_parameter<double>("circle_nms_dist_threshold"));                 // 圆形NMS(非极大值抑制)阈值
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("yaw_norm_thresholds");                              //偏航角归一化阈值
  const std::string densification_world_frame_id =
    this->declare_parameter("densification_world_frame_id", "map");                                   //点云稠密化的世界坐标系 id
  const int densification_num_past_frames =
    this->declare_parameter("densification_num_past_frames", 1);                                      //点云稠密化使用的过去帧数
  const std::string trt_precision = this->declare_parameter("trt_precision", "fp16");                 //TensorRT 推理精度
  const std::string encoder_onnx_path = this->declare_parameter<std::string>("encoder_onnx_path");    //四个模型路径
  const std::string encoder_engine_path =
    this->declare_parameter<std::string>("encoder_engine_path");
  const std::string head_onnx_path = this->declare_parameter<std::string>("head_onnx_path");
  const std::string head_engine_path = this->declare_parameter<std::string>("head_engine_path");
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");                    //检测类别名称列表
  has_twist_ = this->declare_parameter("has_twist", false);                                           //是否使用车辆速度
  const std::size_t point_feature_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("point_feature_size"));            //点云特征维度
  const std::size_t max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("max_voxel_size"));                //最大体素数量
  const auto point_cloud_range = this->declare_parameter<std::vector<double>>("point_cloud_range");   //点云范围 (x_min, y_min, z_min, x_max, y_max, z_max)
  const auto voxel_size = this->declare_parameter<std::vector<double>>("voxel_size");                 //体素大小 (x, y, z)
  const std::size_t downsample_factor =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("downsample_factor"));             //下采样因子
  const std::size_t encoder_in_feature_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("encoder_in_feature_size"));       //Encoder 网络的输入特征维度
  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix");                  //根据面积重映射类别的参数矩阵
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");

  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);                                //参数重映射（传入三个矩阵）

  {
    NMSParams p;
    p.nms_type_ = NMS_TYPE::IoU_BEV;
    p.target_class_names_ =
      this->declare_parameter<std::vector<std::string>>("iou_nms_target_class_names");
    p.search_distance_2d_ = this->declare_parameter<double>("iou_nms_search_distance_2d");
    p.iou_threshold_ = this->declare_parameter<double>("iou_nms_threshold");
    iou_bev_nms_.setParameters(p);                                                                    //配置iou bev nms
  }

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);                  //储存Encoder模型路径和推理精度
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);                           //储存Head模型路径和推理精度
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);                                     //储存点云稠密化相关的配置信息

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"),
      "The size of voxel_size != 3: use the default parameters.");
  }
  // 创建CenterPoint配置对象,包含了模型推理所需的各项参数
  CenterPointConfig config(
    class_names_.size(), point_feature_size, max_voxel_size, point_cloud_range, voxel_size,
    downsample_factor, encoder_in_feature_size, score_threshold, circle_nms_dist_threshold,
    yaw_norm_thresholds);

  // 创建CenterPointTRT检测器实例,传入编码器、头部网络、点云稠密化和配置参数
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);

  // 创建点云订阅器,使用传感器数据QoS策略,只保留最新的一帧数据
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));

  // 创建检测结果发布器,QoS设置为保留1条消息
  objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});

  // 初始化调试工具
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    // 创建计时器,用于性能分析
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    // 创建调试信息发布器
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "lidar_centerpoint");
    // 开始计时循环时间和处理时间
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // 如果build_only参数为true,则只构建TensorRT引擎后关闭节点
  if (this->declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

// 点云回调函数,处理输入的点云数据
void LidarCenterPointNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg)
{
  // 检查是否有订阅者,如果没有则直接返回
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  // 如果启用了调试,记录处理时间
  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing_time", true);
  }

  // 执行目标检测
  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(*input_pointcloud_msg, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  // 将检测到的3D框转换为DetectedObject消息格式
  std::vector<autoware_auto_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_auto_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, obj);
    raw_objects.emplace_back(obj);
  }

  // 创建输出消息
  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_pointcloud_msg->header;
  // 应用IoU BEV NMS进行后处理
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  // 根据面积重新映射检测类别
  detection_class_remapper_.mapClasses(output_msg);

  // 发布检测结果
  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
  }

  // 发布调试信息(处理时间统计)
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace centerpoint

// 注册节点组件
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(centerpoint::LidarCenterPointNode)
