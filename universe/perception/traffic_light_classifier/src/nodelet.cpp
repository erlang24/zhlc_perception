// Copyright 2023 TIER IV, Inc.
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
#include "traffic_light_classifier/nodelet.hpp"

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

autoware_perception_msgs::msg::TrafficSignalElement convert(
  const tier4_perception_msgs::msg::TrafficLightElement & input)
{
  typedef tier4_perception_msgs::msg::TrafficLightElement OldElem;
  typedef autoware_perception_msgs::msg::TrafficSignalElement NewElem;
  static const std::unordered_map<OldElem::_color_type, NewElem::_color_type> color_map(
    {{OldElem::RED, NewElem::RED},
     {OldElem::AMBER, NewElem::AMBER},
     {OldElem::GREEN, NewElem::GREEN},
     {OldElem::WHITE, NewElem::WHITE}});
  static const std::unordered_map<OldElem::_shape_type, NewElem::_shape_type> shape_map(
    {{OldElem::CIRCLE, NewElem::CIRCLE},
     {OldElem::LEFT_ARROW, NewElem::LEFT_ARROW},
     {OldElem::RIGHT_ARROW, NewElem::RIGHT_ARROW},
     {OldElem::UP_ARROW, NewElem::UP_ARROW},
     {OldElem::UP_LEFT_ARROW, NewElem::UP_LEFT_ARROW},
     {OldElem::UP_RIGHT_ARROW, NewElem::UP_RIGHT_ARROW},
     {OldElem::DOWN_ARROW, NewElem::DOWN_ARROW},
     {OldElem::DOWN_LEFT_ARROW, NewElem::DOWN_LEFT_ARROW},
     {OldElem::DOWN_RIGHT_ARROW, NewElem::DOWN_RIGHT_ARROW},
     {OldElem::CROSS, NewElem::CROSS}});
  static const std::unordered_map<OldElem::_status_type, NewElem::_status_type> status_map(
    {{OldElem::SOLID_OFF, NewElem::SOLID_OFF},
     {OldElem::SOLID_ON, NewElem::SOLID_ON},
     {OldElem::FLASHING, NewElem::FLASHING}});
  // clang-format on

  NewElem output;
  output.color = at_or(color_map, input.color, NewElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, NewElem::UNKNOWN);
  output.status = at_or(status_map, input.status, NewElem::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

namespace traffic_light
{
TrafficLightClassifierNodelet::TrafficLightClassifierNodelet(const rclcpp::NodeOptions & options)
: Node("traffic_light_classifier_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  is_approximate_sync_ = this->declare_parameter("approximate_sync", false);
  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  }

  traffic_signal_array_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::TrafficSignalArray>(
      "~/output/traffic_signals", rclcpp::QoS{1});

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightClassifierNodelet::connectCb, this));

  int classifier_type = this->declare_parameter(
    "classifier_type", static_cast<int>(TrafficLightClassifierNodelet::ClassifierType::HSVFilter));
  if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::HSVFilter) {
    classifier_ptr_ = std::make_shared<ColorClassifier>(this);
  } else if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::CNN) {
#if ENABLE_GPU
    classifier_ptr_ = std::make_shared<CNNClassifier>(this);
#else
    RCLCPP_ERROR(
      this->get_logger(), "please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
  }
}

void TrafficLightClassifierNodelet::connectCb()
{
  // set callbacks only when there are subscribers to this node
  if (
    traffic_signal_array_pub_->get_subscription_count() == 0 &&
    traffic_signal_array_pub_->get_intra_process_subscription_count() == 0) {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightClassifierNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg)
{
  if (classifier_ptr_.use_count() == 0) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'rgb8'.",
      input_image_msg->encoding.c_str());
  }

  tier4_perception_msgs::msg::TrafficSignalArray output_msg;

  output_msg.signals.resize(input_rois_msg->rois.size());

  std::vector<cv::Mat> images;
  for (size_t i = 0; i < input_rois_msg->rois.size(); i++) {
    output_msg.signals[i].traffic_light_id = input_rois_msg->rois.at(i).traffic_light_id;
    const sensor_msgs::msg::RegionOfInterest & roi = input_rois_msg->rois.at(i).roi;
    images.emplace_back(cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
  }
  
  if (!classifier_ptr_->getTrafficSignals(images, output_msg)) {
    RCLCPP_ERROR(this->get_logger(), "failed classify image, abort callback");
    return;
  }
  autoware_perception_msgs::msg::TrafficSignalArray msg_out;
  msg_out.stamp = input_image_msg->header.stamp;

  for (const auto & p : output_msg.signals) {
    autoware_perception_msgs::msg::TrafficSignal msg_out_signal;
    msg_out_signal.traffic_signal_id = p.traffic_light_id;
    for (const auto & ele : p.elements) {
      msg_out_signal.elements.push_back(convert(ele));
    }
    msg_out.signals.push_back(msg_out_signal);
  }
  traffic_signal_array_pub_->publish(msg_out);
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightClassifierNodelet)
