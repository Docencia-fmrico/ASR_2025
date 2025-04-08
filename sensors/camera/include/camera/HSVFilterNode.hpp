// Copyright 2023 Intelligent Robotics Lab
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

#ifndef CAMERA__HSVFILTERNODE_HPP_
#define CAMERA__HSVFILTERNODE_HPP_

#include <memory>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "image_geometry/pinhole_camera_model.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera
{

class HSVFilterNode : public rclcpp::Node
{
public:
  HSVFilterNode();

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & image);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);

  cv::Mat1b filter_image(cv::Mat & in, int h, int s, int v, int H, int S, int V);
  void show_image_filtered(const cv::Mat & image, const cv::Mat1b & image_filtered);

  cv::Point2d get_detected_center(const cv::Mat & filtered);
  std::tuple<float, float> get_detected_angles(
    const cv::Point2d & center, std::shared_ptr<image_geometry::PinholeCameraModel> model);
  void publish_detection(
    const sensor_msgs::msg::Image::ConstSharedPtr & image, const cv::Point2d & point,
    const cv::Rect & bbx);

  int h_ {0}, s_ {0}, v_ {0};
  int H_ {180}, S_ {255}, V_ {255};

  int kernel_size_, kernel_shape_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;

  std::shared_ptr<image_geometry::PinholeCameraModel> model_;
};

}  // namespace camera

#endif  // CAMERA__HSVFILTERNODE_HPP_
